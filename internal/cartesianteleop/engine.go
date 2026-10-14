package cartesianteleop

import (
	"context"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// Leader is the slice of arm.Arm the engine reads and aligns. arm.Arm satisfies it, in
// process or as a gRPC client.
type Leader interface {
	EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error)
	JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error)
	MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error
	Kinematics(ctx context.Context) (referenceframe.Model, error)
	DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error)
}

// Gripper is the slice of gripper.Gripper the engine uses on either side.
type Gripper interface {
	Open(ctx context.Context, extra map[string]interface{}) error
	Grab(ctx context.Context, extra map[string]interface{}) (bool, error)
	DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error)
}

// Config is the resolved engine configuration. Build it with Settings.Resolve.
type Config struct {
	Mapping         Mapping
	RateHz          float64
	RotationEnabled bool
	Filter          FilterConfig
	IK              IKOptions

	LeaderHold      LeaderHold
	ManualOverrides map[string]interface{}
	// AlignOnStart drives the leader to the follower's pose before mirroring (IK + a blocking
	// move). Off by default: the default start captures both arms where they are and mirrors
	// deltas from there, so nothing moves on start and no IK is involved.
	AlignOnStart            bool
	CompensateResidual      bool
	AlignMaxPositionErrorMM float64

	FollowerGripperMode GripperMode
	GripperInvert       bool
	GripperRange        float64
	// GripperDeadbandPct is how much the leader percentage must change before a new gripper
	// command is sent. Default 3: the leader trigger is noisy at the 1 % level, and every
	// command is a blocking move on the follower.
	GripperDeadbandPct float64
	// GripperInMinPct / GripperInMaxPct rescale the leader's raw reading before inversion:
	// raw <= min maps to 0, raw >= max maps to 100. Lowering max makes full opening reachable
	// with less trigger travel. Defaults 0 and 100 (no rescale).
	GripperInMinPct float64
	GripperInMaxPct float64

	MoveTimeout          time.Duration
	MaxConsecutiveErrors int
}

// alignReachToleranceDeg is how far any joint may be from the IK target after the blocking
// align move before Start concludes the arm did not actually move. Generous: it must clear
// the servos' gravity droop (a few degrees) and the profiled move's own stop tolerance, and
// it only needs to catch "did not move at all".
const alignReachToleranceDeg = 10.0

// alignMovedThresholdDeg is the smallest joint change that counts as "the arm moved" when the
// align move fell short: below it the servos are judged not to be driving at all.
const alignMovedThresholdDeg = 2.0

// maxJointGapDeg returns the largest per-joint |a-b| in degrees and which joint it was.
func maxJointGapDeg(a, b []referenceframe.Input) (float64, int) {
	worst, idx := 0.0, 0
	for i := range a {
		if i >= len(b) {
			break
		}
		if d := math.Abs(a[i]-b[i]) * 180 / math.Pi; d > worst {
			worst, idx = d, i
		}
	}
	return worst, idx
}

// Deps are the resources the engine drives. LeaderGripper and FollowerGripper may both be
// nil; Now may be nil for the wall clock.
type Deps struct {
	Leader          Leader
	LeaderGripper   Gripper
	Pipeline        *Pipeline
	FollowerGripper Gripper
	Logger          logging.Logger
	Now             func() time.Time
}

// Alignment is what Start established before mirroring began. With AlignOnStart it records
// the move; without it, it records the capture (LeaderTarget and LeaderInputs are then just
// where the leader was).
type Alignment struct {
	Aligned             bool // true if the leader was driven to the follower's pose
	FollowerPose        spatialmath.Pose
	LeaderTarget        spatialmath.Pose // Mapping.Inverse(FollowerPose)
	LeaderInputs        []referenceframe.Input
	PositionErrorMM     float64 // gap between the mapped leader pose and the follower, after Start
	OrientationErrorDeg float64
	Residual            Residual
}

// Status is a snapshot for DoCommand/CLI reporting.
type Status struct {
	Running           bool    `json:"running"`
	Cycles            uint64  `json:"cycles"`
	Sends             uint64  `json:"sends"`
	Dropped           uint64  `json:"dropped"` // a send was due but one was already in flight
	Deadzoned         uint64  `json:"deadzoned"`
	Rejected          uint64  `json:"rejected"` // outlier rejections
	ConsecutiveErrors int     `json:"consecutive_errors"`
	LastError         string  `json:"last_error"`
	LastRPCMs         float64 `json:"last_rpc_ms"`
	RateHz            float64 `json:"rate_hz"`
	AlignPosErrMM     float64 `json:"align_position_error_mm"`
	AlignOriErrDeg    float64 `json:"align_orientation_error_deg"`
	GripperPct        float64 `json:"gripper_pct"`
}

// Engine runs one leader→follower Cartesian mirror session.
type Engine struct {
	deps Deps
	cfg  Config

	mu      sync.Mutex
	running bool // the mirror loop is alive
	// sessionOpen is true from a successful alignment until Stop: the leader is held and the
	// follower is registered with the pipeline. It outlives running, because a loop that
	// stops itself on errors deliberately leaves the leader held (a hand may be on it).
	sessionOpen bool
	cancel      context.CancelFunc
	wg          sync.WaitGroup
	align       *Alignment
	startOri    spatialmath.Orientation
	filter      *Filter

	cycles, sends, dropped uint64
	consecutiveErrors      int
	lastError              string
	sendErr                error // the most recent failed Move, folded into the next tick's error
	lastRPCNanos           atomic.Int64
	inFlight               atomic.Bool
	sessionLost            atomic.Bool
	lastGripperPct         float64
	gripperSent            bool
	gripperOpen            *bool // open_close mode: last state sent
	// gripperInFlight is the one outstanding follower-gripper command. The xArm gripper's
	// "set" blocks until the jaw arrives (up to seconds), which must not stall the mirror
	// loop, so gripper writes are asynchronous and latest-wins like the pose sends.
	gripperInFlight atomic.Bool
	gripperPending  atomic.Value // float64: a percentage that arrived while one was in flight
}

// New returns an Engine. It does nothing until Start.
func New(deps Deps, cfg Config) *Engine {
	if deps.Logger == nil {
		deps.Logger = logging.NewLogger("cartesian-teleop")
	}
	if deps.Now == nil {
		deps.Now = time.Now
	}
	if cfg.RateHz <= 0 {
		cfg.RateHz = 60
	}
	if cfg.MaxConsecutiveErrors <= 0 {
		cfg.MaxConsecutiveErrors = 10
	}
	if cfg.MoveTimeout <= 0 {
		cfg.MoveTimeout = 500 * time.Millisecond
	}
	if cfg.LeaderHold == "" {
		cfg.LeaderHold = HoldManual
	}
	if cfg.GripperRange <= 0 {
		cfg.GripperRange = 850
	}
	if cfg.GripperDeadbandPct <= 0 {
		cfg.GripperDeadbandPct = 3
	}
	return &Engine{deps: deps, cfg: cfg}
}

// Start aligns the leader to the follower, hands the leader to the driver, registers the
// follower with the pipeline at its current pose, and begins mirroring. Nothing on the
// follower side moves during Start; the leader does.
func (e *Engine) Start(ctx context.Context) error {
	e.mu.Lock()
	defer e.mu.Unlock()
	if e.running {
		return nil
	}
	if e.sessionOpen {
		// The loop died on errors and nobody called Stop: clean up before re-aligning.
		e.deps.Logger.Warn("Previous teleop session ended on errors; releasing it before restarting")
		e.closeSessionLocked(ctx)
	}

	var al *Alignment
	var err error
	if e.cfg.AlignOnStart {
		al, err = e.alignLeader(ctx)
	} else {
		al, err = e.captureLeader(ctx)
	}
	if err != nil {
		return err
	}
	if err := e.deps.Pipeline.Start(ctx, al.FollowerPose); err != nil {
		e.releaseLeader(ctx)
		return err
	}

	e.align = al
	e.sessionOpen = true
	e.startOri = al.FollowerPose.Orientation()
	e.filter = NewFilter(e.cfg.Filter)
	e.cycles, e.sends, e.dropped = 0, 0, 0
	e.consecutiveErrors, e.lastError = 0, ""
	e.gripperSent, e.gripperOpen = false, nil
	e.sessionLost.Store(false)
	e.inFlight.Store(false)

	loopCtx, cancel := context.WithCancel(context.Background())
	e.cancel = cancel
	e.running = true
	e.wg.Add(1)
	go e.runLoop(loopCtx)
	if al.Aligned {
		e.deps.Logger.Infof("Cartesian teleop started at %.0f Hz; leader aligned to %.1f mm / %.1f deg of the follower",
			e.cfg.RateHz, al.PositionErrorMM, al.OrientationErrorDeg)
	} else {
		e.deps.Logger.Infof("Cartesian teleop started at %.0f Hz; mirroring deltas from the leader's current pose "+
			"(%.0f mm / %.0f deg offset absorbed)", e.cfg.RateHz, al.PositionErrorMM, al.OrientationErrorDeg)
	}
	return nil
}

// Align is the alignment performed by the last Start, or nil.
func (e *Engine) Align() *Alignment {
	e.mu.Lock()
	defer e.mu.Unlock()
	return e.align
}

// captureLeader is the default start: both arms stay where they are. The gap between the
// mapped leader pose and the follower becomes the session's Residual, so the first target
// equals the follower's current pose and the leader's motion from here is mirrored as deltas.
// The leader is handed to the driver with the configured hold. Caller holds e.mu.
func (e *Engine) captureLeader(ctx context.Context) (*Alignment, error) {
	followerPose, err := e.deps.Pipeline.CurrentPose(ctx)
	if err != nil {
		return nil, err
	}
	joints, err := e.deps.Leader.JointPositions(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("read leader joints: %w", err)
	}
	// A compliant leader from a previous session is fine to keep, but re-entering with the
	// configured overrides keeps the hold consistent across sessions.
	if _, err := e.deps.Leader.DoCommand(ctx, map[string]interface{}{"command": "exit_manual_mode"}); err != nil {
		e.deps.Logger.Debugf("exit_manual_mode before capture: %v", err)
	}
	if err := e.applyHold(ctx); err != nil {
		return nil, err
	}
	actual, err := e.deps.Leader.EndPosition(ctx, nil)
	if err != nil {
		e.releaseLeader(ctx)
		return nil, fmt.Errorf("read leader pose: %w", err)
	}
	mapped := e.cfg.Mapping.Forward(actual)
	al := &Alignment{
		FollowerPose:        followerPose,
		LeaderTarget:        e.cfg.Mapping.Inverse(followerPose),
		LeaderInputs:        joints,
		PositionErrorMM:     followerPose.Point().Sub(mapped.Point()).Norm(),
		OrientationErrorDeg: orientationError(mapped.Orientation(), followerPose.Orientation()).Norm() * 180 / math.Pi,
		Residual:            ResidualBetween(mapped, followerPose),
	}
	return al, nil
}

// alignLeader drives the leader as close as it can get to the follower's current pose and
// applies the configured hold. Caller holds e.mu.
func (e *Engine) alignLeader(ctx context.Context) (*Alignment, error) {
	model, err := e.deps.Leader.Kinematics(ctx)
	if err != nil {
		return nil, fmt.Errorf("leader kinematics: %w", err)
	}
	followerPose, err := e.deps.Pipeline.CurrentPose(ctx)
	if err != nil {
		return nil, err
	}
	want := e.cfg.Mapping.Inverse(followerPose)
	seed, err := e.deps.Leader.JointPositions(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("read leader joints: %w", err)
	}
	res, err := SolveBestEffort(ctx, model, seed, want, e.cfg.IK)
	if err != nil {
		return nil, fmt.Errorf("leader IK: %w", err)
	}
	if e.cfg.AlignMaxPositionErrorMM > 0 && res.PositionErrorMM > e.cfg.AlignMaxPositionErrorMM {
		return nil, fmt.Errorf("leader cannot get within %.0f mm of the follower's pose (best %.1f mm); "+
			"move the follower nearer the mapped workspace or adjust scale/origin",
			e.cfg.AlignMaxPositionErrorMM, res.PositionErrorMM)
	}

	// A previous session may have left the leader compliant; a compliant arm cannot be
	// driven to a pose. Best effort: a leader that is not an so101 has no such command.
	if _, err := e.deps.Leader.DoCommand(ctx, map[string]interface{}{"command": "exit_manual_mode"}); err != nil {
		e.deps.Logger.Debugf("exit_manual_mode before align: %v", err)
	}
	if err := e.deps.Leader.MoveToJointPositions(ctx, res.Inputs, map[string]interface{}{"wait": true}); err != nil {
		return nil, fmt.Errorf("move leader to the follower's pose: %w", err)
	}
	// A blocking move that returns nil is not proof the arm went anywhere: a servo with torque
	// off, or tripped into overload protection, accepts the goal, reports "not moving", and
	// stays put. Seen on hardware: the engine announced a 0.4 mm alignment while the arm had
	// not moved a degree. So verify from the position read, not the move's return value.
	after, err := e.deps.Leader.JointPositions(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("read leader joints after align move: %w", err)
	}
	if worst, joint := maxJointGapDeg(after, res.Inputs); worst > alignReachToleranceDeg {
		// Two different situations end here. If NO joint moved, the servos are not driving
		// (torque off, overload trip): refuse. If the arm moved but a joint stopped short, it
		// hit a limit the model does not know about -- the arm clamps to its CALIBRATED
		// range, the IK only sees Kinematics()' static limits -- or it drooped under load.
		// That is what the residual compensation below exists for, so accept it and let the
		// position-residual check decide whether the leftover gap is usable.
		if moved, _ := maxJointGapDeg(after, seed); moved < alignMovedThresholdDeg {
			// Either the servos are not driving (torque off, overload trip) or every joint
			// the IK wanted to move was already parked at its calibrated limit (seen on
			// hardware: an elbow at the edge of a narrow recorded range). We cannot tell the
			// two apart from here, so this is strict only when a residual limit is configured.
			msg := fmt.Sprintf("leader align: no joint moved although joint %d is %.1f deg from its target. "+
				"Either torque is off / a servo tripped overload, or the arm is already at its calibrated "+
				"limits for this pose", joint+1, worst)
			if e.cfg.AlignMaxPositionErrorMM > 0 {
				return nil, fmt.Errorf("%s", msg)
			}
			e.deps.Logger.Warn(msg)
		} else {
			e.deps.Logger.Warnf("leader align: joint %d stopped %.1f deg short of its target (calibrated limit or "+
				"droop); compensating with the residual", joint+1, worst)
		}
		res.Inputs = after
	}
	if err := e.applyHold(ctx); err != nil {
		return nil, err
	}

	al := &Alignment{
		Aligned:             true,
		FollowerPose:        followerPose,
		LeaderTarget:        want,
		LeaderInputs:        res.Inputs,
		PositionErrorMM:     res.PositionErrorMM,
		OrientationErrorDeg: res.OrientationErrorDeg,
	}
	actual, err := e.deps.Leader.EndPosition(ctx, nil)
	if err != nil {
		e.releaseLeader(ctx)
		return nil, fmt.Errorf("read leader pose after align: %w", err)
	}
	mapped := e.cfg.Mapping.Forward(actual)
	al.PositionErrorMM = followerPose.Point().Sub(mapped.Point()).Norm()
	al.OrientationErrorDeg = orientationError(mapped.Orientation(), followerPose.Orientation()).Norm() * 180 / math.Pi
	if e.cfg.AlignMaxPositionErrorMM > 0 && al.PositionErrorMM > e.cfg.AlignMaxPositionErrorMM {
		e.releaseLeader(ctx)
		return nil, fmt.Errorf("after the align move the leader maps %.0f mm from the follower's pose "+
			"(limit %.0f): a joint stopped short of what the IK asked for. Move the follower nearer the "+
			"leader's reachable workspace or adjust scale/origin", al.PositionErrorMM, e.cfg.AlignMaxPositionErrorMM)
	}
	if al.PositionErrorMM > 25 || al.OrientationErrorDeg > 10 {
		e.deps.Logger.Warnf("leader align is best effort: it maps %.0f mm / %.0f deg from the follower's pose; "+
			"the gap is absorbed as a constant offset (calibrate the leader with a full range sweep to shrink it)",
			al.PositionErrorMM, al.OrientationErrorDeg)
	}
	if e.cfg.CompensateResidual {
		al.Residual = ResidualBetween(mapped, followerPose)
	}
	return al, nil
}

func (e *Engine) applyHold(ctx context.Context) error {
	// The leader gripper's trigger is an input device: free it so it can be squeezed. The
	// so101 gripper releases it at build time already, but the arm's own set_torque (used by
	// the torque_off hold and its release) re-enables the whole bus, servo 6 included.
	if e.deps.LeaderGripper != nil {
		if _, err := e.deps.LeaderGripper.DoCommand(ctx, map[string]interface{}{"command": "set_torque", "enable": false}); err != nil {
			e.deps.Logger.Warnf("could not release the leader gripper's torque (its trigger may be locked): %v", err)
		}
	}
	switch e.cfg.LeaderHold {
	case HoldManual:
		cmd := map[string]interface{}{"command": "enter_manual_mode"}
		for k, v := range e.cfg.ManualOverrides {
			cmd[k] = v
		}
		if _, err := e.deps.Leader.DoCommand(ctx, cmd); err != nil {
			return fmt.Errorf("enter leader manual mode: %w", err)
		}
	case HoldTorqueOff:
		if _, err := e.deps.Leader.DoCommand(ctx, map[string]interface{}{"command": "set_torque", "enable": false}); err != nil {
			return fmt.Errorf("disable leader torque: %w", err)
		}
	case HoldStiff:
	}
	return nil
}

// releaseLeader undoes applyHold and leaves the leader holding its current pose. Torque is
// re-enabled only after the goal register is seeded with the present position, because a
// bare torque enable resumes driving toward whatever goal the servo last held.
func (e *Engine) releaseLeader(ctx context.Context) {
	switch e.cfg.LeaderHold {
	case HoldManual:
		if _, err := e.deps.Leader.DoCommand(ctx, map[string]interface{}{"command": "exit_manual_mode"}); err != nil {
			e.deps.Logger.Warnf("exit leader manual mode: %v", err)
		}
	case HoldTorqueOff:
		if cur, err := e.deps.Leader.JointPositions(ctx, nil); err != nil {
			e.deps.Logger.Warnf("seed leader goal before torque enable: %v (leaving torque off)", err)
			return
		} else if err := e.deps.Leader.MoveToJointPositions(ctx, cur, map[string]interface{}{"wait": false}); err != nil {
			e.deps.Logger.Warnf("seed leader goal before torque enable: %v (leaving torque off)", err)
			return
		}
		if _, err := e.deps.Leader.DoCommand(ctx, map[string]interface{}{"command": "set_torque", "enable": true}); err != nil {
			e.deps.Logger.Warnf("re-enable leader torque: %v", err)
		}
	case HoldStiff:
	}
}

// Stop ends the session: the loop stops, the follower is removed from the pipeline and holds
// where it is, and the leader is released from its hold.
func (e *Engine) Stop(ctx context.Context) error {
	e.mu.Lock()
	defer e.mu.Unlock()
	if !e.running && !e.sessionOpen {
		return nil
	}
	err := e.closeSessionLocked(ctx)
	e.deps.Logger.Info("Cartesian teleop stopped")
	return err
}

// closeSessionLocked stops the loop if it is alive, removes the follower from the pipeline
// and releases the leader. Caller holds e.mu; the loop never takes e.mu while it is being
// waited on except inside tick, which returns promptly.
func (e *Engine) closeSessionLocked(ctx context.Context) error {
	if e.cancel != nil {
		e.cancel()
	}
	e.mu.Unlock()
	e.wg.Wait()
	e.mu.Lock()

	var firstErr error
	if err := e.deps.Pipeline.Stop(ctx); err != nil && !IsSessionLost(err) {
		firstErr = err
		e.deps.Logger.Warnf("%v", err)
	}
	e.releaseLeader(ctx)
	e.running, e.sessionOpen = false, false
	return firstErr
}

// Status returns a snapshot of the session.
func (e *Engine) Status() Status {
	e.mu.Lock()
	defer e.mu.Unlock()
	st := Status{
		Running:           e.running,
		Cycles:            e.cycles,
		Sends:             e.sends,
		Dropped:           e.dropped,
		ConsecutiveErrors: e.consecutiveErrors,
		LastError:         e.lastError,
		LastRPCMs:         float64(e.lastRPCNanos.Load()) / 1e6,
		RateHz:            e.cfg.RateHz,
		GripperPct:        e.lastGripperPct,
	}
	if e.filter != nil {
		st.Deadzoned, st.Rejected = e.filter.Deadzoned, e.filter.Rejected
	}
	if e.align != nil {
		st.AlignPosErrMM, st.AlignOriErrDeg = e.align.PositionErrorMM, e.align.OrientationErrorDeg
	}
	return st
}

func (e *Engine) runLoop(ctx context.Context) {
	defer e.wg.Done()
	ticker := time.NewTicker(time.Duration(float64(time.Second) / e.cfg.RateHz))
	defer ticker.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			if e.tick(ctx) {
				return
			}
		}
	}
}

// tick runs one cycle and reports whether the loop should stop. A failed Move is
// asynchronous, so it is folded into the error of the tick that first sees it.
func (e *Engine) tick(ctx context.Context) bool {
	var err error
	if !e.sessionLost.Load() {
		err = e.syncOnce(ctx)
	}

	e.mu.Lock()
	defer e.mu.Unlock()
	if se := e.sendErr; se != nil {
		e.sendErr = nil
		if err == nil {
			err = se
		}
	}
	if e.sessionLost.Load() {
		err = fmt.Errorf("teleop pipeline session lost: %w", err)
	}
	if err != nil {
		e.consecutiveErrors++
		e.lastError = err.Error()
		e.deps.Logger.Warnf("Cartesian teleop cycle error (%d/%d): %v", e.consecutiveErrors, e.cfg.MaxConsecutiveErrors, err)
		if e.sessionLost.Load() || e.consecutiveErrors >= e.cfg.MaxConsecutiveErrors {
			e.deps.Logger.Errorf("Cartesian teleop loop stopping: %v", err)
			e.running = false
			// The leader is still held; a Stop from the caller releases it. We do not
			// release here because the hand may be on the arm.
			return true
		}
		return false
	}
	e.consecutiveErrors, e.lastError = 0, ""
	e.cycles++
	return false
}

// syncOnce reads the leader, maps and filters the target, and hands it to the pipeline
// with at most one move in flight (latest wins, like the vive module and the mobile app).
func (e *Engine) syncOnce(ctx context.Context) error {
	pose, err := e.deps.Leader.EndPosition(ctx, nil)
	if err != nil {
		return fmt.Errorf("read leader pose: %w", err)
	}
	target := e.MapTarget(pose)

	e.mu.Lock()
	filter := e.filter
	e.mu.Unlock()
	send, ok := filter.Step(e.deps.Now(), target)
	if ok {
		if e.inFlight.CompareAndSwap(false, true) {
			e.mu.Lock()
			e.sends++
			e.mu.Unlock()
			go e.sendMove(send)
		} else {
			e.mu.Lock()
			e.dropped++
			e.mu.Unlock()
		}
	}
	return e.mirrorGripper(ctx)
}

// MapTarget turns a leader end-effector pose into the follower target for this session:
// mapping, alignment residual, and the rotation lock when rotation is disabled.
func (e *Engine) MapTarget(leader spatialmath.Pose) spatialmath.Pose {
	e.mu.Lock()
	al, startOri := e.align, e.startOri
	e.mu.Unlock()
	target := e.cfg.Mapping.Forward(leader)
	if al != nil {
		target = al.Residual.Apply(target)
	}
	if !e.cfg.RotationEnabled && startOri != nil {
		target = spatialmath.NewPose(target.Point(), startOri)
	}
	return target
}

func (e *Engine) sendMove(target spatialmath.Pose) {
	defer e.inFlight.Store(false)
	ctx, cancel := context.WithTimeout(context.Background(), e.cfg.MoveTimeout)
	defer cancel()
	start := time.Now()
	err := e.deps.Pipeline.Move(ctx, target)
	e.lastRPCNanos.Store(time.Since(start).Nanoseconds())
	if err != nil {
		if IsSessionLost(err) {
			e.sessionLost.Store(true)
		}
		e.mu.Lock()
		e.sendErr = err
		e.mu.Unlock()
	}
}

// mirrorGripper reads the leader gripper and drives the follower gripper on change. The
// read is synchronous (one bus transaction); the write is handed to a goroutine, because
// the xArm gripper's "set" polls until the jaw arrives and would otherwise stall every
// mirror cycle for the jaw's travel time.
func (e *Engine) mirrorGripper(ctx context.Context) error {
	if e.deps.LeaderGripper == nil || e.deps.FollowerGripper == nil || e.cfg.FollowerGripperMode == GripperModeOff {
		return nil
	}
	resp, err := e.deps.LeaderGripper.DoCommand(ctx, map[string]interface{}{"command": "get_position"})
	if err != nil {
		return fmt.Errorf("read leader gripper: %w", err)
	}
	pct, ok := resp["position_percentage"].(float64)
	if !ok {
		return fmt.Errorf("leader gripper get_position missing position_percentage")
	}
	if hi := e.cfg.GripperInMaxPct; hi > e.cfg.GripperInMinPct {
		pct = (pct - e.cfg.GripperInMinPct) / (hi - e.cfg.GripperInMinPct) * 100
	}
	pct = math.Max(0, math.Min(100, pct))
	if e.cfg.GripperInvert {
		pct = 100 - pct
	}

	e.mu.Lock()
	changed := !e.gripperSent || math.Abs(pct-e.lastGripperPct) >= e.cfg.GripperDeadbandPct
	if changed {
		e.lastGripperPct, e.gripperSent = pct, true
	}
	e.mu.Unlock()
	if !changed {
		return nil
	}
	if e.gripperInFlight.CompareAndSwap(false, true) {
		go e.writeGripper(pct)
	} else {
		e.gripperPending.Store(pct) // latest wins; sent when the current write returns
	}
	return nil
}

// writeGripper sends pct to the follower gripper, then drains any value that arrived
// while it was blocked. Errors are folded into the next tick like pose-send errors.
func (e *Engine) writeGripper(pct float64) {
	defer e.gripperInFlight.Store(false)
	for {
		ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
		err := e.writeGripperOnce(ctx, pct)
		cancel()
		if err != nil {
			e.mu.Lock()
			e.sendErr = err
			e.mu.Unlock()
		}
		next := e.gripperPending.Swap(float64(math.NaN()))
		if next == nil {
			return
		}
		if v, ok := next.(float64); !ok || math.IsNaN(v) {
			return
		} else {
			pct = v
		}
	}
}

func (e *Engine) writeGripperOnce(ctx context.Context, pct float64) error {
	switch e.cfg.FollowerGripperMode {
	case GripperModeXArm:
		steps := math.Round(pct / 100 * e.cfg.GripperRange)
		if _, err := e.deps.FollowerGripper.DoCommand(ctx, map[string]interface{}{"set": steps}); err != nil {
			return fmt.Errorf("write follower gripper: %w", err)
		}
	case GripperModeOpenClose:
		e.mu.Lock()
		prev := e.gripperOpen
		e.mu.Unlock()
		var want *bool
		switch {
		case pct > 60 && (prev == nil || !*prev):
			t := true
			want = &t
		case pct < 40 && (prev == nil || *prev):
			f := false
			want = &f
		}
		if want == nil {
			return nil
		}
		if *want {
			if err := e.deps.FollowerGripper.Open(ctx, nil); err != nil {
				return fmt.Errorf("open follower gripper: %w", err)
			}
		} else if _, err := e.deps.FollowerGripper.Grab(ctx, nil); err != nil {
			return fmt.Errorf("close follower gripper: %w", err)
		}
		e.mu.Lock()
		e.gripperOpen = want
		e.mu.Unlock()
	}
	return nil
}
