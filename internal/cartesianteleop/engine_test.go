package cartesianteleop

import (
	"context"
	"encoding/json"
	"errors"
	"sync"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"
	pb "go.viam.com/api/service/motion/v1"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
	"google.golang.org/protobuf/encoding/protojson"
)

// fakeLeader is an arm whose joints are what they were last commanded to; EndPosition is
// real forward kinematics on the so101 model, so alignment is exercised end to end.
type fakeLeader struct {
	mu       sync.Mutex
	model    referenceframe.Model
	joints   []referenceframe.Input
	commands []map[string]interface{}
	moves    []map[string]interface{}
	failRead error
	// clampMax, if set, caps each commanded joint like the real arm clamping to its
	// calibrated range: the move "succeeds" but stops short.
	clampMax []referenceframe.Input
	// limp makes MoveToJointPositions accept the goal and return nil without changing the
	// joints, which is what a torque-off or overload-tripped servo bus does.
	limp bool
}

func newFakeLeader(t *testing.T, joints []referenceframe.Input) *fakeLeader {
	return &fakeLeader{model: leaderModel(t), joints: joints}
}

func (f *fakeLeader) EndPosition(context.Context, map[string]interface{}) (spatialmath.Pose, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	if f.failRead != nil {
		return nil, f.failRead
	}
	return f.model.Transform(f.joints)
}

func (f *fakeLeader) JointPositions(context.Context, map[string]interface{}) ([]referenceframe.Input, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	return append([]referenceframe.Input(nil), f.joints...), nil
}

func (f *fakeLeader) MoveToJointPositions(_ context.Context, p []referenceframe.Input, extra map[string]interface{}) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.moves = append(f.moves, extra)
	if !f.limp {
		q := append([]referenceframe.Input(nil), p...)
		for i := range q {
			if f.clampMax != nil && q[i] > f.clampMax[i] {
				q[i] = f.clampMax[i]
			}
		}
		f.joints = q
	}
	return nil
}

func (f *fakeLeader) Kinematics(context.Context) (referenceframe.Model, error) { return f.model, nil }

func (f *fakeLeader) DoCommand(_ context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.commands = append(f.commands, cmd)
	return map[string]interface{}{}, nil
}

func (f *fakeLeader) commandNames() []string {
	f.mu.Lock()
	defer f.mu.Unlock()
	var out []string
	for _, c := range f.commands {
		out = append(out, c["command"].(string))
	}
	return out
}

func (f *fakeLeader) setJoints(j []referenceframe.Input) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.joints = j
}

// fakeMotion records the teleop DoCommands and serves a fixed follower pose.
type fakeMotion struct {
	mu       sync.Mutex
	pose     spatialmath.Pose
	starts   []*pb.MoveRequest
	moves    []*commonpb.PoseInFrame
	stops    int
	moveErr  error
	moveWait chan struct{} // if set, Move blocks until closed
}

func (m *fakeMotion) GetPose(_ context.Context, name, dst string, _ []*referenceframe.LinkInFrame, _ map[string]interface{}) (*referenceframe.PoseInFrame, error) {
	return referenceframe.NewPoseInFrame(dst, m.pose), nil
}

func (m *fakeMotion) DoCommand(_ context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if s, ok := cmd["teleop_start"].(string); ok {
		var req pb.MoveRequest
		if err := protojson.Unmarshal([]byte(s), &req); err != nil {
			return nil, err
		}
		m.mu.Lock()
		m.starts = append(m.starts, &req)
		m.mu.Unlock()
		return map[string]interface{}{"teleop_start": true}, nil
	}
	if s, ok := cmd["teleop_move"].(string); ok {
		if m.moveWait != nil {
			<-m.moveWait
		}
		if m.moveErr != nil {
			return nil, m.moveErr
		}
		var pif commonpb.PoseInFrame
		if err := protojson.Unmarshal([]byte(s), &pif); err != nil {
			return nil, err
		}
		if _, ok := cmd["component_name"].(string); !ok {
			return nil, errors.New("component_name missing")
		}
		m.mu.Lock()
		m.moves = append(m.moves, &pif)
		m.mu.Unlock()
		return map[string]interface{}{"teleop_move": true}, nil
	}
	if _, ok := cmd["teleop_stop"]; ok {
		m.mu.Lock()
		m.stops++
		m.mu.Unlock()
		return map[string]interface{}{"teleop_stop": true}, nil
	}
	if _, ok := cmd["teleop_status"]; ok {
		return map[string]interface{}{"teleop_status": map[string]interface{}{"running": true}}, nil
	}
	return nil, errors.New("unknown command")
}

func (m *fakeMotion) lastMove() spatialmath.Pose {
	m.mu.Lock()
	defer m.mu.Unlock()
	if len(m.moves) == 0 {
		return nil
	}
	return referenceframe.ProtobufToPoseInFrame(m.moves[len(m.moves)-1]).Pose()
}

func (m *fakeMotion) moveCount() int {
	m.mu.Lock()
	defer m.mu.Unlock()
	return len(m.moves)
}

type fakeGripper struct {
	mu     sync.Mutex
	pct    float64
	sets   []float64
	open   []bool
	block  chan struct{} // if set, DoCommand "set" blocks until closed (an xArm jaw in motion)
	torque *bool         // last set_torque enable, if any
}

func (g *fakeGripper) Open(context.Context, map[string]interface{}) error {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.open = append(g.open, true)
	return nil
}

func (g *fakeGripper) Grab(context.Context, map[string]interface{}) (bool, error) {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.open = append(g.open, false)
	return true, nil
}

func (g *fakeGripper) DoCommand(_ context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	g.mu.Lock()
	defer g.mu.Unlock()
	if cmd["command"] == "get_position" {
		return map[string]interface{}{"position_percentage": g.pct}, nil
	}
	if cmd["command"] == "set_torque" {
		v, _ := cmd["enable"].(bool)
		g.torque = &v
		return map[string]interface{}{}, nil
	}
	if v, ok := cmd["set"].(float64); ok {
		g.sets = append(g.sets, v)
		if g.block != nil {
			g.mu.Unlock()
			<-g.block
			g.mu.Lock()
		}
		return map[string]interface{}{}, nil
	}
	return nil, errors.New("unknown")
}

// followerPoseReachableBy returns a follower pose the leader can reach exactly under m, so
// alignment tests can distinguish "aligned" from "compensated".
func followerPoseReachableBy(t *testing.T, leader *fakeLeader, m Mapping, joints []referenceframe.Input) spatialmath.Pose {
	t.Helper()
	p, err := leader.model.Transform(joints)
	test.That(t, err, test.ShouldBeNil)
	return m.Forward(p)
}

// captureModeTests flips newTestEngine to the default (capture) start for the tests that
// set it; they restore it with defer. Everything else was written against the align path.
var captureModeTests bool

func newTestEngine(t *testing.T, leader *fakeLeader, motion *fakeMotion, cfg Config) *Engine {
	t.Helper()
	if cfg.RateHz == 0 {
		cfg.RateHz = 200
	}
	if !captureModeTests {
		cfg.AlignOnStart = true
	}
	cfg.Filter = FilterConfig{} // deterministic: no smoothing, no deadzone
	return New(Deps{
		Leader:   leader,
		Pipeline: NewPipeline(motion, "builtin", "xarm-gripper"),
		Logger:   logging.NewTestLogger(t),
	}, cfg)
}

func waitFor(t *testing.T, cond func() bool) {
	t.Helper()
	deadline := time.Now().Add(3 * time.Second)
	for time.Now().Before(deadline) {
		if cond() {
			return
		}
		time.Sleep(5 * time.Millisecond)
	}
	t.Fatal("condition not met in time")
}

func TestStartMovesTheLeaderToTheFollowerAndNotTheOtherWayRound(t *testing.T) {
	m := Mapping{Scale: 2, Origin: spatialmath.NewPose(r3.Vector{X: 300}, ov(0, 0, 1, 90))}
	leader := newFakeLeader(t, []referenceframe.Input{0, 0, 0, 0, 0})
	target := []referenceframe.Input{0.3, -0.5, 0.8, 0.2, 0.1}
	motion := &fakeMotion{pose: followerPoseReachableBy(t, leader, m, target)}
	e := newTestEngine(t, leader, motion, Config{Mapping: m, LeaderHold: HoldManual, CompensateResidual: true, RotationEnabled: true})

	ctx := context.Background()
	test.That(t, e.Start(ctx), test.ShouldBeNil)
	defer func() { test.That(t, e.Stop(ctx), test.ShouldBeNil) }()

	al := e.Align()
	test.That(t, al, test.ShouldNotBeNil)
	test.That(t, al.PositionErrorMM, test.ShouldBeLessThan, 0.5)
	// The leader was driven there, blocking.
	test.That(t, len(leader.moves), test.ShouldEqual, 1)
	test.That(t, leader.moves[0]["wait"], test.ShouldEqual, true)
	for i := range target {
		test.That(t, leader.joints[i], test.ShouldAlmostEqual, target[i], 0.01)
	}
	// Then handed to the driver in manual mode, after any stale manual session was exited.
	test.That(t, leader.commandNames(), test.ShouldResemble, []string{"exit_manual_mode", "enter_manual_mode"})
	// The pipeline was registered AT the follower's current pose, so the follower did not move.
	test.That(t, len(motion.starts), test.ShouldEqual, 1)
	test.That(t, motion.starts[0].ComponentName, test.ShouldEqual, "xarm-gripper")
	test.That(t, motion.starts[0].Name, test.ShouldEqual, "builtin")
	start := referenceframe.ProtobufToPoseInFrame(motion.starts[0].Destination).Pose()
	test.That(t, spatialmath.PoseAlmostCoincidentEps(start, motion.pose, 1e-6), test.ShouldBeTrue)
}

func TestFirstMirroredTargetEqualsTheFollowersPoseEvenWhenTheLeaderCannotReachIt(t *testing.T) {
	m := Mapping{Scale: 2}
	leader := newFakeLeader(t, []referenceframe.Input{0, 0, 0, 0, 0})
	reachable := followerPoseReachableBy(t, leader, m, []referenceframe.Input{0.2, -0.4, 0.7, 0.3, 0})
	// Twist the follower pose about an axis the 5-DoF leader cannot follow.
	unreachable := spatialmath.Compose(reachable, spatialmath.NewPoseFromOrientation(ov(1, 0, 0, 0)))
	motion := &fakeMotion{pose: unreachable}
	e := newTestEngine(t, leader, motion, Config{Mapping: m, LeaderHold: HoldStiff, CompensateResidual: true, RotationEnabled: true})

	ctx := context.Background()
	test.That(t, e.Start(ctx), test.ShouldBeNil)
	defer func() { test.That(t, e.Stop(ctx), test.ShouldBeNil) }()
	test.That(t, e.Align().OrientationErrorDeg, test.ShouldBeGreaterThan, 1)

	waitFor(t, func() bool { return motion.moveCount() >= 1 })
	first := motion.lastMove()
	test.That(t, spatialmath.PoseAlmostCoincidentEps(first, unreachable, 1e-3), test.ShouldBeTrue)
	test.That(t, spatialmath.OrientationAlmostEqualEps(first.Orientation(), unreachable.Orientation(), 1e-4), test.ShouldBeTrue)
}

func TestWithoutCompensationTheFirstTargetCarriesTheAlignResidual(t *testing.T) {
	m := Mapping{Scale: 2}
	leader := newFakeLeader(t, []referenceframe.Input{0, 0, 0, 0, 0})
	reachable := followerPoseReachableBy(t, leader, m, []referenceframe.Input{0.2, -0.4, 0.7, 0.3, 0})
	unreachable := spatialmath.Compose(reachable, spatialmath.NewPoseFromOrientation(ov(1, 0, 0, 0)))
	motion := &fakeMotion{pose: unreachable}
	e := newTestEngine(t, leader, motion, Config{Mapping: m, LeaderHold: HoldStiff, CompensateResidual: false, RotationEnabled: true})

	ctx := context.Background()
	test.That(t, e.Start(ctx), test.ShouldBeNil)
	defer func() { test.That(t, e.Stop(ctx), test.ShouldBeNil) }()
	waitFor(t, func() bool { return motion.moveCount() >= 1 })
	first := motion.lastMove()
	test.That(t, spatialmath.OrientationAlmostEqualEps(first.Orientation(), unreachable.Orientation(), 1e-3), test.ShouldBeFalse)
}

func TestMirrorTracksLeaderMotionThroughTheMapping(t *testing.T) {
	m := Mapping{Scale: 2, Origin: spatialmath.NewPose(r3.Vector{Y: 100}, spatialmath.NewZeroOrientation())}
	leader := newFakeLeader(t, []referenceframe.Input{0, 0, 0, 0, 0})
	home := []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0}
	motion := &fakeMotion{pose: followerPoseReachableBy(t, leader, m, home)}
	e := newTestEngine(t, leader, motion, Config{Mapping: m, LeaderHold: HoldStiff, CompensateResidual: true, RotationEnabled: true})

	ctx := context.Background()
	test.That(t, e.Start(ctx), test.ShouldBeNil)
	defer func() { test.That(t, e.Stop(ctx), test.ShouldBeNil) }()

	moved := []referenceframe.Input{0.4, -0.3, 0.6, 0.2, 0}
	leader.setJoints(moved)
	expect := followerPoseReachableBy(t, leader, m, moved)
	waitFor(t, func() bool {
		last := motion.lastMove()
		return last != nil && spatialmath.PoseAlmostCoincidentEps(last, expect, 0.5)
	})
}

func TestRotationDisabledHoldsTheFollowersStartOrientation(t *testing.T) {
	m := Mapping{Scale: 1}
	leader := newFakeLeader(t, []referenceframe.Input{0, 0, 0, 0, 0})
	home := []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0}
	motion := &fakeMotion{pose: followerPoseReachableBy(t, leader, m, home)}
	e := newTestEngine(t, leader, motion, Config{Mapping: m, LeaderHold: HoldStiff, CompensateResidual: true, RotationEnabled: false})

	ctx := context.Background()
	test.That(t, e.Start(ctx), test.ShouldBeNil)
	defer func() { test.That(t, e.Stop(ctx), test.ShouldBeNil) }()

	leader.setJoints([]referenceframe.Input{0.1, -0.3, 0.6, 0.2, 1.2}) // roll the wrist
	waitFor(t, func() bool { return motion.moveCount() >= 3 })
	last := motion.lastMove()
	test.That(t, spatialmath.OrientationAlmostEqualEps(last.Orientation(), motion.pose.Orientation(), 1e-4), test.ShouldBeTrue)
	// And the wrist roll really did change the leader's orientation, so the lock did something.
	rolled, err := leader.model.Transform(leader.joints)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, spatialmath.OrientationAlmostEqualEps(rolled.Orientation(), motion.pose.Orientation(), 0.5), test.ShouldBeFalse)
}

func TestOnlyOneMoveIsInFlightAndTheRestAreDropped(t *testing.T) {
	m := Mapping{}
	leader := newFakeLeader(t, []referenceframe.Input{0, 0, 0, 0, 0})
	motion := &fakeMotion{pose: followerPoseReachableBy(t, leader, m, []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0}), moveWait: make(chan struct{})}
	e := newTestEngine(t, leader, motion, Config{Mapping: m, LeaderHold: HoldStiff, RateHz: 500})

	ctx := context.Background()
	test.That(t, e.Start(ctx), test.ShouldBeNil)
	waitFor(t, func() bool { return e.Status().Dropped >= 5 })
	test.That(t, e.Status().Sends, test.ShouldEqual, 1)
	close(motion.moveWait)
	test.That(t, e.Stop(ctx), test.ShouldBeNil)
}

func TestSessionLostStopsTheLoopButLeavesTheLeaderHeld(t *testing.T) {
	m := Mapping{}
	leader := newFakeLeader(t, []referenceframe.Input{0, 0, 0, 0, 0})
	motion := &fakeMotion{
		pose:    followerPoseReachableBy(t, leader, m, []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0}),
		moveErr: errors.New("teleop pipeline is not running; call teleop_start first"),
	}
	e := newTestEngine(t, leader, motion, Config{Mapping: m, LeaderHold: HoldManual})

	ctx := context.Background()
	test.That(t, e.Start(ctx), test.ShouldBeNil)
	waitFor(t, func() bool { return !e.Status().Running })
	test.That(t, e.Status().LastError, test.ShouldContainSubstring, "session lost")
	// Still in manual mode: nobody has released the hand's arm.
	test.That(t, leader.commandNames(), test.ShouldResemble, []string{"exit_manual_mode", "enter_manual_mode"})
	test.That(t, e.Stop(ctx), test.ShouldBeNil)
	test.That(t, leader.commandNames()[len(leader.commandNames())-1], test.ShouldEqual, "exit_manual_mode")
}

func TestStopReleasesTheLeaderAndTheFollower(t *testing.T) {
	m := Mapping{}
	leader := newFakeLeader(t, []referenceframe.Input{0, 0, 0, 0, 0})
	motion := &fakeMotion{pose: followerPoseReachableBy(t, leader, m, []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0})}
	e := newTestEngine(t, leader, motion, Config{Mapping: m, LeaderHold: HoldTorqueOff})

	ctx := context.Background()
	test.That(t, e.Start(ctx), test.ShouldBeNil)
	test.That(t, leader.commandNames(), test.ShouldResemble, []string{"exit_manual_mode", "set_torque"})
	test.That(t, leader.commands[1]["enable"], test.ShouldEqual, false)

	test.That(t, e.Stop(ctx), test.ShouldBeNil)
	test.That(t, motion.stops, test.ShouldEqual, 1)
	// Torque back on, and only after a goal seed at the present position.
	test.That(t, len(leader.moves), test.ShouldEqual, 2)
	test.That(t, leader.moves[1]["wait"], test.ShouldEqual, false)
	names := leader.commandNames()
	test.That(t, names[len(names)-1], test.ShouldEqual, "set_torque")
	test.That(t, leader.commands[len(leader.commands)-1]["enable"], test.ShouldEqual, true)
	test.That(t, e.Status().Running, test.ShouldBeFalse)
}

func TestStartRefusesAnAlignmentThatIsTooFarOff(t *testing.T) {
	m := Mapping{}
	leader := newFakeLeader(t, []referenceframe.Input{0, 0, 0, 0, 0})
	motion := &fakeMotion{pose: spatialmath.NewPose(r3.Vector{X: 3000}, spatialmath.NewZeroOrientation())}
	e := newTestEngine(t, leader, motion, Config{Mapping: m, LeaderHold: HoldManual, AlignMaxPositionErrorMM: 100})

	err := e.Start(context.Background())
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "cannot get within")
	test.That(t, len(leader.moves), test.ShouldEqual, 0) // the leader was not moved
	test.That(t, len(motion.starts), test.ShouldEqual, 0)
	test.That(t, e.Status().Running, test.ShouldBeFalse)
}

func TestGripperMirrorsOnChangeOnly(t *testing.T) {
	m := Mapping{}
	leader := newFakeLeader(t, []referenceframe.Input{0, 0, 0, 0, 0})
	motion := &fakeMotion{pose: followerPoseReachableBy(t, leader, m, []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0})}
	lg := &fakeGripper{pct: 80}
	fg := &fakeGripper{}
	e := New(Deps{
		Leader: leader, LeaderGripper: lg, FollowerGripper: fg,
		Pipeline: NewPipeline(motion, "builtin", "xarm-gripper"),
		Logger:   logging.NewTestLogger(t),
	}, Config{Mapping: m, LeaderHold: HoldStiff, RateHz: 200, FollowerGripperMode: GripperModeXArm, GripperRange: 850})

	ctx := context.Background()
	test.That(t, e.Start(ctx), test.ShouldBeNil)
	defer func() { test.That(t, e.Stop(ctx), test.ShouldBeNil) }()
	waitFor(t, func() bool { return e.Status().Cycles >= 5 })
	fg.mu.Lock()
	test.That(t, fg.sets, test.ShouldResemble, []float64{680}) // 80 % of 850, once
	fg.mu.Unlock()

	lg.mu.Lock()
	lg.pct = 10
	lg.mu.Unlock()
	waitFor(t, func() bool {
		fg.mu.Lock()
		defer fg.mu.Unlock()
		return len(fg.sets) == 2 && fg.sets[1] == 85
	})
}

func TestABlockingFollowerGripperDoesNotStallTheMirrorLoop(t *testing.T) {
	m := Mapping{}
	leader := newFakeLeader(t, []referenceframe.Input{0, 0, 0, 0, 0})
	motion := &fakeMotion{pose: followerPoseReachableBy(t, leader, m, []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0})}
	lg := &fakeGripper{pct: 80}
	fg := &fakeGripper{block: make(chan struct{})}
	e := New(Deps{
		Leader: leader, LeaderGripper: lg, FollowerGripper: fg,
		Pipeline: NewPipeline(motion, "builtin", "xarm-gripper"),
		Logger:   logging.NewTestLogger(t),
	}, Config{Mapping: m, LeaderHold: HoldStiff, RateHz: 200, FollowerGripperMode: GripperModeXArm, GripperRange: 850, RotationEnabled: true})

	ctx := context.Background()
	test.That(t, e.Start(ctx), test.ShouldBeNil)
	// The first gripper write is now blocked inside the fake. Pose mirroring must carry on.
	waitFor(t, func() bool { fg.mu.Lock(); defer fg.mu.Unlock(); return len(fg.sets) == 1 })
	leader.setJoints([]referenceframe.Input{0.4, -0.3, 0.6, 0.2, 0})
	waitFor(t, func() bool { return motion.moveCount() >= 3 && e.Status().Cycles >= 20 })

	// Changes that arrive while blocked collapse to the latest one.
	for _, p := range []float64{70, 60, 20} {
		lg.mu.Lock()
		lg.pct = p
		lg.mu.Unlock()
		waitFor(t, func() bool { return e.Status().GripperPct == p })
	}
	close(fg.block)
	waitFor(t, func() bool { fg.mu.Lock(); defer fg.mu.Unlock(); return len(fg.sets) == 2 })
	fg.mu.Lock()
	test.That(t, fg.sets[1], test.ShouldEqual, 170) // 20 % of 850: the 70 and 60 were skipped
	fg.mu.Unlock()
	test.That(t, e.Stop(ctx), test.ShouldBeNil)
}

func TestGripperOpenCloseModeHasHysteresis(t *testing.T) {
	m := Mapping{}
	leader := newFakeLeader(t, []referenceframe.Input{0, 0, 0, 0, 0})
	motion := &fakeMotion{pose: followerPoseReachableBy(t, leader, m, []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0})}
	lg := &fakeGripper{pct: 50}
	fg := &fakeGripper{}
	e := New(Deps{
		Leader: leader, LeaderGripper: lg, FollowerGripper: fg,
		Pipeline: NewPipeline(motion, "builtin", "xarm-gripper"),
		Logger:   logging.NewTestLogger(t),
	}, Config{Mapping: m, LeaderHold: HoldStiff, RateHz: 200, FollowerGripperMode: GripperModeOpenClose})

	ctx := context.Background()
	test.That(t, e.Start(ctx), test.ShouldBeNil)
	defer func() { test.That(t, e.Stop(ctx), test.ShouldBeNil) }()
	waitFor(t, func() bool { return e.Status().Cycles >= 5 })
	fg.mu.Lock()
	test.That(t, fg.open, test.ShouldBeEmpty) // 50 % is inside the band: nothing sent
	fg.mu.Unlock()

	lg.mu.Lock()
	lg.pct = 90
	lg.mu.Unlock()
	waitFor(t, func() bool { fg.mu.Lock(); defer fg.mu.Unlock(); return len(fg.open) == 1 && fg.open[0] })
	lg.mu.Lock()
	lg.pct = 20
	lg.mu.Unlock()
	waitFor(t, func() bool { fg.mu.Lock(); defer fg.mu.Unlock(); return len(fg.open) == 2 && !fg.open[1] })
}

func TestSettingsResolveDefaultsAndParsesOrientations(t *testing.T) {
	var s Settings
	err := json.Unmarshal([]byte(`{
		"leader_arm": "leader-arm", "follower_component": "xarm-gripper",
		"scale": 2.5,
		"origin_translation_mm": {"x": 100, "y": 0, "z": 50},
		"origin_orientation": {"type": "ov_degrees", "value": {"x": 0, "y": 0, "z": 1, "th": -90}},
		"tool_orientation": {"type": "ov_degrees", "value": {"x": 0, "y": 1, "z": 0, "th": 0}},
		"smooth_tau_ms": 0,
		"leader_hold": "stiff"
	}`), &s)
	test.That(t, err, test.ShouldBeNil)
	cfg, err := s.Resolve()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cfg.RateHz, test.ShouldEqual, 60)
	test.That(t, cfg.Mapping.Scale, test.ShouldEqual, 2.5)
	test.That(t, cfg.Mapping.Origin.Point().X, test.ShouldEqual, 100)
	test.That(t, cfg.Mapping.ToolRotation, test.ShouldNotBeNil)
	test.That(t, cfg.Filter.SmoothTau, test.ShouldEqual, 0) // explicit 0 disables
	test.That(t, cfg.Filter.DeadzonePosMM, test.ShouldEqual, 1)
	test.That(t, cfg.LeaderHold, test.ShouldEqual, HoldStiff)
	test.That(t, cfg.RotationEnabled, test.ShouldBeTrue)
	test.That(t, cfg.CompensateResidual, test.ShouldBeTrue)
	test.That(t, cfg.FollowerGripperMode, test.ShouldEqual, GripperModeXArm)
	test.That(t, cfg.MaxConsecutiveErrors, test.ShouldEqual, 10)
	test.That(t, s.MotionName(), test.ShouldEqual, "builtin")

	// Defaults when nothing is set.
	cfg, err = Settings{LeaderArm: "a", FollowerComponent: "g"}.Resolve()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cfg.Filter.SmoothTau, test.ShouldEqual, 100*time.Millisecond)
	test.That(t, cfg.LeaderHold, test.ShouldEqual, HoldManual)
}

func TestSettingsValidate(t *testing.T) {
	test.That(t, Settings{}.Validate(), test.ShouldNotBeNil)
	test.That(t, Settings{LeaderArm: "a"}.Validate(), test.ShouldNotBeNil)
	test.That(t, Settings{LeaderArm: "a", FollowerComponent: "g", LeaderHold: "limp"}.Validate(), test.ShouldNotBeNil)
	test.That(t, Settings{LeaderArm: "a", FollowerComponent: "g", LeaderGripper: "lg"}.Validate(), test.ShouldNotBeNil)
	test.That(t, Settings{LeaderArm: "a", FollowerComponent: "g", RateHz: 5000}.Validate(), test.ShouldNotBeNil)
	test.That(t, Settings{LeaderArm: "a", FollowerComponent: "g", FollowerGripperMode: "maybe"}.Validate(), test.ShouldNotBeNil)
	test.That(t, Settings{LeaderArm: "a", FollowerComponent: "g"}.Validate(), test.ShouldBeNil)
}

func TestStartRefusesWhenTheLeaderAcceptsTheMoveButDoesNotMove(t *testing.T) {
	m := Mapping{}
	leader := newFakeLeader(t, []referenceframe.Input{0, 0, 0, 0, 0})
	leader.limp = true
	motion := &fakeMotion{pose: followerPoseReachableBy(t, leader, m, []referenceframe.Input{0.3, -0.5, 0.8, 0.2, 0})}
	e := newTestEngine(t, leader, motion, Config{Mapping: m, LeaderHold: HoldManual, AlignMaxPositionErrorMM: 100})

	err := e.Start(context.Background())
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "no joint moved")
	test.That(t, err.Error(), test.ShouldContainSubstring, "torque")
	test.That(t, len(leader.moves), test.ShouldEqual, 1)                                   // the move was attempted
	test.That(t, leader.commandNames(), test.ShouldResemble, []string{"exit_manual_mode"}) // but no hold applied
	test.That(t, len(motion.starts), test.ShouldEqual, 0)                                  // and the pipeline untouched
	test.That(t, e.Status().Running, test.ShouldBeFalse)
}

func TestAlignAcceptsAJointStoppingShortAndCompensates(t *testing.T) {
	m := Mapping{}
	leader := newFakeLeader(t, []referenceframe.Input{0, 0, 0, 0, 0})
	// The elbow is clamped well below what the follower pose needs, like a calibrated
	// range narrower than the model's.
	leader.clampMax = []referenceframe.Input{10, 10, 0.3, 10, 10}
	motion := &fakeMotion{pose: followerPoseReachableBy(t, leader, m, []referenceframe.Input{0.3, -0.5, 0.8, 0.2, 0})}
	e := newTestEngine(t, leader, motion, Config{Mapping: m, LeaderHold: HoldStiff, CompensateResidual: true, RotationEnabled: true, AlignMaxPositionErrorMM: 200})

	ctx := context.Background()
	test.That(t, e.Start(ctx), test.ShouldBeNil)
	defer func() { test.That(t, e.Stop(ctx), test.ShouldBeNil) }()
	al := e.Align()
	test.That(t, al.PositionErrorMM, test.ShouldBeGreaterThan, 5) // honest: measured after the move
	test.That(t, al.Residual.IsZero(), test.ShouldBeFalse)
	// And the first target is still exactly the follower's pose.
	waitFor(t, func() bool { return motion.moveCount() >= 1 })
	test.That(t, spatialmath.PoseAlmostCoincidentEps(motion.lastMove(), motion.pose, 1e-3), test.ShouldBeTrue)
}

func TestAlignRefusesWhenTheShortStopLeavesTooLargeAGap(t *testing.T) {
	m := Mapping{}
	leader := newFakeLeader(t, []referenceframe.Input{0, 0, 0, 0, 0})
	leader.clampMax = []referenceframe.Input{10, 10, 0.3, 10, 10}
	motion := &fakeMotion{pose: followerPoseReachableBy(t, leader, m, []referenceframe.Input{0.3, -0.5, 0.8, 0.2, 0})}
	e := newTestEngine(t, leader, motion, Config{Mapping: m, LeaderHold: HoldManual, CompensateResidual: true, RotationEnabled: true, AlignMaxPositionErrorMM: 5})

	err := e.Start(context.Background())
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "stopped short")
	// The hold was applied and then released again, so the leader is not left in manual mode.
	names := leader.commandNames()
	test.That(t, names[len(names)-1], test.ShouldEqual, "exit_manual_mode")
	test.That(t, len(motion.starts), test.ShouldEqual, 0)
}

func TestCaptureStartMovesNothingAndMirrorsDeltas(t *testing.T) {
	captureModeTests = true
	defer func() { captureModeTests = false }()
	m := Mapping{Scale: 2}
	leaderHome := []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0}
	leader := newFakeLeader(t, leaderHome)
	// The follower is somewhere unrelated to where the leader maps.
	followerPose := spatialmath.NewPose(r3.Vector{X: 400, Y: 100, Z: 300}, ov(0, 0, -1, 0))
	motion := &fakeMotion{pose: followerPose}
	e := newTestEngine(t, leader, motion, Config{Mapping: m, LeaderHold: HoldManual, CompensateResidual: true, RotationEnabled: true})

	ctx := context.Background()
	test.That(t, e.Start(ctx), test.ShouldBeNil)
	defer func() { test.That(t, e.Stop(ctx), test.ShouldBeNil) }()

	al := e.Align()
	test.That(t, al.Aligned, test.ShouldBeFalse)
	test.That(t, len(leader.moves), test.ShouldEqual, 0) // the leader was NOT moved
	test.That(t, leader.commandNames(), test.ShouldResemble, []string{"exit_manual_mode", "enter_manual_mode"})
	test.That(t, al.PositionErrorMM, test.ShouldBeGreaterThan, 50) // honest about the gap it absorbed

	// First target is the follower's own pose.
	waitFor(t, func() bool { return motion.moveCount() >= 1 })
	test.That(t, spatialmath.PoseAlmostCoincidentEps(motion.lastMove(), followerPose, 1e-3), test.ShouldBeTrue)

	// A leader delta becomes the same scaled delta on the follower, from the follower's pose.
	before, err := leader.model.Transform(leaderHome)
	test.That(t, err, test.ShouldBeNil)
	moved := []referenceframe.Input{0.1, -0.5, 0.6, 0.2, 0}
	after, err := leader.model.Transform(moved)
	test.That(t, err, test.ShouldBeNil)
	delta := after.Point().Sub(before.Point()).Mul(2)
	leader.setJoints(moved)
	waitFor(t, func() bool {
		last := motion.lastMove()
		return last != nil && last.Point().Sub(followerPose.Point().Add(delta)).Norm() < 0.5
	})
}

func TestLimpLeaderIsAWarningInBestEffortMode(t *testing.T) {
	m := Mapping{}
	leader := newFakeLeader(t, []referenceframe.Input{0, 0, 0, 0, 0})
	leader.limp = true
	motion := &fakeMotion{pose: followerPoseReachableBy(t, leader, m, []referenceframe.Input{0.3, -0.5, 0.8, 0.2, 0})}
	e := newTestEngine(t, leader, motion, Config{Mapping: m, LeaderHold: HoldManual, CompensateResidual: true, RotationEnabled: true})

	ctx := context.Background()
	test.That(t, e.Start(ctx), test.ShouldBeNil) // best effort: warned, not refused
	defer func() { test.That(t, e.Stop(ctx), test.ShouldBeNil) }()
	waitFor(t, func() bool { return motion.moveCount() >= 1 })
	test.That(t, spatialmath.PoseAlmostCoincidentEps(motion.lastMove(), motion.pose, 1e-3), test.ShouldBeTrue)
}

func TestStartFreesTheLeaderGripperTrigger(t *testing.T) {
	captureModeTests = true
	defer func() { captureModeTests = false }()
	m := Mapping{}
	leader := newFakeLeader(t, []referenceframe.Input{0, 0, 0, 0, 0})
	motion := &fakeMotion{pose: followerPoseReachableBy(t, leader, m, []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0})}
	lg := &fakeGripper{pct: 50}
	e := New(Deps{
		Leader: leader, LeaderGripper: lg, FollowerGripper: &fakeGripper{},
		Pipeline: NewPipeline(motion, "builtin", "xarm-gripper"),
		Logger:   logging.NewTestLogger(t),
	}, Config{Mapping: m, LeaderHold: HoldManual, RateHz: 200, FollowerGripperMode: GripperModeXArm, RotationEnabled: true})
	ctx := context.Background()
	test.That(t, e.Start(ctx), test.ShouldBeNil)
	defer func() { test.That(t, e.Stop(ctx), test.ShouldBeNil) }()
	lg.mu.Lock()
	defer lg.mu.Unlock()
	test.That(t, lg.torque, test.ShouldNotBeNil)
	test.That(t, *lg.torque, test.ShouldBeFalse)
}
