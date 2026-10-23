package cartesianteleop

import (
	"context"
	"fmt"
	"math"
	"strings"
	"sync"
	"time"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// FeedbackConfig tunes the blocked-follower feedback: when this pair's follower cannot follow
// (the planner rejects its target, or it stalls short of it), the leader is pulled back toward
// the last pose the follower actually reached, through the SO-101 arm's manual_hold command.
// It is the hand-held analogue of the vive controller's and the phone's haptic pulse, and it
// only exists with leader_hold "manual" -- the hold lives inside the leader's manual mode.
type FeedbackConfig struct {
	Enabled bool
	// TorqueLimit is the leader's torque cap while it pushes back (STS3215 torque_limit,
	// 0-1000). Higher pushes harder; manual mode's own cap is 50. Default 250.
	TorqueLimit int
	// MaxSpeedDegPerSec bounds how fast the leader is pulled back. Default 45.
	MaxSpeedDegPerSec float64
	// PollHz is how often the pipeline and the follower are checked. Default 10.
	PollHz float64
	// LagMM and StallMM define a stall: the follower is more than LagMM from its target and
	// moved less than StallMM since the previous check, for at least StallFor. Defaults 40,
	// 3, 300 ms. A follower merely trailing a fast hand is still moving, so it never stalls.
	LagMM    float64
	StallMM  float64
	StallFor time.Duration
	// FollowerArm, if set, lets a planner error that names the follower ARM's geometry (not
	// just its gripper) be attributed to this pair immediately, without waiting for a stall.
	FollowerArm string
	// History is how far back the leader's poses are remembered. Default 3 s.
	History time.Duration
}

func (c FeedbackConfig) withDefaults() FeedbackConfig {
	if c.TorqueLimit <= 0 {
		c.TorqueLimit = 250
	}
	if c.MaxSpeedDegPerSec <= 0 {
		c.MaxSpeedDegPerSec = 45
	}
	if c.PollHz <= 0 {
		c.PollHz = 10
	}
	if c.LagMM <= 0 {
		c.LagMM = 40
	}
	if c.StallMM <= 0 {
		c.StallMM = 3
	}
	if c.StallFor <= 0 {
		c.StallFor = 300 * time.Millisecond
	}
	if c.History <= 0 {
		c.History = 3 * time.Second
	}
	return c
}

// leaderSample is one mirror cycle: where the leader's joints were and the follower target
// they mapped to.
type leaderSample struct {
	at     time.Time
	joints []referenceframe.Input
	target spatialmath.Pose
}

// feedbackState is the engine's blocked-follower bookkeeping. Its own mutex keeps the mirror
// loop (which records samples) and the feedback loop (which reads them) off e.mu.
type feedbackState struct {
	mu         sync.Mutex
	history    []leaderSample
	lastSent   spatialmath.Pose
	blocked    bool
	reason     string
	holdJoints []referenceframe.Input
}

func (f *feedbackState) record(s leaderSample, keep time.Duration) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.history = append(f.history, s)
	cut := 0
	for cut < len(f.history) && s.at.Sub(f.history[cut].at) > keep {
		cut++
	}
	f.history = f.history[cut:]
}

func (f *feedbackState) setLastSent(p spatialmath.Pose) {
	f.mu.Lock()
	f.lastSent = p
	f.mu.Unlock()
}

func (f *feedbackState) lastSentTarget() spatialmath.Pose {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.lastSent
}

// anchorFor returns the leader joints whose mapped target was closest to where the follower
// actually is: "the last pose the follower reached", expressed as a leader pose. Choosing by
// distance rather than by time matters because a planner failure takes up to the planner's
// timeout (~1 s) to surface, by which time the most recent samples are already deep in the
// unreachable region; they are also, by construction, far from the stuck follower.
func (f *feedbackState) anchorFor(follower spatialmath.Pose) []referenceframe.Input {
	f.mu.Lock()
	defer f.mu.Unlock()
	best, bestD := -1, math.Inf(1)
	for i, s := range f.history {
		if d := s.target.Point().Sub(follower.Point()).Norm(); d < bestD {
			best, bestD = i, d
		}
	}
	if best < 0 {
		return nil
	}
	return append([]referenceframe.Input(nil), f.history[best].joints...)
}

func (f *feedbackState) setBlocked(blocked bool, reason string, hold []referenceframe.Input) {
	f.mu.Lock()
	f.blocked, f.reason, f.holdJoints = blocked, reason, hold
	f.mu.Unlock()
}

func (f *feedbackState) snapshot() (bool, string) {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.blocked, f.reason
}

// errorNamesSide reports whether a pipeline error names one of this pair's follower parts.
// Collision errors name geometries as "<component>:<geometry>", e.g. "violation between
// left-gripper:claws and obstacle-table:table"; matching on the trailing colon keeps
// "left-arm" from matching "left-arm-2". Errors that name nothing (an unreachable goal)
// return false and are left to the stall check, which can tell the two sides apart.
func errorNamesSide(errStr string, names ...string) bool {
	for _, n := range names {
		if n != "" && strings.Contains(errStr, n+":") {
			return true
		}
	}
	return false
}

// stallTracker turns successive follower poses into a "stuck short of its target" verdict.
type stallTracker struct {
	prev     spatialmath.Pose
	stuckFor time.Duration
}

func (s *stallTracker) step(follower, target spatialmath.Pose, period time.Duration, cfg FeedbackConfig) (stalled bool, gapMM float64) {
	moved := math.Inf(1)
	if s.prev != nil {
		moved = follower.Point().Sub(s.prev.Point()).Norm()
	}
	s.prev = follower
	if target == nil {
		s.stuckFor = 0
		return false, 0
	}
	gapMM = follower.Point().Sub(target.Point()).Norm()
	if gapMM > cfg.LagMM && moved < cfg.StallMM {
		s.stuckFor += period
	} else {
		s.stuckFor = 0
	}
	return s.stuckFor >= cfg.StallFor, gapMM
}

// runFeedback watches the pipeline and the follower and drives the leader's hold. It exits
// with the mirror loop, releasing any hold on the way out -- although the hold's TTL would
// release it anyway, a hand on a leader should not have to wait for that.
func (e *Engine) runFeedback(ctx context.Context) {
	defer e.wg.Done()
	cfg := e.cfg.Feedback
	period := time.Duration(float64(time.Second) / cfg.PollHz)
	ticker := time.NewTicker(period)
	defer ticker.Stop()

	var stall stallTracker
	clearPolls := 0
	defer func() {
		if blocked, _ := e.fb.snapshot(); blocked {
			e.releaseHold(context.Background())
		}
	}()
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
		}
		if !e.isRunning() {
			return
		}
		if e.gripperLocked() {
			// gripper_lock owns the leader's hold and the follower is deliberately not
			// being sent targets; neither is a block.
			stall = stallTracker{}
			continue
		}
		reason := e.blockedReason(ctx, &stall, period)
		blocked, _ := e.fb.snapshot()
		switch {
		case reason != "":
			clearPolls = 0
			e.holdBack(ctx, reason, blocked)
		case blocked:
			// Two clear checks in a row before letting go, so a pipeline error that flickers
			// while the leader is on its way back does not pump the hold on and off.
			clearPolls++
			if clearPolls >= 2 {
				clearPolls = 0
				e.releaseHold(ctx)
			}
		}
	}
}

// blockedReason is "" when this pair's follower is following, otherwise why it is not.
func (e *Engine) blockedReason(ctx context.Context, stall *stallTracker, period time.Duration) string {
	errStr := ""
	if st, err := e.deps.Pipeline.Status(ctx); err == nil {
		errStr, _ = st["error"].(string)
	}
	follower, err := e.deps.Pipeline.CurrentPose(ctx)
	if err != nil {
		return ""
	}
	e.fbFollower.Store(&follower)
	stalled, gap := stall.step(follower, e.fb.lastSentTarget(), period, e.cfg.Feedback)
	if errStr != "" && errorNamesSide(errStr, e.deps.Pipeline.Component(), e.cfg.Feedback.FollowerArm) {
		return "planner: " + errStr
	}
	if stalled {
		r := fmt.Sprintf("follower stalled %.0f mm short of its target", gap)
		if errStr != "" {
			r += " (planner: " + errStr + ")"
		}
		return r
	}
	return ""
}

// holdBack starts or refreshes the leader's pull toward the anchor. The anchor is chosen
// once, when the block starts, and kept: re-choosing every poll would chase samples recorded
// while the leader is already being pulled back.
func (e *Engine) holdBack(ctx context.Context, reason string, alreadyBlocked bool) {
	e.fb.mu.Lock()
	anchor := e.fb.holdJoints
	e.fb.mu.Unlock()
	if !alreadyBlocked || anchor == nil {
		if p := e.fbFollower.Load(); p != nil {
			anchor = e.fb.anchorFor(*p)
		}
		e.deps.Logger.Infof("follower blocked (%s); pulling the leader back", reason)
	}
	e.fb.setBlocked(true, reason, anchor)
	if anchor == nil {
		return // nothing recorded yet: report the block, but there is nowhere to pull to
	}
	joints := make([]interface{}, len(anchor))
	for i, v := range anchor {
		joints[i] = float64(v)
	}
	cfg := e.cfg.Feedback
	period := time.Duration(float64(time.Second) / cfg.PollHz)
	if _, err := e.deps.Leader.DoCommand(ctx, map[string]interface{}{
		"command":               "manual_hold",
		"joints":                joints,
		"torque_limit":          float64(cfg.TorqueLimit),
		"ttl_ms":                float64((3 * period).Milliseconds()),
		"max_speed_deg_per_sec": cfg.MaxSpeedDegPerSec,
	}); err != nil {
		e.deps.Logger.Debugf("manual_hold: %v", err)
	}
}

func (e *Engine) releaseHold(ctx context.Context) {
	e.fb.setBlocked(false, "", nil)
	if _, err := e.deps.Leader.DoCommand(ctx, map[string]interface{}{"command": "manual_release"}); err != nil {
		e.deps.Logger.Debugf("manual_release: %v", err)
	}
	e.deps.Logger.Info("follower following again; leader released")
}

func (e *Engine) isRunning() bool {
	e.mu.Lock()
	defer e.mu.Unlock()
	return e.running
}
