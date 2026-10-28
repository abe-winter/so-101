package cartesianteleop

import (
	"context"
	"strings"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestErrorNamesSideMatchesWholeComponentNames(t *testing.T) {
	e := "all IK solutions failed constraints. Failures: { robot constraint: violation between left-arm:upper_forearm and obstacle-right-wall:right-wall geometries: 100.00% },"
	test.That(t, errorNamesSide(e, "left-gripper", "left-arm"), test.ShouldBeTrue)
	test.That(t, errorNamesSide(e, "right-gripper", "right-arm"), test.ShouldBeFalse)
	test.That(t, errorNamesSide(e, "left-gripper", ""), test.ShouldBeFalse) // arm not configured
	test.That(t, errorNamesSide("violation between left-arm-2:wrist and x:y", "left-arm"), test.ShouldBeFalse)
	// Unattributable errors name nothing; the stall check decides those.
	test.That(t, errorNamesSide("zero IK solutions produced, goal positions appears to be physically unreachable", "left-gripper", "left-arm"), test.ShouldBeFalse)
	// A collision between the two followers blocks both sides.
	both := "self-collision constraint: violation between left-gripper:claws and right-gripper:case-gripper"
	test.That(t, errorNamesSide(both, "left-gripper"), test.ShouldBeTrue)
	test.That(t, errorNamesSide(both, "right-gripper"), test.ShouldBeTrue)
}

func pt(x float64) spatialmath.Pose { return spatialmath.NewPoseFromPoint(r3.Vector{X: x}) }

func TestStallNeedsBothAGapAndNoMotion(t *testing.T) {
	cfg := FeedbackConfig{}.withDefaults() // lag 40 mm, stall 3 mm, for 300 ms
	period := 100 * time.Millisecond

	// Trailing a fast hand: far from the target but still moving -> never stalled.
	var s stallTracker
	for i := 0; i < 10; i++ {
		stalled, _ := s.step(pt(float64(i*20)), pt(500), period, cfg)
		test.That(t, stalled, test.ShouldBeFalse)
	}

	// Stopped, but close to its target -> following, not stalled.
	s = stallTracker{}
	for i := 0; i < 10; i++ {
		stalled, _ := s.step(pt(0), pt(10), period, cfg)
		test.That(t, stalled, test.ShouldBeFalse)
	}

	// Stopped far from its target: stalled only once that has lasted StallFor.
	s = stallTracker{}
	var verdicts []bool
	for i := 0; i < 5; i++ {
		stalled, gap := s.step(pt(0), pt(100), period, cfg)
		test.That(t, gap, test.ShouldEqual, 100)
		verdicts = append(verdicts, stalled)
	}
	// The first poll has no previous pose, so it cannot count as "not moving".
	test.That(t, verdicts, test.ShouldResemble, []bool{false, false, false, true, true})
}

func TestAnchorIsTheSampleWhoseTargetIsNearestTheFollower(t *testing.T) {
	var f feedbackState
	now := time.Now()
	for i, x := range []float64{0, 50, 100, 400, 800} {
		f.record(leaderSample{at: now.Add(time.Duration(i) * 10 * time.Millisecond),
			joints: []referenceframe.Input{x / 1000}, target: pt(x)}, time.Second)
	}
	// The follower got stuck at ~110 mm while the leader ran on to 800: pull back to 100,
	// not to the most recent sample.
	test.That(t, f.anchorFor(pt(110)), test.ShouldResemble, []referenceframe.Input{0.1})

	// Old samples age out.
	f.record(leaderSample{at: now.Add(5 * time.Second), joints: []referenceframe.Input{9}, target: pt(900)}, time.Second)
	test.That(t, f.anchorFor(pt(0)), test.ShouldResemble, []referenceframe.Input{9})
}

func feedbackTestConfig() Config {
	return Config{
		RateHz:     200,
		LeaderHold: HoldManual,
		Feedback: FeedbackConfig{
			Enabled: true, PollHz: 100, StallFor: 30 * time.Millisecond, TorqueLimit: 300,
		},
	}
}

func (f *fakeLeader) holdCommands() []map[string]interface{} {
	f.mu.Lock()
	defer f.mu.Unlock()
	var out []map[string]interface{}
	for _, c := range f.commands {
		if c["command"] == "manual_hold" {
			out = append(out, c)
		}
	}
	return out
}

func (f *fakeLeader) countCommand(name string) int {
	n := 0
	for _, c := range f.commandNames() {
		if c == name {
			n++
		}
	}
	return n
}

// A planner error naming this side's follower pulls the leader back to the pose that mapped
// onto where the follower actually is, and lets go once the error clears.
func TestPlannerErrorOnThisSidePullsTheLeaderBackAndReleases(t *testing.T) {
	m := Mapping{Scale: 1}
	start := []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0}
	leader := newFakeLeader(t, start)
	motion := &fakeMotion{pose: followerPoseReachableBy(t, leader, m, start)}
	cfg := feedbackTestConfig()
	cfg.Mapping = m
	// The fake follower never moves, so the stall check would also fire once the leader runs
	// ahead; push it out of reach so this test exercises error attribution alone.
	cfg.Feedback.StallFor = time.Hour
	e := newTestEngine(t, leader, motion, cfg)
	test.That(t, e.Start(context.Background()), test.ShouldBeNil)
	defer e.Stop(context.Background())

	// The operator pushes on; the follower (fixed in the fake) stays where it was.
	waitFor(t, func() bool { return motion.moveCount() > 3 })
	leader.setJoints([]referenceframe.Input{0.6, -0.1, 0.3, 0.5, 0})
	waitFor(t, func() bool { return motion.moveCount() > 10 })
	motion.setStatusError("violation between xarm-gripper:claws and obstacle-table:table")

	waitFor(t, func() bool { return len(leader.holdCommands()) > 0 })
	hold := leader.holdCommands()[0]
	test.That(t, hold["torque_limit"], test.ShouldEqual, 300.0)
	joints := hold["joints"].([]interface{})
	test.That(t, len(joints), test.ShouldEqual, len(start))
	for i := range start {
		test.That(t, joints[i].(float64), test.ShouldAlmostEqual, float64(start[i]), 1e-9)
	}
	st := e.Status()
	test.That(t, st.Blocked, test.ShouldBeTrue)
	test.That(t, st.BlockedReason, test.ShouldContainSubstring, "obstacle-table")

	// The hold keeps being refreshed (its TTL is short) while the block lasts.
	n := len(leader.holdCommands())
	waitFor(t, func() bool { return len(leader.holdCommands()) > n+2 })

	// The leader comes back and the planner recovers: release.
	leader.setJoints(start)
	motion.setStatusError("")
	waitFor(t, func() bool { return leader.countCommand("manual_release") > 0 })
	test.That(t, e.Status().Blocked, test.ShouldBeFalse)
}

// An error naming only the OTHER side's follower must not touch this leader while this
// follower is keeping up.
func TestPlannerErrorOnTheOtherSideIsIgnored(t *testing.T) {
	m := Mapping{Scale: 1}
	start := []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0}
	leader := newFakeLeader(t, start)
	motion := &fakeMotion{pose: followerPoseReachableBy(t, leader, m, start)}
	cfg := feedbackTestConfig()
	cfg.Mapping = m
	e := newTestEngine(t, leader, motion, cfg)
	test.That(t, e.Start(context.Background()), test.ShouldBeNil)
	defer e.Stop(context.Background())

	motion.setStatusError("violation between right-gripper:claws and obstacle-table:table")
	time.Sleep(150 * time.Millisecond)
	test.That(t, leader.holdCommands(), test.ShouldBeEmpty)
	test.That(t, e.Status().Blocked, test.ShouldBeFalse)
}

// An unreachable goal names no component; the side whose follower is stuck short of its
// target is the one that gets pulled back.
func TestAStalledFollowerPullsTheLeaderBackWithoutANamedError(t *testing.T) {
	m := Mapping{Scale: 1}
	start := []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0}
	leader := newFakeLeader(t, start)
	motion := &fakeMotion{pose: followerPoseReachableBy(t, leader, m, start)}
	cfg := feedbackTestConfig()
	cfg.Mapping = m
	e := newTestEngine(t, leader, motion, cfg)
	test.That(t, e.Start(context.Background()), test.ShouldBeNil)
	defer e.Stop(context.Background())

	motion.setStatusError("zero IK solutions produced, goal positions appears to be physically unreachable")
	time.Sleep(60 * time.Millisecond)
	test.That(t, leader.holdCommands(), test.ShouldBeEmpty) // following: target == follower

	leader.setJoints([]referenceframe.Input{0.6, -0.1, 0.3, 0.5, 0}) // far away; follower stays
	waitFor(t, func() bool { return len(leader.holdCommands()) > 0 })
	reason := e.Status().BlockedReason
	test.That(t, strings.Contains(reason, "stalled"), test.ShouldBeTrue)
	test.That(t, reason, test.ShouldContainSubstring, "unreachable")
}

// Stop must not leave a hold behind.
func TestStopReleasesAnActiveHold(t *testing.T) {
	m := Mapping{Scale: 1}
	start := []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0}
	leader := newFakeLeader(t, start)
	motion := &fakeMotion{pose: followerPoseReachableBy(t, leader, m, start)}
	cfg := feedbackTestConfig()
	cfg.Mapping = m
	e := newTestEngine(t, leader, motion, cfg)
	test.That(t, e.Start(context.Background()), test.ShouldBeNil)

	waitFor(t, func() bool { return motion.moveCount() > 3 })
	motion.setStatusError("violation between xarm-gripper:claws and obstacle-table:table")
	waitFor(t, func() bool { return len(leader.holdCommands()) > 0 })
	test.That(t, e.Stop(context.Background()), test.ShouldBeNil)
	test.That(t, leader.countCommand("manual_release"), test.ShouldBeGreaterThan, 0)
}

func TestFeedbackIsOffUnlessAsked(t *testing.T) {
	m := Mapping{Scale: 1}
	start := []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0}
	leader := newFakeLeader(t, start)
	motion := &fakeMotion{pose: followerPoseReachableBy(t, leader, m, start)}
	e := newTestEngine(t, leader, motion, Config{Mapping: m, LeaderHold: HoldManual})
	test.That(t, e.Start(context.Background()), test.ShouldBeNil)
	defer e.Stop(context.Background())
	motion.setStatusError("violation between xarm-gripper:claws and obstacle-table:table")
	time.Sleep(100 * time.Millisecond)
	test.That(t, leader.holdCommands(), test.ShouldBeEmpty)
}

func TestSettingsValidateBlockedFeedback(t *testing.T) {
	base := Settings{LeaderArm: "l", FollowerComponent: "f"}
	s := base
	s.BlockedFeedback = "spring"
	test.That(t, s.Validate(), test.ShouldBeNil)
	cfg, err := s.Resolve()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cfg.Feedback.Enabled, test.ShouldBeTrue)

	s.LeaderHold = HoldStiff
	test.That(t, s.Validate(), test.ShouldNotBeNil)

	s = base
	s.BlockedFeedback = "buzz"
	test.That(t, s.Validate(), test.ShouldNotBeNil)

	s = base
	s.BlockedFeedback = "spring"
	s.FeedbackTorqueLimit = 2000
	test.That(t, s.Validate(), test.ShouldNotBeNil)
}

// gripper_lock: a follower-gripper command freezes the pair -- no pose sends, the leader held
// where it was -- until the lock lapses, then the leader is released and mirroring resumes.
func TestGripperLockFreezesThePairWhileTheGripperMoves(t *testing.T) {
	m := Mapping{}
	start := []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0}
	leader := newFakeLeader(t, []referenceframe.Input{0, 0, 0, 0, 0})
	motion := &fakeMotion{pose: followerPoseReachableBy(t, leader, m, start)}
	lg := &fakeGripper{pct: 80}
	fg := &fakeGripper{}
	e := New(Deps{
		Leader: leader, LeaderGripper: lg, FollowerGripper: fg,
		Pipeline: NewPipeline(motion, "builtin", "xarm-gripper"),
		Logger:   logging.NewTestLogger(t),
	}, Config{
		Mapping: m, LeaderHold: HoldManual, RateHz: 200, AlignOnStart: true, RotationEnabled: true,
		FollowerGripperMode: GripperModeXArm, GripperRange: 850,
		GripperLock: 200 * time.Millisecond, GripperLockTorqueLimit: 250,
	})
	ctx := context.Background()
	test.That(t, e.Start(ctx), test.ShouldBeNil)
	defer func() { test.That(t, e.Stop(ctx), test.ShouldBeNil) }()

	// The initial gripper send locks the pair and holds the leader where it is.
	waitFor(t, func() bool { return e.Status().GripperLocked && len(leader.holdCommands()) == 1 })
	// A pose send that started before the gripper command may still be completing. Let that
	// existing command settle before asserting that the active lock suppresses new sends.
	waitFor(t, func() bool { return !e.inFlight.Load() })
	hold := leader.holdCommands()[0]
	test.That(t, hold["torque_limit"], test.ShouldEqual, 250.0)
	test.That(t, len(hold["joints"].([]interface{})), test.ShouldEqual, 5)

	// The hand moves the leader anyway: nothing reaches the follower while locked.
	sent := motion.moveCount()
	leader.setJoints([]referenceframe.Input{0.4, -0.3, 0.6, 0.2, 0})
	time.Sleep(60 * time.Millisecond)
	test.That(t, motion.moveCount(), test.ShouldEqual, sent)
	test.That(t, leader.countCommand("manual_release"), test.ShouldEqual, 0)

	// A further gripper change extends the lock and refreshes the hold.
	lg.mu.Lock()
	lg.pct = 20
	lg.mu.Unlock()
	waitFor(t, func() bool { return len(leader.holdCommands()) == 2 })

	// Once it lapses the leader is released and mirroring resumes.
	waitFor(t, func() bool { return !e.Status().GripperLocked && leader.countCommand("manual_release") == 1 })
	waitFor(t, func() bool { return motion.moveCount() > sent })
}

func TestGripperLockSettings(t *testing.T) {
	base := Settings{LeaderArm: "l", FollowerComponent: "f"}
	cfg, err := base.Resolve()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cfg.GripperLock, test.ShouldEqual, time.Duration(0))

	on := base
	on.GripperLock = true
	cfg, err = on.Resolve()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cfg.GripperLock, test.ShouldEqual, 1500*time.Millisecond)
	test.That(t, cfg.GripperLockTorqueLimit, test.ShouldEqual, 250)

	on.GripperLockMs = 5000
	test.That(t, on.Validate(), test.ShouldNotBeNil)
}
