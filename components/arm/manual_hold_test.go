package arm

import (
	"context"
	"math"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
)

func TestHoldStepIsRateLimitedAndClamped(t *testing.T) {
	lim := [2]float64{-1, 1}
	ng, moved := holdStep(0, 0.5, 0.1, lim)
	if !moved || !approx(ng, 0.1) {
		t.Fatalf("a far target must advance by maxStep only: ng=%v", ng)
	}
	ng, _ = holdStep(0, -0.05, 0.1, lim)
	if !approx(ng, -0.05) {
		t.Fatalf("a near target is reached exactly: ng=%v", ng)
	}
	ng, moved = holdStep(0.3, 0.3, 0.1, lim)
	if moved || !approx(ng, 0.3) {
		t.Fatalf("at target must not move: ng=%v moved=%v", ng, moved)
	}
	ng, _ = holdStep(0.95, 5, 0.1, lim)
	if !approx(ng, 1) {
		t.Fatalf("must clamp to the limit: ng=%v", ng)
	}
}

func waitFor(t *testing.T, what string, cond func() bool) {
	t.Helper()
	deadline := time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) {
		if cond() {
			return
		}
		time.Sleep(5 * time.Millisecond)
	}
	t.Fatalf("timed out waiting for %s", what)
}

// A hold pulls the goals toward its target at a bounded rate (not in one write, which at
// Speed 0 would snap the arm), raises the torque cap, and on release restores the cap and
// re-anchors the goals where the arm actually is.
func TestManualHoldPullsBackAndReleases(t *testing.T) {
	io := newFakeIO()
	p := manualParams{posDeadband: 0.05, dt: 0.01}
	sess := newManualSession(context.Background(), io, p, 8, 50, logging.NewTestLogger(t))
	sess.start()
	defer sess.stop()

	// 90 deg/s at a 10 ms tick is at most ~0.0157 rad per write.
	if err := sess.setHold(context.Background(), map[int]float64{1: 0.4, 2: 0}, 250, time.Second, 90); err != nil {
		t.Fatal(err)
	}
	time.Sleep(25 * time.Millisecond)
	io.mu.Lock()
	early := io.pos[1]
	torque := append([]int(nil), io.torqueWrites...)
	io.mu.Unlock()
	if early <= 0 || early >= 0.2 {
		t.Fatalf("after ~2 ticks the hold must be part-way, not snapped: pos=%v", early)
	}
	if len(torque) != 1 || torque[0] != 250 {
		t.Fatalf("the hold must raise torque_limit once to 250, got %v", torque)
	}
	waitFor(t, "hold to reach its target", func() bool {
		io.mu.Lock()
		defer io.mu.Unlock()
		return approx(io.pos[1], 0.4)
	})

	// Refreshing with the same torque must not rewrite the register.
	if err := sess.setHold(context.Background(), map[int]float64{1: 0.4, 2: 0}, 250, time.Second, 90); err != nil {
		t.Fatal(err)
	}

	// The hand now holds the joint away from the target; releasing must anchor there.
	io.mu.Lock()
	io.held[1] = 0.1
	io.mu.Unlock()
	if err := sess.releaseHold(context.Background()); err != nil {
		t.Fatal(err)
	}
	if sess.holding() {
		t.Fatal("release must clear the hold")
	}
	io.mu.Lock()
	torque = append([]int(nil), io.torqueWrites...)
	io.mu.Unlock()
	if len(torque) != 2 || torque[1] != 50 {
		t.Fatalf("release must restore the session torque 50 exactly once more, got %v", torque)
	}
	waitFor(t, "goal re-anchored at the hand", func() bool {
		sess.mu.Lock()
		defer sess.mu.Unlock()
		return math.Abs(sess.state[1].goal-0.1) < 1e-9
	})
}

// A hold nobody refreshes expires on its own: if the teleop engine dies mid-hold, the leader
// goes back to following the hand rather than staying stiff.
func TestManualHoldExpires(t *testing.T) {
	io := newFakeIO()
	p := manualParams{posDeadband: 0.05, dt: 0.01}
	sess := newManualSession(context.Background(), io, p, 8, 50, logging.NewTestLogger(t))
	sess.start()
	defer sess.stop()

	if err := sess.setHold(context.Background(), map[int]float64{1: 0.2}, 200, 40*time.Millisecond, 0); err != nil {
		t.Fatal(err)
	}
	waitFor(t, "hold to expire", func() bool { return !sess.holding() })
	io.mu.Lock()
	defer io.mu.Unlock()
	if n := len(io.torqueWrites); n != 2 || io.torqueWrites[1] != 50 {
		t.Fatalf("expiry must restore torque like a release, got %v", io.torqueWrites)
	}
}

// With the torque_limit register left alone (explicit 0 override), a hold cannot know what to
// restore, so it must not touch the register at all.
func TestManualHoldLeavesUnmanagedTorqueAlone(t *testing.T) {
	io := newFakeIO()
	sess := newManualSession(context.Background(), io, manualParams{posDeadband: 0.05, dt: 0.01}, 8, 0, logging.NewTestLogger(t))
	sess.start()
	defer sess.stop()
	if err := sess.setHold(context.Background(), map[int]float64{1: 0.1}, 300, time.Second, 0); err != nil {
		t.Fatal(err)
	}
	if err := sess.releaseHold(context.Background()); err != nil {
		t.Fatal(err)
	}
	io.mu.Lock()
	defer io.mu.Unlock()
	if len(io.torqueWrites) != 0 {
		t.Fatalf("unmanaged torque_limit must not be written, got %v", io.torqueWrites)
	}
}

func TestManualHoldRejectsUnknownServoAndStoppedSession(t *testing.T) {
	io := newFakeIO()
	sess := newManualSession(context.Background(), io, manualParams{posDeadband: 0.05, dt: 0.01}, 8, 50, logging.NewTestLogger(t))
	if err := sess.setHold(context.Background(), map[int]float64{1: 0}, 0, 0, 0); err == nil {
		t.Fatal("a hold on a session that is not running must fail")
	}
	sess.start()
	defer sess.stop()
	if err := sess.setHold(context.Background(), map[int]float64{9: 0}, 0, 0, 0); err == nil {
		t.Fatal("a hold on a servo outside the session must fail")
	}
}
