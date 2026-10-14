package cartesianteleop

import (
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func at(x float64) spatialmath.Pose {
	return spatialmath.NewPose(r3.Vector{X: x}, spatialmath.NewZeroOrientation())
}

func TestFilterFirstSampleSeedsAndPasses(t *testing.T) {
	f := NewFilter(FilterConfig{SmoothTau: time.Second, DeadzonePosMM: 5})
	p, ok := f.Step(time.Unix(0, 0), at(100))
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, p.Point().X, test.ShouldAlmostEqual, 100)
}

func TestFilterSmoothingIsTimeNormalised(t *testing.T) {
	// The same wall-clock span reaches the same fraction of the step whether sampled at
	// 30 Hz or 120 Hz.
	run := func(hz float64) float64 {
		f := NewFilter(FilterConfig{SmoothTau: 200 * time.Millisecond})
		t0 := time.Unix(0, 0)
		f.Step(t0, at(0))
		dt := time.Duration(float64(time.Second) / hz)
		var last spatialmath.Pose
		for now := t0.Add(dt); !now.After(t0.Add(400 * time.Millisecond)); now = now.Add(dt) {
			last, _ = f.Step(now, at(100))
		}
		return last.Point().X
	}
	slow, fast := run(30), run(120)
	// 400 ms at tau 200 ms is ~86 % of the way there in the continuous limit.
	test.That(t, slow, test.ShouldBeBetween, 75, 95)
	test.That(t, fast, test.ShouldBeBetween, 75, 95)
	test.That(t, slow-fast, test.ShouldBeBetween, -6, 6)
}

func TestFilterDeadzoneSkipsSmallMoves(t *testing.T) {
	f := NewFilter(FilterConfig{DeadzonePosMM: 2})
	f.Step(time.Unix(0, 0), at(0))
	_, ok := f.Step(time.Unix(1, 0), at(1))
	test.That(t, ok, test.ShouldBeFalse)
	test.That(t, f.Deadzoned, test.ShouldEqual, 1)
	_, ok = f.Step(time.Unix(2, 0), at(3))
	test.That(t, ok, test.ShouldBeTrue)
}

func TestFilterRejectsOutliersThenAcceptsAPersistentJump(t *testing.T) {
	f := NewFilter(FilterConfig{OutlierPosMMPerSec: 1000, OutlierAcceptAfter: 3})
	t0 := time.Unix(0, 0)
	f.Step(t0, at(0))
	// 500 mm in 10 ms is 50 m/s: an outlier.
	for i := 1; i <= 2; i++ {
		_, ok := f.Step(t0.Add(time.Duration(i)*10*time.Millisecond), at(500))
		test.That(t, ok, test.ShouldBeFalse)
	}
	test.That(t, f.Rejected, test.ShouldEqual, 2)
	p, ok := f.Step(t0.Add(30*time.Millisecond), at(500))
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, p.Point().X, test.ShouldAlmostEqual, 500)
}

func TestFilterClampsDtSoAStallIsNotInfiniteSpeed(t *testing.T) {
	f := NewFilter(FilterConfig{OutlierPosMMPerSec: 1000})
	t0 := time.Unix(0, 0)
	f.Step(t0, at(0))
	// Same timestamp twice: dt clamps to 1/120 s, so 5 mm is 600 mm/s, under the limit.
	_, ok := f.Step(t0, at(5))
	test.That(t, ok, test.ShouldBeTrue)
}
