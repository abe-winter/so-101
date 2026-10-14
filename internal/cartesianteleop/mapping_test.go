package cartesianteleop

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func ov(x, y, z, thDeg float64) spatialmath.Orientation {
	return &spatialmath.OrientationVectorDegrees{OX: x, OY: y, OZ: z, Theta: thDeg}
}

func TestMappingRoundTrips(t *testing.T) {
	m := Mapping{
		Origin:       spatialmath.NewPose(r3.Vector{X: 500, Y: -200, Z: 30}, ov(0, 0, 1, -90)),
		Scale:        2.5,
		ToolRotation: ov(0, 1, 0, 0),
	}
	leader := spatialmath.NewPose(r3.Vector{X: 120, Y: 40, Z: 210}, ov(0.3, 0.1, 0.9, 25))
	back := m.Inverse(m.Forward(leader))
	test.That(t, spatialmath.PoseAlmostCoincidentEps(back, leader, 1e-6), test.ShouldBeTrue)
	test.That(t, spatialmath.OrientationAlmostEqual(back.Orientation(), leader.Orientation()), test.ShouldBeTrue)
}

func TestMappingScalesTranslationAboutTheLeaderBase(t *testing.T) {
	m := Mapping{Scale: 3}
	fwd := m.Forward(spatialmath.NewPose(r3.Vector{X: 100, Y: 0, Z: 50}, spatialmath.NewZeroOrientation()))
	test.That(t, fwd.Point().X, test.ShouldAlmostEqual, 300)
	test.That(t, fwd.Point().Z, test.ShouldAlmostEqual, 150)
}

func TestMappingZeroValueIsIdentity(t *testing.T) {
	p := spatialmath.NewPose(r3.Vector{X: 1, Y: 2, Z: 3}, ov(0, 0, 1, 40))
	test.That(t, spatialmath.PoseAlmostCoincident(Mapping{}.Forward(p), p), test.ShouldBeTrue)
	test.That(t, spatialmath.PoseAlmostCoincident(Mapping{}.Inverse(p), p), test.ShouldBeTrue)
}

func TestResidualMakesMappedCoincideWithActual(t *testing.T) {
	// Non-identity on BOTH sides, so the test distinguishes a tool-frame correction from a
	// world-frame one (they agree only when mapped is the identity orientation).
	mapped := spatialmath.NewPose(r3.Vector{X: 100, Y: 0, Z: 0}, ov(0.5, 0.3, 0.8, -40))
	actual := spatialmath.NewPose(r3.Vector{X: 112, Y: -3, Z: 5}, ov(0.2, 0, 0.98, 15))
	r := ResidualBetween(mapped, actual)
	got := r.Apply(mapped)
	test.That(t, spatialmath.PoseAlmostCoincidentEps(got, actual, 1e-6), test.ShouldBeTrue)
	test.That(t, spatialmath.OrientationAlmostEqual(got.Orientation(), actual.Orientation()), test.ShouldBeTrue)
	test.That(t, r.IsZero(), test.ShouldBeFalse)
	test.That(t, ResidualBetween(mapped, mapped).IsZero(), test.ShouldBeTrue)
}

func TestResidualRotationIsAToolOffsetNotAWorldTilt(t *testing.T) {
	// A rotation residual applied to a pose that has since rotated must follow the tool,
	// which is what a right-multiplied (tool-frame) correction does.
	mapped := spatialmath.NewPose(r3.Vector{}, spatialmath.NewZeroOrientation())
	actual := spatialmath.NewPose(r3.Vector{}, ov(1, 0, 0, 0)) // tool tipped to +X
	r := ResidualBetween(mapped, actual)

	later := spatialmath.NewPose(r3.Vector{}, ov(0, 0, 1, 90)) // leader rolled 90 about Z
	got := r.Apply(later)
	expect := spatialmath.Compose(later, spatialmath.NewPoseFromOrientation(actual.Orientation()))
	test.That(t, spatialmath.OrientationAlmostEqual(got.Orientation(), expect.Orientation()), test.ShouldBeTrue)
}

func TestOrientationErrorIsAWorldFrameRotationVector(t *testing.T) {
	from := spatialmath.NewZeroOrientation()
	to := &spatialmath.R4AA{Theta: 0.3, RX: 0, RY: 0, RZ: 1}
	v := orientationError(from, to)
	test.That(t, v.Z, test.ShouldAlmostEqual, 0.3, 1e-9)
	test.That(t, math.Abs(v.X)+math.Abs(v.Y), test.ShouldBeLessThan, 1e-9)
	test.That(t, orientationError(to, to).Norm(), test.ShouldBeLessThan, 1e-9)
}
