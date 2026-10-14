package cartesianteleop

import (
	"context"
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	"so_arm/internal/geometry"
)

func leaderModel(t *testing.T) referenceframe.Model {
	t.Helper()
	m, err := geometry.ArmModelJSON("leader")
	test.That(t, err, test.ShouldBeNil)
	return m
}

func TestSolveBestEffortReachesAReachablePose(t *testing.T) {
	model := leaderModel(t)
	// Forward kinematics of a real configuration is reachable by construction.
	truth := []referenceframe.Input{0.3, -0.6, 0.9, 0.4, -0.2}
	goal, err := model.Transform(truth)
	test.That(t, err, test.ShouldBeNil)

	seed := []referenceframe.Input{0, 0, 0, 0, 0}
	res, err := SolveBestEffort(context.Background(), model, seed, goal, IKOptions{})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res.PositionErrorMM, test.ShouldBeLessThan, 0.5)
	test.That(t, res.OrientationErrorDeg, test.ShouldBeLessThan, 0.5)
	test.That(t, len(res.Inputs), test.ShouldEqual, 5)
	for i, l := range model.DoF() {
		test.That(t, res.Inputs[i], test.ShouldBeBetweenOrEqual, l.Min, l.Max)
	}
}

func TestSolveBestEffortGetsCloseToAnUnreachableOrientation(t *testing.T) {
	model := leaderModel(t)
	truth := []referenceframe.Input{0.2, -0.5, 0.8, 0.3, 0}
	reachable, err := model.Transform(truth)
	test.That(t, err, test.ShouldBeNil)

	// Same position, but twist the tool about an axis a 5-DoF wrist cannot command
	// independently. The solver must keep the position and report the orientation residual
	// instead of failing.
	twisted := spatialmath.Compose(reachable, spatialmath.NewPoseFromOrientation(
		&spatialmath.OrientationVectorDegrees{OX: 1, OY: 0, OZ: 0, Theta: 0}))
	res, err := SolveBestEffort(context.Background(), model, []referenceframe.Input{0, 0, 0, 0, 0}, twisted, IKOptions{})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res.PositionErrorMM, test.ShouldBeLessThan, 25)
	test.That(t, res.OrientationErrorDeg, test.ShouldBeGreaterThan, 1) // genuinely unreachable
	test.That(t, res.OrientationErrorDeg, test.ShouldBeLessThan, 90)
}

func TestSolveBestEffortNeverFailsForAnOutOfReachPosition(t *testing.T) {
	model := leaderModel(t)
	farAway := spatialmath.NewPose(r3.Vector{X: 2000, Y: 0, Z: 300}, spatialmath.NewZeroOrientation())
	res, err := SolveBestEffort(context.Background(), model, []referenceframe.Input{0, 0, 0, 0, 0}, farAway, IKOptions{})
	test.That(t, err, test.ShouldBeNil)
	// It stretched toward the goal: the residual is far smaller than the 2 m request but
	// the arm is only ~0.5 m long, so it cannot be zero.
	test.That(t, res.PositionErrorMM, test.ShouldBeGreaterThan, 1000)
	test.That(t, res.PositionErrorMM, test.ShouldBeLessThan, 1900)
	test.That(t, math.IsNaN(res.Cost), test.ShouldBeFalse)
}

func TestSolveBestEffortIsReproducible(t *testing.T) {
	model := leaderModel(t)
	goal := spatialmath.NewPose(r3.Vector{X: 150, Y: 80, Z: 120}, &spatialmath.OrientationVectorDegrees{OX: 0, OY: 0, OZ: -1, Theta: 0})
	a, err := SolveBestEffort(context.Background(), model, []referenceframe.Input{0, 0, 0, 0, 0}, goal, IKOptions{})
	test.That(t, err, test.ShouldBeNil)
	b, err := SolveBestEffort(context.Background(), model, []referenceframe.Input{0, 0, 0, 0, 0}, goal, IKOptions{})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, a.Inputs, test.ShouldResemble, b.Inputs)
}

func TestSolveBestEffortRejectsAWrongSeedLength(t *testing.T) {
	model := leaderModel(t)
	_, err := SolveBestEffort(context.Background(), model, []referenceframe.Input{0}, spatialmath.NewZeroPose(), IKOptions{})
	test.That(t, err, test.ShouldNotBeNil)
}
