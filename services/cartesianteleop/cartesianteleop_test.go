package cartesianteleop

import (
	"context"
	"testing"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/test"

	engine "so_arm/internal/cartesianteleop"
	"so_arm/internal/geometry"
)

func TestValidateDeclaresEveryDependency(t *testing.T) {
	cfg := &Config{Settings: engine.Settings{
		LeaderArm: "leader-arm", FollowerComponent: "xarm-gripper",
		LeaderGripper: "leader-gripper", FollowerGripper: "xarm-gripper",
	}}
	deps, _, err := cfg.Validate("")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldResemble, []string{"leader-arm", "xarm-gripper", "builtin", "leader-gripper", "xarm-gripper"})

	cfg.MotionService = "motion-2"
	deps, _, err = cfg.Validate("")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldContain, "motion-2")

	_, _, err = (&Config{}).Validate("")
	test.That(t, err, test.ShouldNotBeNil)
}

func TestConstructorWiresTheEngineAndStatusReportsIdle(t *testing.T) {
	model, err := geometry.ArmModelJSON("leader")
	test.That(t, err, test.ShouldBeNil)
	joints := []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0}

	leader := inject.NewArm("leader-arm")
	leader.KinematicsFunc = func(context.Context) (referenceframe.Model, error) { return model, nil }
	leader.JointPositionsFunc = func(context.Context, map[string]interface{}) ([]referenceframe.Input, error) { return joints, nil }
	leader.EndPositionFunc = func(context.Context, map[string]interface{}) (spatialmath.Pose, error) {
		return model.Transform(joints)
	}
	leader.MoveToJointPositionsFunc = func(_ context.Context, p []referenceframe.Input, _ map[string]interface{}) error {
		joints = p
		return nil
	}
	leader.DoFunc = func(context.Context, map[string]interface{}) (map[string]interface{}, error) {
		return map[string]interface{}{}, nil
	}

	followerPose, err := model.Transform(joints)
	test.That(t, err, test.ShouldBeNil)
	ms := &fakeMotion{pose: followerPose}

	deps := resource.Dependencies{
		arm.Named("leader-arm"):       leader,
		gripper.Named("xarm-gripper"): inject.NewGripper("xarm-gripper"),
		motion.Named("builtin"):       ms,
	}
	conf := resource.Config{
		Name: "teleop-cartesian", API: generic.API, Model: Model,
		ConvertedAttributes: &Config{Settings: engine.Settings{LeaderArm: "leader-arm", FollowerComponent: "xarm-gripper", LeaderHold: engine.HoldStiff}},
	}
	res, err := newService(context.Background(), deps, conf, logging.NewTestLogger(t))
	test.That(t, err, test.ShouldBeNil)
	defer res.Close(context.Background())

	st, err := res.DoCommand(context.Background(), map[string]interface{}{"command": "status"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, st["running"], test.ShouldBeFalse)
	test.That(t, st["rate_hz"], test.ShouldEqual, 60.0)

	st, err = res.DoCommand(context.Background(), map[string]interface{}{"command": "start"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, st["running"], test.ShouldBeTrue)
	test.That(t, st["align_position_error_mm"], test.ShouldBeLessThan, 0.5)

	st, err = res.DoCommand(context.Background(), map[string]interface{}{"command": "stop"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, st["running"], test.ShouldBeFalse)

	_, err = res.DoCommand(context.Background(), map[string]interface{}{"command": "dance"})
	test.That(t, err, test.ShouldNotBeNil)
}

// fakeMotion stands in for motion.Service without importing rdk's inject/motion package,
// which exists separately precisely because it drags in the NLopt (cgo) dependency.
type fakeMotion struct {
	motion.Service
	pose spatialmath.Pose
}

func (m *fakeMotion) Name() resource.Name { return motion.Named("builtin") }

func (m *fakeMotion) GetPose(_ context.Context, _, dst string, _ []*referenceframe.LinkInFrame, _ map[string]interface{}) (*referenceframe.PoseInFrame, error) {
	return referenceframe.NewPoseInFrame(dst, m.pose), nil
}

func (m *fakeMotion) DoCommand(context.Context, map[string]interface{}) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}
