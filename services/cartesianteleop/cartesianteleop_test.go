package cartesianteleop

import (
	"context"
	"errors"
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
	"go.viam.com/rdk/utils"
	"go.viam.com/test"
	"google.golang.org/protobuf/types/known/structpb"

	engine "so_arm/internal/cartesianteleop"
	"so_arm/internal/geometry"
)

// TestAttributesDecodeThroughRDK decodes a real attribute map the way viam-server does
// (resource.TransformAttributeMap, i.e. mapstructure on the json tag). Building Config as a
// Go literal, as the other tests do, bypasses that decoder entirely -- which is how an
// embedded Settings that decoded to all-zero shipped with every test green.
func TestAttributesDecodeThroughRDK(t *testing.T) {
	attrs := utils.AttributeMap{
		"leader_arm":            "right-leader-arm",
		"leader_gripper":        "right-leader-gripper",
		"follower_component":    "right-gripper",
		"follower_gripper":      "right-gripper",
		"scale":                 2.5,
		"leader_tool_offset_mm": map[string]interface{}{"x": 0.0, "y": 0.0, "z": 96.0},
		"origin_translation_mm": map[string]interface{}{"x": 0.0, "y": 1003.0, "z": 0.0},
		"origin_orientation": map[string]interface{}{
			"type": "ov_degrees", "value": map[string]interface{}{"x": 0.0, "y": 0.0, "z": 1.0, "th": 0.0},
		},
		"align_on_start":     true,
		"leader_hold":        "manual",
		"gripper_in_max_pct": 70.0,
		"auto_start":         true,
	}
	cfg, err := resource.TransformAttributeMap[*Config](attrs)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cfg.LeaderArm, test.ShouldEqual, "right-leader-arm")
	test.That(t, cfg.FollowerComponent, test.ShouldEqual, "right-gripper")
	test.That(t, cfg.Scale, test.ShouldEqual, 2.5)
	test.That(t, cfg.AlignOnStart, test.ShouldBeTrue)
	test.That(t, cfg.AutoStart, test.ShouldBeTrue)
	test.That(t, cfg.LeaderToolOffset, test.ShouldNotBeNil)
	test.That(t, cfg.LeaderToolOffset.Z, test.ShouldEqual, 96)
	test.That(t, cfg.OriginTranslation.Y, test.ShouldEqual, 1003)
	test.That(t, cfg.OriginOrientation, test.ShouldNotBeNil)

	_, _, err = cfg.Validate("")
	test.That(t, err, test.ShouldBeNil)

	resolved, err := cfg.Settings.Resolve()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resolved.Mapping.Origin.Point().Y, test.ShouldEqual, 1003)
	test.That(t, resolved.GripperInMaxPct, test.ShouldEqual, 70)
}

func TestValidateDeclaresEveryDependency(t *testing.T) {
	cfg := &Config{Settings: engine.Settings{
		LeaderArm: "leader-arm", FollowerComponent: "xarm-gripper",
		LeaderGripper: "leader-gripper", FollowerGripper: "xarm-gripper",
	}}
	deps, _, err := cfg.Validate("")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldResemble, []string{"leader-arm", "xarm-gripper", "builtin", "leader-gripper"})

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

	main := func(st map[string]interface{}) map[string]interface{} {
		return st["pairs"].(map[string]interface{})["main"].(map[string]interface{})
	}
	st, err := res.DoCommand(context.Background(), map[string]interface{}{"command": "status"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, st["running"], test.ShouldBeFalse)
	test.That(t, main(st)["rate_hz"], test.ShouldEqual, 60.0)

	st, err = res.DoCommand(context.Background(), map[string]interface{}{"command": "start"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, st["running"], test.ShouldBeTrue)
	test.That(t, main(st)["align_position_error_mm"], test.ShouldBeLessThan, 0.5)
	// DoCommand results cross gRPC as a protobuf Struct; a []float64 would fail there.
	_, err = structpb.NewStruct(st)
	test.That(t, err, test.ShouldBeNil)

	st, err = res.DoCommand(context.Background(), map[string]interface{}{"command": "stop"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, st["running"], test.ShouldBeFalse)

	_, err = res.DoCommand(context.Background(), map[string]interface{}{"command": "dance"})
	test.That(t, err, test.ShouldNotBeNil)
}

// pairsAttrs is a two-pair config in the shape the cart uses: shared defaults at the top,
// per-side arm names and origin in each pair.
func pairsAttrs() utils.AttributeMap {
	return utils.AttributeMap{
		"scale":                 2.5,
		"leader_tool_offset_mm": map[string]interface{}{"x": 0.0, "y": 0.0, "z": 96.0},
		"leader_hold":           "stiff",
		"gripper_in_max_pct":    70.0,
		"pairs": []interface{}{
			map[string]interface{}{
				"name": "left", "leader_arm": "left-leader-arm", "follower_component": "left-gripper",
				"origin_translation_mm": map[string]interface{}{"x": 0.0, "y": 1003.0, "z": 0.0},
			},
			map[string]interface{}{
				"name": "right", "leader_arm": "right-leader-arm", "follower_component": "right-gripper",
				"scale": 1.5, // a pair may override a shared default
			},
		},
	}
}

func TestPairsInheritDefaultsAndOverride(t *testing.T) {
	cfg, err := resource.TransformAttributeMap[*Config](pairsAttrs())
	test.That(t, err, test.ShouldBeNil)
	pairs, err := cfg.resolvePairs()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(pairs), test.ShouldEqual, 2)

	left, right := pairs[0], pairs[1]
	test.That(t, left.Name, test.ShouldEqual, "left")
	test.That(t, left.Settings.LeaderArm, test.ShouldEqual, "left-leader-arm")
	test.That(t, left.Settings.Scale, test.ShouldEqual, 2.5) // inherited
	test.That(t, left.Settings.LeaderToolOffset.Z, test.ShouldEqual, 96)
	test.That(t, left.Settings.OriginTranslation.Y, test.ShouldEqual, 1003)
	test.That(t, left.Settings.GripperInMaxPct, test.ShouldEqual, 70)
	test.That(t, right.Settings.Scale, test.ShouldEqual, 1.5) // overridden
	test.That(t, right.Settings.OriginTranslation, test.ShouldBeNil)
	test.That(t, right.Settings.LeaderToolOffset.Z, test.ShouldEqual, 96)

	deps, _, err := cfg.Validate("")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldResemble,
		[]string{"left-leader-arm", "left-gripper", "builtin", "right-leader-arm", "right-gripper"})
}

func TestPairsValidation(t *testing.T) {
	attrs := pairsAttrs()
	attrs["pairs"] = []interface{}{map[string]interface{}{"leader_arm": "a", "follower_component": "b"}}
	cfg, err := resource.TransformAttributeMap[*Config](attrs)
	test.That(t, err, test.ShouldBeNil)
	_, _, err = cfg.Validate("")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "name is required")

	attrs["pairs"] = []interface{}{
		map[string]interface{}{"name": "x", "leader_arm": "a", "follower_component": "b"},
		map[string]interface{}{"name": "x", "leader_arm": "c", "follower_component": "d"},
	}
	cfg, _ = resource.TransformAttributeMap[*Config](attrs)
	_, _, err = cfg.Validate("")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "duplicate")

	// A pair missing a required key names the pair in the error.
	attrs["pairs"] = []interface{}{map[string]interface{}{"name": "left", "leader_arm": "a"}}
	cfg, _ = resource.TransformAttributeMap[*Config](attrs)
	_, _, err = cfg.Validate("")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "left")
	test.That(t, err.Error(), test.ShouldContainSubstring, "follower_component")
}

func TestPairsStartStopPerSideAndAll(t *testing.T) {
	model, err := geometry.ArmModelJSON("leader")
	test.That(t, err, test.ShouldBeNil)
	newLeader := func(name string) *inject.Arm {
		joints := []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0}
		a := inject.NewArm(name)
		a.KinematicsFunc = func(context.Context) (referenceframe.Model, error) { return model, nil }
		a.JointPositionsFunc = func(context.Context, map[string]interface{}) ([]referenceframe.Input, error) { return joints, nil }
		a.EndPositionFunc = func(context.Context, map[string]interface{}) (spatialmath.Pose, error) {
			return model.Transform(joints)
		}
		a.MoveToJointPositionsFunc = func(_ context.Context, p []referenceframe.Input, _ map[string]interface{}) error {
			joints = p
			return nil
		}
		a.DoFunc = func(context.Context, map[string]interface{}) (map[string]interface{}, error) {
			return map[string]interface{}{}, nil
		}
		return a
	}
	pose, err := model.Transform([]referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0})
	test.That(t, err, test.ShouldBeNil)
	deps := resource.Dependencies{
		arm.Named("left-leader-arm"):   newLeader("left-leader-arm"),
		arm.Named("right-leader-arm"):  newLeader("right-leader-arm"),
		gripper.Named("left-gripper"):  inject.NewGripper("left-gripper"),
		gripper.Named("right-gripper"): inject.NewGripper("right-gripper"),
		motion.Named("builtin"):        &fakeMotion{pose: pose},
	}
	cfg, err := resource.TransformAttributeMap[*Config](pairsAttrs())
	test.That(t, err, test.ShouldBeNil)
	conf := resource.Config{Name: "teleop", API: generic.API, Model: Model, ConvertedAttributes: cfg}
	res, err := newService(context.Background(), deps, conf, logging.NewTestLogger(t))
	test.That(t, err, test.ShouldBeNil)
	defer res.Close(context.Background())
	ctx := context.Background()

	running := func(st map[string]interface{}, side string) bool {
		return st["pairs"].(map[string]interface{})[side].(map[string]interface{})["running"].(bool)
	}

	st, err := res.DoCommand(ctx, map[string]interface{}{"command": "start", "side": "left"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, st["running"], test.ShouldBeTrue)
	test.That(t, len(st["pairs"].(map[string]interface{})), test.ShouldEqual, 1) // scoped to the side

	st, err = res.DoCommand(ctx, map[string]interface{}{"command": "status"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, running(st, "left"), test.ShouldBeTrue)
	test.That(t, running(st, "right"), test.ShouldBeFalse)

	st, err = res.DoCommand(ctx, map[string]interface{}{"command": "start"}) // all; left already running
	test.That(t, err, test.ShouldBeNil)
	test.That(t, running(st, "left"), test.ShouldBeTrue)
	test.That(t, running(st, "right"), test.ShouldBeTrue)

	st, err = res.DoCommand(ctx, map[string]interface{}{"command": "stop"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, st["running"], test.ShouldBeFalse)

	_, err = res.DoCommand(ctx, map[string]interface{}{"command": "start", "side": "middle"})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "unknown side")
}

// TestStartLogsEachPairsFailureByName pins that a pair failing to start shows up in the
// machine logs under its own name. The joined error only reaches the DoCommand caller --
// on hardware a start that failed after the align move left the leader stiff at the aligned
// pose with nothing in the logs to say why -- and the other pair must still start.
func TestStartLogsEachPairsFailureByName(t *testing.T) {
	model, err := geometry.ArmModelJSON("leader")
	test.That(t, err, test.ShouldBeNil)
	newLeader := func(name string, moveErr error) *inject.Arm {
		joints := []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0}
		a := inject.NewArm(name)
		a.KinematicsFunc = func(context.Context) (referenceframe.Model, error) { return model, nil }
		a.JointPositionsFunc = func(context.Context, map[string]interface{}) ([]referenceframe.Input, error) { return joints, nil }
		a.EndPositionFunc = func(context.Context, map[string]interface{}) (spatialmath.Pose, error) {
			return model.Transform(joints)
		}
		a.MoveToJointPositionsFunc = func(_ context.Context, p []referenceframe.Input, _ map[string]interface{}) error {
			if moveErr != nil {
				return moveErr
			}
			joints = p
			return nil
		}
		a.DoFunc = func(context.Context, map[string]interface{}) (map[string]interface{}, error) {
			return map[string]interface{}{}, nil
		}
		return a
	}
	pose, err := model.Transform([]referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0})
	test.That(t, err, test.ShouldBeNil)
	deps := resource.Dependencies{
		arm.Named("left-leader-arm"):   newLeader("left-leader-arm", nil),
		arm.Named("right-leader-arm"):  newLeader("right-leader-arm", errors.New("servo 4 sync_read failed: [checksum]")),
		gripper.Named("left-gripper"):  inject.NewGripper("left-gripper"),
		gripper.Named("right-gripper"): inject.NewGripper("right-gripper"),
		motion.Named("builtin"):        &fakeMotion{pose: pose},
	}
	attrs := pairsAttrs()
	attrs["align_on_start"] = true // the failure is in the align move
	cfg, err := resource.TransformAttributeMap[*Config](attrs)
	test.That(t, err, test.ShouldBeNil)
	conf := resource.Config{Name: "teleop", API: generic.API, Model: Model, ConvertedAttributes: cfg}
	logger, logs := logging.NewObservedTestLogger(t)
	res, err := newService(context.Background(), deps, conf, logger)
	test.That(t, err, test.ShouldBeNil)
	defer res.Close(context.Background())
	ctx := context.Background()

	_, err = res.DoCommand(ctx, map[string]interface{}{"command": "start"})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "right: ")
	test.That(t, err.Error(), test.ShouldContainSubstring, "checksum")

	test.That(t, logs.FilterMessageSnippet(`pair "right": start failed`).Len(), test.ShouldEqual, 1)
	test.That(t, logs.FilterMessageSnippet("checksum").Len(), test.ShouldEqual, 1)
	test.That(t, logs.FilterMessageSnippet(`pair "left"`).Len(), test.ShouldEqual, 0)

	st, err := res.DoCommand(ctx, map[string]interface{}{"command": "status"})
	test.That(t, err, test.ShouldBeNil)
	pairs := st["pairs"].(map[string]interface{})
	test.That(t, pairs["left"].(map[string]interface{})["running"], test.ShouldBeTrue)
	test.That(t, pairs["right"].(map[string]interface{})["running"], test.ShouldBeFalse)
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
