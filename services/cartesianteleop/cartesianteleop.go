// Package cartesianteleop implements devrel:so101:teleop-cartesian, which mirrors a leader
// SO-101's end-effector pose onto a follower arm of different kinematics (an xArm, say)
// through the builtin motion service's teleop pipeline.
//
// The service is a thin resource wrapper; the loop, alignment and mapping live in
// internal/cartesianteleop so the standalone client (cmd/teleop-client) runs the same code.
package cartesianteleop

import (
	"context"
	"fmt"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"

	engine "so_arm/internal/cartesianteleop"
)

// Model is devrel:so101:teleop-cartesian.
var Model = resource.NewModel("devrel", "so101", "teleop-cartesian")

func init() {
	resource.RegisterService(
		generic.API,
		Model,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newService,
		})
}

// Config is the service's attributes: the engine Settings plus auto_start.
type Config struct {
	engine.Settings
	AutoStart bool `json:"auto_start,omitempty"`
}

// Validate checks the attributes and declares the dependencies. The follower component is
// declared so the frame system contains it before the service starts, even though the
// service never calls it directly (the motion service does).
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if err := cfg.Settings.Validate(); err != nil {
		return nil, nil, err
	}
	deps := []string{cfg.LeaderArm, cfg.FollowerComponent, cfg.MotionName()}
	if cfg.LeaderGripper != "" {
		deps = append(deps, cfg.LeaderGripper)
	}
	if cfg.FollowerGripper != "" {
		deps = append(deps, cfg.FollowerGripper)
	}
	return deps, nil, nil
}

type service struct {
	resource.Named
	resource.AlwaysRebuild
	logger logging.Logger
	engine *engine.Engine
}

func newService(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (resource.Resource, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}
	engineCfg, err := cfg.Settings.Resolve()
	if err != nil {
		return nil, err
	}

	leader, err := arm.FromDependencies(deps, cfg.LeaderArm)
	if err != nil {
		return nil, fmt.Errorf("failed to get leader_arm %q: %w", cfg.LeaderArm, err)
	}
	motionSvc, err := motion.FromDependencies(deps, cfg.MotionName())
	if err != nil {
		return nil, fmt.Errorf("failed to get motion_service %q: %w", cfg.MotionName(), err)
	}
	var leaderGripper, followerGripper gripper.Gripper
	if cfg.LeaderGripper != "" {
		if leaderGripper, err = gripper.FromDependencies(deps, cfg.LeaderGripper); err != nil {
			return nil, fmt.Errorf("failed to get leader_gripper %q: %w", cfg.LeaderGripper, err)
		}
	}
	if cfg.FollowerGripper != "" {
		if followerGripper, err = gripper.FromDependencies(deps, cfg.FollowerGripper); err != nil {
			return nil, fmt.Errorf("failed to get follower_gripper %q: %w", cfg.FollowerGripper, err)
		}
	}

	eDeps := engine.Deps{
		Leader:   leader,
		Pipeline: engine.NewPipeline(motionSvc, cfg.MotionName(), cfg.FollowerComponent),
		Logger:   logger,
	}
	// A nil interface must stay nil: a typed nil gripper would be "present" to the engine.
	if leaderGripper != nil {
		eDeps.LeaderGripper = leaderGripper
	}
	if followerGripper != nil {
		eDeps.FollowerGripper = followerGripper
	}

	s := &service{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
		engine: engine.New(eDeps, engineCfg),
	}
	if cfg.AutoStart {
		if err := s.engine.Start(ctx); err != nil {
			return nil, fmt.Errorf("auto_start failed: %w", err)
		}
	}
	return s, nil
}

// DoCommand accepts start, stop and status.
func (s *service) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	switch cmd["command"] {
	case "start":
		if err := s.engine.Start(ctx); err != nil {
			return nil, err
		}
		return s.statusMap(), nil
	case "stop":
		if err := s.engine.Stop(ctx); err != nil {
			return nil, err
		}
		return map[string]interface{}{"running": false}, nil
	case "status":
		return s.statusMap(), nil
	default:
		return nil, fmt.Errorf("unknown command: %v", cmd["command"])
	}
}

func (s *service) statusMap() map[string]interface{} {
	st := s.engine.Status()
	out := map[string]interface{}{
		"running":                     st.Running,
		"cycles":                      st.Cycles,
		"sends":                       st.Sends,
		"dropped":                     st.Dropped,
		"deadzoned":                   st.Deadzoned,
		"rejected":                    st.Rejected,
		"consecutive_errors":          st.ConsecutiveErrors,
		"last_error":                  st.LastError,
		"last_rpc_ms":                 st.LastRPCMs,
		"rate_hz":                     st.RateHz,
		"align_position_error_mm":     st.AlignPosErrMM,
		"align_orientation_error_deg": st.AlignOriErrDeg,
		"gripper_pct":                 st.GripperPct,
	}
	if al := s.engine.Align(); al != nil {
		out["align_leader_joints"] = al.LeaderInputs
		out["align_poses"] = al.PosesMap()
	}
	return out
}

func (s *service) Close(ctx context.Context) error {
	return s.engine.Stop(ctx)
}
