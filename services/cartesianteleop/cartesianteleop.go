// Package cartesianteleop implements devrel:so101:teleop-cartesian, which mirrors one or more
// leader SO-101s' end-effector poses onto follower arms of different kinematics (xArms, say)
// through the builtin motion service's teleop pipeline.
//
// One service drives every leader/follower pair on a machine, like the vive module's "hands"
// list: top-level attributes are defaults shared by every pair, and each entry of "pairs"
// overrides what it needs (normally just the arm names and the origin). start/stop/status act
// on every pair unless given a "side". The service is a thin resource wrapper; the loop,
// alignment and mapping live in internal/cartesianteleop so the standalone client
// (cmd/teleop-client) runs the same code.
package cartesianteleop

import (
	"context"
	"encoding/json"
	"errors"
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

// defaultPairName names the single pair of a config that has no "pairs" list.
const defaultPairName = "main"

func init() {
	resource.RegisterService(
		generic.API,
		Model,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newService,
		})
}

// Config is the service's attributes.
//
// The embedded Settings are the defaults every pair inherits. With no "pairs" they ARE the
// one pair (named "main"), so a single-arm config is just the flat attribute block the
// standalone client takes. Each entry of Pairs is a "name" plus any Settings keys, overlaid on
// the defaults key by key.
//
// The ",squash" is load-bearing. rdk decodes attributes with mapstructure keyed on the json
// tag and WITHOUT its global Squash option, so an untagged embedded struct is read as a
// nested "Settings" key and every attribute is silently dropped -- the service then fails
// validation with "leader_arm is required" no matter what the config says. encoding/json
// ignores the unknown option and inlines the (unnamed) embedded struct as before.
// TestAttributesDecodeThroughRDK pins this.
type Config struct {
	engine.Settings `json:",squash"`
	Pairs           []map[string]interface{} `json:"pairs,omitempty"`
	AutoStart       bool                     `json:"auto_start,omitempty"`
}

// pair is one resolved leader/follower pair.
type pair struct {
	Name     string
	Settings engine.Settings
}

// resolvePairs overlays each pair on the shared defaults. It works at the JSON level so that
// "set in the pair" and "set in the defaults" mean exactly what they mean in the client's
// config files: a key present in the pair wins, an absent key inherits.
func (cfg *Config) resolvePairs() ([]pair, error) {
	if len(cfg.Pairs) == 0 {
		return []pair{{Name: defaultPairName, Settings: cfg.Settings}}, nil
	}
	base, err := json.Marshal(cfg.Settings)
	if err != nil {
		return nil, err
	}
	var baseMap map[string]interface{}
	if err := json.Unmarshal(base, &baseMap); err != nil {
		return nil, err
	}

	out := make([]pair, 0, len(cfg.Pairs))
	seen := map[string]bool{}
	for i, overrides := range cfg.Pairs {
		name, _ := overrides["name"].(string)
		if name == "" {
			return nil, fmt.Errorf("pairs[%d]: name is required", i)
		}
		if seen[name] {
			return nil, fmt.Errorf("pairs[%d]: duplicate name %q", i, name)
		}
		seen[name] = true

		merged := make(map[string]interface{}, len(baseMap)+len(overrides))
		for k, v := range baseMap {
			merged[k] = v
		}
		for k, v := range overrides {
			if k != "name" {
				merged[k] = v
			}
		}
		raw, err := json.Marshal(merged)
		if err != nil {
			return nil, fmt.Errorf("pairs[%d] (%s): %w", i, name, err)
		}
		var s engine.Settings
		if err := json.Unmarshal(raw, &s); err != nil {
			return nil, fmt.Errorf("pairs[%d] (%s): %w", i, name, err)
		}
		if err := s.Validate(); err != nil {
			return nil, fmt.Errorf("pairs[%d] (%s): %w", i, name, err)
		}
		out = append(out, pair{Name: name, Settings: s})
	}
	return out, nil
}

// Validate checks the attributes and declares the dependencies of every pair. Each follower
// component is declared so the frame system contains it before the service starts, even
// though the service never calls it directly (the motion service does).
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	pairs, err := cfg.resolvePairs()
	if err != nil {
		return nil, nil, err
	}
	var deps []string
	seen := map[string]bool{}
	add := func(names ...string) {
		for _, n := range names {
			if n != "" && !seen[n] {
				seen[n] = true
				deps = append(deps, n)
			}
		}
	}
	for _, p := range pairs {
		if err := p.Settings.Validate(); err != nil {
			if len(cfg.Pairs) == 0 {
				return nil, nil, err
			}
			return nil, nil, fmt.Errorf("pair %q: %w", p.Name, err)
		}
		add(p.Settings.LeaderArm, p.Settings.FollowerComponent, p.Settings.MotionName(),
			p.Settings.LeaderGripper, p.Settings.FollowerGripper)
	}
	return deps, nil, nil
}

type namedEngine struct {
	name   string
	engine *engine.Engine
}

type service struct {
	resource.Named
	resource.AlwaysRebuild
	logger  logging.Logger
	engines []namedEngine
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
	pairs, err := cfg.resolvePairs()
	if err != nil {
		return nil, err
	}

	s := &service{Named: conf.ResourceName().AsNamed(), logger: logger}
	for _, p := range pairs {
		e, err := buildEngine(deps, p, logger)
		if err != nil {
			return nil, err
		}
		s.engines = append(s.engines, namedEngine{name: p.Name, engine: e})
	}
	if cfg.AutoStart {
		if _, err := s.forEach(ctx, "", "start", func(ctx context.Context, e *engine.Engine) error { return e.Start(ctx) }); err != nil {
			return nil, fmt.Errorf("auto_start failed: %w", err)
		}
	}
	return s, nil
}

func buildEngine(deps resource.Dependencies, p pair, logger logging.Logger) (*engine.Engine, error) {
	s := p.Settings
	engineCfg, err := s.Resolve()
	if err != nil {
		return nil, fmt.Errorf("pair %q: %w", p.Name, err)
	}
	leader, err := arm.FromDependencies(deps, s.LeaderArm)
	if err != nil {
		return nil, fmt.Errorf("pair %q: failed to get leader_arm %q: %w", p.Name, s.LeaderArm, err)
	}
	motionSvc, err := motion.FromDependencies(deps, s.MotionName())
	if err != nil {
		return nil, fmt.Errorf("pair %q: failed to get motion_service %q: %w", p.Name, s.MotionName(), err)
	}
	eDeps := engine.Deps{
		Leader:   leader,
		Pipeline: engine.NewPipeline(motionSvc, s.MotionName(), s.FollowerComponent),
		Logger:   logger.Sublogger(p.Name),
	}
	// A nil interface must stay nil: a typed nil gripper would be "present" to the engine,
	// so assign only on success rather than through a typed variable.
	if s.LeaderGripper != "" {
		g, err := gripper.FromDependencies(deps, s.LeaderGripper)
		if err != nil {
			return nil, fmt.Errorf("pair %q: failed to get leader_gripper %q: %w", p.Name, s.LeaderGripper, err)
		}
		eDeps.LeaderGripper = g
	}
	if s.FollowerGripper != "" {
		g, err := gripper.FromDependencies(deps, s.FollowerGripper)
		if err != nil {
			return nil, fmt.Errorf("pair %q: failed to get follower_gripper %q: %w", p.Name, s.FollowerGripper, err)
		}
		eDeps.FollowerGripper = g
	}
	return engine.New(eDeps, engineCfg), nil
}

// forEach runs fn on the named pair, or on every pair when side is empty. Every pair is
// attempted even if an earlier one fails -- one side's align failing must not leave the other
// side unstarted (or, for stop, still holding its leader). It returns the names it acted on.
// Each pair's failure is also logged under its own name: the joined error only reaches the
// DoCommand caller, and a start that fails after the align move leaves the leader stiff at
// the aligned pose with nothing in the machine logs to say why.
func (s *service) forEach(ctx context.Context, side, what string, fn func(context.Context, *engine.Engine) error) ([]string, error) {
	var names []string
	var errs []error
	for _, ne := range s.engines {
		if side != "" && ne.name != side {
			continue
		}
		names = append(names, ne.name)
		if err := fn(ctx, ne.engine); err != nil {
			s.logger.Errorf("pair %q: %s failed: %v", ne.name, what, err)
			errs = append(errs, fmt.Errorf("%s: %w", ne.name, err))
		}
	}
	if side != "" && len(names) == 0 {
		return nil, fmt.Errorf("unknown side %q; configured: %v", side, s.pairNames())
	}
	return names, errors.Join(errs...)
}

func (s *service) pairNames() []string {
	out := make([]string, len(s.engines))
	for i, ne := range s.engines {
		out[i] = ne.name
	}
	return out
}

// DoCommand accepts start, stop and status, each optionally scoped with "side": "<pair name>".
// start reports a failing pair as an error but still starts the others; status then shows
// which ones are running.
func (s *service) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	side, _ := cmd["side"].(string)
	switch cmd["command"] {
	case "start":
		_, err := s.forEach(ctx, side, "start", func(ctx context.Context, e *engine.Engine) error { return e.Start(ctx) })
		if err != nil {
			return nil, err
		}
		return s.statusMap(side)
	case "stop":
		_, err := s.forEach(ctx, side, "stop", func(ctx context.Context, e *engine.Engine) error { return e.Stop(ctx) })
		if err != nil {
			return nil, err
		}
		return s.statusMap(side)
	case "status":
		return s.statusMap(side)
	default:
		return nil, fmt.Errorf("unknown command: %v", cmd["command"])
	}
}

// statusMap reports {"running": any pair running, "pairs": {name: status}}. Values are kept to
// structpb-native types (no []float64), because DoCommand results cross gRPC as a Struct.
func (s *service) statusMap(side string) (map[string]interface{}, error) {
	pairs := map[string]interface{}{}
	anyRunning := false
	for _, ne := range s.engines {
		if side != "" && ne.name != side {
			continue
		}
		st := ne.engine.Status()
		anyRunning = anyRunning || st.Running
		ps := map[string]interface{}{
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
			"blocked":                     st.Blocked,
			"blocked_reason":              st.BlockedReason,
		}
		if al := ne.engine.Align(); al != nil {
			joints := make([]interface{}, len(al.LeaderInputs))
			for i, v := range al.LeaderInputs {
				joints[i] = float64(v)
			}
			ps["align_leader_joints"] = joints
			ps["align_poses"] = al.PosesMap()
		}
		pairs[ne.name] = ps
	}
	if side != "" && len(pairs) == 0 {
		return nil, fmt.Errorf("unknown side %q; configured: %v", side, s.pairNames())
	}
	return map[string]interface{}{"running": anyRunning, "pairs": pairs}, nil
}

func (s *service) Close(ctx context.Context) error {
	_, err := s.forEach(ctx, "", "stop", func(ctx context.Context, e *engine.Engine) error { return e.Stop(ctx) })
	return err
}
