package cartesianteleop

import (
	"fmt"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// LeaderHold is what the leader arm does with its servos once it has been aligned, i.e. how
// it is presented to the driver's hand.
type LeaderHold string

const (
	// HoldManual enters the so101 arm's manual mode: torque on at reduced gain with a torque
	// cap, so the arm holds itself against gravity but yields to a hand and stays where it is
	// left. The default, and the only mode in which a limp leader does not fall.
	HoldManual LeaderHold = "manual"
	// HoldStiff leaves the leader at full stiffness. The driver cannot move it; useful for
	// checking alignment without a hand on the arm.
	HoldStiff LeaderHold = "stiff"
	// HoldTorqueOff disables the leader's torque after alignment. The arm WILL fall unless a
	// hand is already on it, and that fall is mirrored.
	HoldTorqueOff LeaderHold = "torque_off"
)

// GripperMode is how the follower gripper is driven from the leader gripper's percentage.
type GripperMode string

const (
	// GripperModeXArm sends DoCommand {"set": steps} to a viam:ufactory gripper, with steps
	// on the xArm's 0..GripperRange scale (850 fully open).
	GripperModeXArm GripperMode = "xarm"
	// GripperModeOpenClose uses the gripper API: Open above 60 %, Grab below 40 %.
	GripperModeOpenClose GripperMode = "open_close"
	// GripperModeOff mirrors no gripper even when both are configured.
	GripperModeOff GripperMode = "off"
)

// Vec3 is a JSON-friendly millimetre vector.
type Vec3 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

// Settings is the user-facing configuration, shared verbatim between the module's service
// attributes and the standalone client's config file. Zero values select the defaults noted
// on each field.
type Settings struct {
	// Resource names. The follower component is the frame-system name the pipeline positions
	// (normally the follower's gripper, so its TCP is what tracks the leader).
	LeaderArm         string `json:"leader_arm"`
	LeaderGripper     string `json:"leader_gripper,omitempty"`
	FollowerComponent string `json:"follower_component"`
	FollowerGripper   string `json:"follower_gripper,omitempty"`
	MotionService     string `json:"motion_service,omitempty"` // default "builtin"

	// Mapping. See Mapping.
	Scale             float64                        `json:"scale,omitempty"`                 // default 1
	OriginTranslation *Vec3                          `json:"origin_translation_mm,omitempty"` // leader base in follower world
	OriginOrientation *spatialmath.OrientationConfig `json:"origin_orientation,omitempty"`
	LeaderToolOffset  *Vec3                          `json:"leader_tool_offset_mm,omitempty"` // leader tip in its EndPosition frame, applied before scale
	ToolOrientation   *spatialmath.OrientationConfig `json:"tool_orientation,omitempty"`
	RotationEnabled   *bool                          `json:"rotation_enabled,omitempty"` // default true

	// Loop and filter.
	RateHz              float64  `json:"rate_hz,omitempty"`                // default 60
	SmoothTauMs         *float64 `json:"smooth_tau_ms,omitempty"`          // default 100; 0 disables
	PosDeadzoneMM       float64  `json:"pos_deadzone_mm,omitempty"`        // default 1
	RotDeadzoneDeg      float64  `json:"rot_deadzone_deg,omitempty"`       // default 1
	OutlierPosMMPerSec  float64  `json:"outlier_pos_mm_per_s,omitempty"`   // default 4500
	OutlierRotDegPerSec float64  `json:"outlier_rot_deg_per_s,omitempty"`  // default 1800
	MoveTimeoutMs       float64  `json:"move_timeout_ms,omitempty"`        // default 500
	MaxConsecutiveErr   int      `json:"max_consecutive_errors,omitempty"` // default 10

	// Alignment and leader handling.
	AlignOnStart                   bool                   `json:"align_on_start,omitempty"`                      // default false: capture where both arms are, move nothing
	LeaderHold                     LeaderHold             `json:"leader_hold,omitempty"`                         // default "manual"
	ManualMode                     map[string]interface{} `json:"manual_mode,omitempty"`                         // enter_manual_mode overrides
	CompensateAlignResidual        *bool                  `json:"compensate_align_residual,omitempty"`           // default true
	AlignOrientationWeightMMPerRad float64                `json:"align_orientation_weight_mm_per_rad,omitempty"` // default 50
	AlignMaxPositionErrorMM        float64                `json:"align_max_position_error_mm,omitempty"`         // default 0: best effort, any gap is absorbed

	// Gripper.
	FollowerGripperMode GripperMode `json:"follower_gripper_mode,omitempty"` // default "xarm"
	GripperInvert       bool        `json:"gripper_invert,omitempty"`
	GripperRange        float64     `json:"gripper_range,omitempty"`        // default 850 (xarm mode)
	GripperDeadbandPct  float64     `json:"gripper_deadband_pct,omitempty"` // default 3
	GripperInMinPct     float64     `json:"gripper_in_min_pct,omitempty"`   // leader raw reading that maps to 0 (default 0)
	GripperInMaxPct     float64     `json:"gripper_in_max_pct,omitempty"`   // leader raw reading that maps to 100 (default 100)
	// GripperLock freezes the whole pair while the follower gripper moves: no pose targets go
	// to the follower and the leader is held stiff (manual_hold), for GripperLockMs after each
	// gripper command. For followers whose controller pauses arm motion during a gripper move
	// (the Fairino FR5), so the leader cannot run ahead of a follower that is not moving.
	GripperLock   bool    `json:"gripper_lock,omitempty"`
	GripperLockMs float64 `json:"gripper_lock_ms,omitempty"` // default 1500, max 4500

	// Blocked-follower feedback. See FeedbackConfig.
	BlockedFeedback           string  `json:"blocked_feedback,omitempty"`               // "spring" or "off" (default)
	FeedbackTorqueLimit       int     `json:"feedback_torque_limit,omitempty"`          // default 250
	FeedbackMaxSpeedDegPerSec float64 `json:"feedback_max_speed_deg_per_sec,omitempty"` // default 45
	FeedbackLagMM             float64 `json:"feedback_lag_mm,omitempty"`                // default 40
	FollowerArm               string  `json:"follower_arm,omitempty"`                   // lets planner errors naming the arm be attributed
}

// Validate checks the settings. It does not check that resources exist.
func (s Settings) Validate() error {
	if s.LeaderArm == "" {
		return fmt.Errorf("leader_arm is required")
	}
	if s.FollowerComponent == "" {
		return fmt.Errorf("follower_component is required")
	}
	if s.Scale < 0 {
		return fmt.Errorf("scale must be positive, got %v", s.Scale)
	}
	if s.RateHz < 0 || s.RateHz > 1000 {
		return fmt.Errorf("rate_hz must be in (0, 1000], got %v", s.RateHz)
	}
	if s.SmoothTauMs != nil && *s.SmoothTauMs < 0 {
		return fmt.Errorf("smooth_tau_ms must not be negative")
	}
	switch s.LeaderHold {
	case "", HoldManual, HoldStiff, HoldTorqueOff:
	default:
		return fmt.Errorf("leader_hold must be %q, %q or %q, got %q", HoldManual, HoldStiff, HoldTorqueOff, s.LeaderHold)
	}
	switch s.FollowerGripperMode {
	case "", GripperModeXArm, GripperModeOpenClose, GripperModeOff:
	default:
		return fmt.Errorf("follower_gripper_mode must be %q, %q or %q, got %q",
			GripperModeXArm, GripperModeOpenClose, GripperModeOff, s.FollowerGripperMode)
	}
	if s.GripperInMaxPct != 0 && s.GripperInMaxPct <= s.GripperInMinPct {
		return fmt.Errorf("gripper_in_max_pct (%v) must be greater than gripper_in_min_pct (%v)", s.GripperInMaxPct, s.GripperInMinPct)
	}
	if s.GripperLockMs < 0 || s.GripperLockMs > 4500 {
		return fmt.Errorf("gripper_lock_ms must be in [0, 4500], got %v", s.GripperLockMs)
	}
	switch s.BlockedFeedback {
	case "", "off":
	case "spring":
		if s.LeaderHold != "" && s.LeaderHold != HoldManual {
			return fmt.Errorf("blocked_feedback %q needs leader_hold %q (the pull-back lives in the leader's manual mode), got %q",
				s.BlockedFeedback, HoldManual, s.LeaderHold)
		}
	default:
		return fmt.Errorf("blocked_feedback must be \"spring\" or \"off\", got %q", s.BlockedFeedback)
	}
	if s.FeedbackTorqueLimit < 0 || s.FeedbackTorqueLimit > 1000 {
		return fmt.Errorf("feedback_torque_limit must be in [0,1000], got %d", s.FeedbackTorqueLimit)
	}
	if s.FeedbackMaxSpeedDegPerSec < 0 || s.FeedbackMaxSpeedDegPerSec > 180 {
		return fmt.Errorf("feedback_max_speed_deg_per_sec must be in [0,180], got %v", s.FeedbackMaxSpeedDegPerSec)
	}
	if (s.LeaderGripper == "") != (s.FollowerGripper == "") {
		return fmt.Errorf("leader_gripper and follower_gripper must be set together")
	}
	if s.OriginOrientation != nil {
		if _, err := s.OriginOrientation.ParseConfig(); err != nil {
			return fmt.Errorf("origin_orientation: %w", err)
		}
	}
	if s.ToolOrientation != nil {
		if _, err := s.ToolOrientation.ParseConfig(); err != nil {
			return fmt.Errorf("tool_orientation: %w", err)
		}
	}
	return nil
}

// MotionName returns the motion service name, defaulted.
func (s Settings) MotionName() string {
	if s.MotionService == "" {
		return "builtin"
	}
	return s.MotionService
}

// Resolve turns the settings into an engine Config with every default applied.
func (s Settings) Resolve() (Config, error) {
	if err := s.Validate(); err != nil {
		return Config{}, err
	}
	m := Mapping{Scale: s.Scale}
	var originOri spatialmath.Orientation = spatialmath.NewZeroOrientation()
	if s.OriginOrientation != nil {
		o, err := s.OriginOrientation.ParseConfig()
		if err != nil {
			return Config{}, err
		}
		originOri = o
	}
	var originPt r3.Vector
	if s.OriginTranslation != nil {
		originPt = r3.Vector{X: s.OriginTranslation.X, Y: s.OriginTranslation.Y, Z: s.OriginTranslation.Z}
	}
	m.Origin = spatialmath.NewPose(originPt, originOri)
	if s.LeaderToolOffset != nil {
		m.LeaderToolOffset = r3.Vector{X: s.LeaderToolOffset.X, Y: s.LeaderToolOffset.Y, Z: s.LeaderToolOffset.Z}
	}
	if s.ToolOrientation != nil {
		o, err := s.ToolOrientation.ParseConfig()
		if err != nil {
			return Config{}, err
		}
		m.ToolRotation = o
	}

	cfg := Config{
		Mapping:         m,
		RateHz:          pick(s.RateHz, 60),
		RotationEnabled: s.RotationEnabled == nil || *s.RotationEnabled,
		Filter: FilterConfig{
			SmoothTau:           100 * time.Millisecond,
			OutlierPosMMPerSec:  pick(s.OutlierPosMMPerSec, 4500),
			OutlierRotDegPerSec: pick(s.OutlierRotDegPerSec, 1800),
			DeadzonePosMM:       pick(s.PosDeadzoneMM, 1),
			DeadzoneRotDeg:      pick(s.RotDeadzoneDeg, 1),
		},
		IK:                      IKOptions{OrientationWeightMMPerRad: pick(s.AlignOrientationWeightMMPerRad, 50)},
		LeaderHold:              s.LeaderHold,
		AlignOnStart:            s.AlignOnStart,
		ManualOverrides:         s.ManualMode,
		CompensateResidual:      s.CompensateAlignResidual == nil || *s.CompensateAlignResidual,
		AlignMaxPositionErrorMM: s.AlignMaxPositionErrorMM, // 0 = best effort, never refuse
		FollowerGripperMode:     s.FollowerGripperMode,
		GripperInvert:           s.GripperInvert,
		GripperRange:            pick(s.GripperRange, 850),
		GripperDeadbandPct:      pick(s.GripperDeadbandPct, 3),
		GripperInMinPct:         s.GripperInMinPct,
		GripperInMaxPct:         pick(s.GripperInMaxPct, 100),
		GripperLockTorqueLimit:  int(pick(float64(s.FeedbackTorqueLimit), 250)),
		MoveTimeout:             time.Duration(pick(s.MoveTimeoutMs, 500) * float64(time.Millisecond)),
		MaxConsecutiveErrors:    s.MaxConsecutiveErr,
		Feedback: FeedbackConfig{
			Enabled:           s.BlockedFeedback == "spring",
			TorqueLimit:       s.FeedbackTorqueLimit,
			MaxSpeedDegPerSec: s.FeedbackMaxSpeedDegPerSec,
			LagMM:             s.FeedbackLagMM,
			FollowerArm:       s.FollowerArm,
		},
	}
	if s.GripperLock {
		cfg.GripperLock = time.Duration(pick(s.GripperLockMs, 1500) * float64(time.Millisecond))
	}
	if s.SmoothTauMs != nil {
		cfg.Filter.SmoothTau = time.Duration(*s.SmoothTauMs * float64(time.Millisecond))
	}
	if cfg.LeaderHold == "" {
		cfg.LeaderHold = HoldManual
	}
	if cfg.FollowerGripperMode == "" {
		cfg.FollowerGripperMode = GripperModeXArm
	}
	if cfg.MaxConsecutiveErrors <= 0 {
		cfg.MaxConsecutiveErrors = 10
	}
	return cfg, nil
}

func pick(v, def float64) float64 {
	if v == 0 {
		return def
	}
	return v
}
