package cartesianteleop

import (
	"math"
	"time"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// Debug pose logging. Every pose is reported as a flat map of rounded float64s
// ({x, y, z} in mm, {ox, oy, oz, theta} as an orientation vector in degrees) so the same
// value can go into a structured log line and, unchanged, into a structpb DoCommand result.

// poseLogInterval rate-limits the run loop's pose line to one per pair per interval.
const poseLogInterval = time.Second

func round1(v float64) float64 { return math.Round(v*10) / 10 }

// poseMap renders p as {x, y, z, ox, oy, oz, theta}, rounded to 0.1 mm / 0.1 deg. nil → nil.
func poseMap(p spatialmath.Pose) map[string]interface{} {
	if p == nil {
		return nil
	}
	pt := p.Point()
	ov := p.Orientation().OrientationVectorDegrees()
	return map[string]interface{}{
		"x": round1(pt.X), "y": round1(pt.Y), "z": round1(pt.Z),
		"ox": round1(ov.OX), "oy": round1(ov.OY), "oz": round1(ov.OZ), "theta": round1(ov.Theta),
	}
}

// inputsDeg renders revolute joint inputs (radians) as rounded degrees.
func inputsDeg(in []referenceframe.Input) []interface{} {
	out := make([]interface{}, len(in))
	for i, v := range in {
		out[i] = round1(float64(v) * 180 / math.Pi)
	}
	return out
}

// residualMap renders the residual's translation (mm) and rotation (orientation vector, deg,
// plus its total angle).
func residualMap(r Residual) map[string]interface{} {
	m := map[string]interface{}{
		"x": round1(r.Translation.X), "y": round1(r.Translation.Y), "z": round1(r.Translation.Z),
		"translation_mm": round1(r.Translation.Norm()),
		"rotation_deg":   0.0,
	}
	if r.Rotation != nil {
		ov := r.Rotation.OrientationVectorDegrees()
		m["ox"], m["oy"], m["oz"], m["theta"] = round1(ov.OX), round1(ov.OY), round1(ov.OZ), round1(ov.Theta)
		m["rotation_deg"] = round1(orientationError(spatialmath.NewZeroOrientation(), r.Rotation).Norm() * 180 / math.Pi)
	}
	return m
}

// PosesMap is the alignment's poses for the status DoCommand: structpb-compatible (only
// float64, string, bool, []interface{} and map[string]interface{}).
func (al *Alignment) PosesMap() map[string]interface{} {
	m := map[string]interface{}{
		"aligned":               al.Aligned,
		"position_error_mm":     round1(al.PositionErrorMM),
		"orientation_error_deg": round1(al.OrientationErrorDeg),
		"leader_inputs_deg":     inputsDeg(al.LeaderInputs),
		"residual":              residualMap(al.Residual),
	}
	for k, p := range map[string]spatialmath.Pose{
		"follower_pose": al.FollowerPose,
		"leader_target": al.LeaderTarget,
		"leader_actual": al.LeaderActual,
		"mapped":        al.Mapped,
	} {
		if pm := poseMap(p); pm != nil {
			m[k] = pm
		}
	}
	return m
}

// logAlign writes the one structured align line.
func (e *Engine) logAlign(al *Alignment) {
	e.deps.Logger.Infow("cartesian teleop align poses",
		"aligned", al.Aligned,
		"follower_pose", poseMap(al.FollowerPose),
		"want", poseMap(al.LeaderTarget),
		"leader_inputs_deg", inputsDeg(al.LeaderInputs),
		"leader_actual", poseMap(al.LeaderActual),
		"mapped", poseMap(al.Mapped),
		"residual", residualMap(al.Residual),
		"position_error_mm", round1(al.PositionErrorMM),
		"orientation_error_deg", round1(al.OrientationErrorDeg),
	)
}

// maybeLogPose writes the run loop's pose line, at most once per poseLogInterval. leader is
// the leader's end pose, target the mapped follower target (residual applied), and
// follower_cmd the last target handed to the pipeline (the follower's commanded pose).
func (e *Engine) maybeLogPose(leader, target spatialmath.Pose) {
	now := e.deps.Now()
	last := e.poseLogAt.Load()
	if last != 0 && now.Sub(time.Unix(0, last)) < poseLogInterval {
		return
	}
	if !e.poseLogAt.CompareAndSwap(last, now.UnixNano()) {
		return
	}
	var cmd spatialmath.Pose
	if p := e.lastSentPose.Load(); p != nil {
		cmd = *p
	}
	e.deps.Logger.Infow("cartesian teleop pose",
		"leader", poseMap(leader),
		"target", poseMap(target),
		"follower_cmd", poseMap(cmd),
	)
}
