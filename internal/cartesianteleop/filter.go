package cartesianteleop

import (
	"math"
	"time"

	"go.viam.com/rdk/spatialmath"
)

// FilterConfig conditions the stream of target poses before they are sent. Every rate is per
// unit time rather than per frame so the feel does not change with rate_hz; this is the mobile
// app's time-normalised form of the vive module's filter.
type FilterConfig struct {
	// SmoothTau is the EMA time constant for position and the SLERP time constant for
	// orientation. Zero disables smoothing.
	SmoothTau time.Duration
	// OutlierPosMMPerSec / OutlierRotDegPerSec reject a sample that would imply a faster
	// motion than this. Zero disables that check.
	OutlierPosMMPerSec  float64
	OutlierRotDegPerSec float64
	// OutlierAcceptAfter is how many consecutive rejections before the jump is accepted as
	// real and the filter re-seeded. Default 5.
	OutlierAcceptAfter int
	// DeadzonePosMM / DeadzoneRotDeg skip a send that differs from the last sent pose by less
	// than this. Zero disables the deadzone.
	DeadzonePosMM  float64
	DeadzoneRotDeg float64
}

// Filter is the per-session state behind FilterConfig. The zero value is unseeded; the first
// Step seeds it and always passes.
type Filter struct {
	cfg       FilterConfig
	prev      spatialmath.Pose
	prevTime  time.Time
	lastSent  spatialmath.Pose
	streak    int
	Rejected  uint64 // outlier rejections, for status
	Deadzoned uint64 // deadzone skips, for status
}

// NewFilter returns a Filter for cfg.
func NewFilter(cfg FilterConfig) *Filter {
	if cfg.OutlierAcceptAfter <= 0 {
		cfg.OutlierAcceptAfter = 5
	}
	return &Filter{cfg: cfg}
}

// Reset forgets the session state so the next Step seeds again.
func (f *Filter) Reset() {
	f.prev, f.lastSent, f.streak = nil, nil, 0
	f.prevTime = time.Time{}
}

// Step conditions raw, sampled at now. It returns the pose to send and whether to send it.
func (f *Filter) Step(now time.Time, raw spatialmath.Pose) (spatialmath.Pose, bool) {
	if f.prev == nil {
		f.prev, f.prevTime, f.lastSent = raw, now, raw
		return raw, true
	}
	dt := now.Sub(f.prevTime).Seconds()
	// Clamp like the mobile app does: a stall must not become an infinite speed, a burst must
	// not become an instant one.
	dt = math.Min(math.Max(dt, 1.0/120), 0.25)

	if f.cfg.OutlierPosMMPerSec > 0 || f.cfg.OutlierRotDegPerSec > 0 {
		posRate := raw.Point().Sub(f.prev.Point()).Norm() / dt
		rotRate := angleBetweenDeg(f.prev, raw) / dt
		if (f.cfg.OutlierPosMMPerSec > 0 && posRate > f.cfg.OutlierPosMMPerSec) ||
			(f.cfg.OutlierRotDegPerSec > 0 && rotRate > f.cfg.OutlierRotDegPerSec) {
			f.streak++
			if f.streak < f.cfg.OutlierAcceptAfter {
				f.Rejected++
				return nil, false
			}
			// Accepted as real: re-seed so the EMA does not drag it out.
			f.prev, f.prevTime, f.lastSent, f.streak = raw, now, raw, 0
			return raw, true
		}
	}
	f.streak = 0

	target := raw
	if f.cfg.SmoothTau > 0 {
		alpha := 1 - math.Exp(-dt/f.cfg.SmoothTau.Seconds())
		target = spatialmath.Interpolate(f.prev, raw, alpha)
	}
	f.prev, f.prevTime = target, now

	if f.cfg.DeadzonePosMM > 0 || f.cfg.DeadzoneRotDeg > 0 {
		dPos := target.Point().Sub(f.lastSent.Point()).Norm()
		dRot := angleBetweenDeg(f.lastSent, target)
		// A zero threshold means "ignore that axis", not "never inside".
		inPos := f.cfg.DeadzonePosMM == 0 || dPos < f.cfg.DeadzonePosMM
		inRot := f.cfg.DeadzoneRotDeg == 0 || dRot < f.cfg.DeadzoneRotDeg
		if inPos && inRot {
			f.Deadzoned++
			return nil, false
		}
	}
	f.lastSent = target
	return target, true
}

func angleBetweenDeg(a, b spatialmath.Pose) float64 {
	return orientationError(a.Orientation(), b.Orientation()).Norm() * 180 / math.Pi
}
