package arm

import (
	"context"
	"fmt"
	"time"
)

// torqueWatchInterval is deliberately slower than motion polling but fast enough that an
// unexpected re-enable cannot leave a hand-driven leader stiff for long. The state read is one
// sync transaction for all arm servos; a write happens only after a non-zero register is seen.
const torqueWatchInterval = 250 * time.Millisecond

// enforceTorqueDisabled observes the live registers and repairs a torqueless-policy violation.
// It returns the IDs found enabled so the caller can log the false-to-true transition.
func (s *so101) enforceTorqueDisabled(ctx context.Context) ([]int, error) {
	enabled, err := s.controller.EnabledTorqueServos(ctx, s.armServoIDs)
	if err != nil || len(enabled) == 0 {
		return enabled, err
	}
	if err := s.controller.SetTorqueEnable(ctx, false); err != nil {
		return enabled, fmt.Errorf("disable re-enabled servos %v: %w", enabled, err)
	}
	return enabled, nil
}

func (s *so101) runTorqueWatchdog() {
	defer s.backgroundWG.Done()
	ticker := time.NewTicker(torqueWatchInterval)
	defer ticker.Stop()

	lastErr := ""
	check := func() {
		enabled, err := s.enforceTorqueDisabled(s.cancelCtx)
		if err != nil {
			if msg := err.Error(); msg != lastErr {
				s.logger.Warnf("Torque watchdog read/repair failed: %v", err)
				lastErr = msg
			}
			return
		}
		if lastErr != "" {
			s.logger.Info("Torque watchdog recovered after a bus error")
			lastErr = ""
		}
		if len(enabled) != 0 {
			s.logger.Warnf("Torque watchdog detected enabled servos %v and forced the bus off", enabled)
		}
	}

	check()
	for {
		select {
		case <-s.cancelCtx.Done():
			return
		case <-ticker.C:
			check()
		}
	}
}
