package calibration

import (
	"context"
	"path/filepath"
	"testing"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"

	"so_arm/internal/testfake"
)

func TestSaveCalibrationLeavesTorquelessBranchDisabled(t *testing.T) {
	ft := testfake.NewFakeTransport()
	cs := testRecordingSensor(t, ft)
	cs.state = StateCompleted
	cs.cfg.CalibrationFile = filepath.Join(t.TempDir(), "so101_calibration.json")
	for id, joint := range cs.joints {
		joint.HomingOffset = id
		joint.RangeMin = 500 + id*10
		joint.RangeMax = 3500 + id*10
		joint.IsCompleted = true
	}

	cs.mu.Lock()
	result, err := cs.saveCalibration(context.Background())
	cs.mu.Unlock()

	require.NoError(t, err)
	assert.Equal(t, true, result["success"])
	assert.Equal(t, false, result["torque_enabled"])
	assert.Contains(t, result["message"], "Torque remains disabled")
	assert.Equal(t, StateIdle, cs.state)
	for id := 1; id <= 5; id++ {
		assert.Zero(t, ft.WriteCount(id, feetech.RegGoalPosition.Address),
			"calibration save must not wake arm servo %d with a goal write", id)
	}
}
