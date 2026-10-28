package calibration

import (
	"context"
	"path/filepath"
	"testing"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"

	"so_arm/internal/controller"
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

func TestGripperOnlySavePreservesExistingArmCalibration(t *testing.T) {
	path := filepath.Join(t.TempDir(), "so101_calibration.json")
	existing := controller.DefaultSO101FullCalibration
	shoulder := *existing.ShoulderPan
	shoulder.RangeMin = 777
	existing.ShoulderPan = &shoulder
	require.NoError(t, controller.SaveFullCalibrationToFile(path, existing))

	cs := testRecordingSensor(t, testfake.NewFakeTransport())
	cs.state = StateCompleted
	cs.cfg.CalibrationFile = path
	for id := range cs.joints {
		if id != 6 {
			delete(cs.joints, id)
		}
	}
	gripper := cs.joints[6]
	gripper.HomingOffset = 123
	gripper.RangeMin = 1400
	gripper.RangeMax = 2500
	gripper.IsCompleted = true

	cs.mu.Lock()
	_, err := cs.saveCalibration(context.Background())
	cs.mu.Unlock()
	require.NoError(t, err)

	saved, err := controller.LoadFullCalibrationFromFile(path, cs.logger)
	require.NoError(t, err)
	assert.Equal(t, 777, saved.ShoulderPan.RangeMin)
	assert.Equal(t, 1400, saved.Gripper.RangeMin)
	assert.Equal(t, 2500, saved.Gripper.RangeMax)
}
