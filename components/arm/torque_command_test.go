package arm

import (
	"context"
	"testing"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/logging"

	"so_arm/internal/testfake"
)

func TestSetTorqueReportsPerServoReadback(t *testing.T) {
	ft := testfake.NewFakeTransport()
	ids := []int{1, 2, 3, 4, 5}
	for _, id := range ids {
		ft.SetRegister(id, feetech.RegTorqueEnable.Address, []byte{1})
	}
	s := &so101{
		controller:  testArmHandle(t, ft),
		armServoIDs: ids,
		logger:      logging.NewTestLogger(t),
	}

	res, err := s.DoCommand(context.Background(), map[string]interface{}{
		"command": "set_torque",
		"enable":  false,
	})
	require.NoError(t, err)
	assert.Equal(t, true, res["success"])
	assert.Equal(t, false, res["enable"])
	assert.Equal(t, true, res["all_match"])

	rows := res["servos"].([]interface{})
	require.Len(t, rows, len(ids))
	for i, item := range rows {
		row := item.(map[string]interface{})
		assert.Equal(t, ids[i], row["servo"])
		assert.Equal(t, false, row["enabled"])
		assert.Equal(t, true, row["matches"])
	}

	readback, err := s.DoCommand(context.Background(), map[string]interface{}{"command": "get_torque"})
	require.NoError(t, err)
	assert.Equal(t, true, readback["all_readable"])
	for _, item := range readback["servos"].([]interface{}) {
		assert.Equal(t, false, item.(map[string]interface{})["enabled"])
	}
}
