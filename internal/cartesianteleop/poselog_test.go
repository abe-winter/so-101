package cartesianteleop

import (
	"testing"

	"github.com/golang/geo/r3"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"google.golang.org/protobuf/types/known/structpb"
)

func TestAlignmentPosesMapIsStructpbCompatible(t *testing.T) {
	follower := spatialmath.NewPose(r3.Vector{X: 300.04, Y: -12.36, Z: 250}, &spatialmath.OrientationVectorDegrees{OZ: -1, Theta: 30})
	mapped := spatialmath.NewPose(r3.Vector{X: 295, Y: -10, Z: 248}, &spatialmath.OrientationVectorDegrees{OZ: -1, Theta: 25})
	al := &Alignment{
		Aligned:      true,
		FollowerPose: follower,
		LeaderTarget: spatialmath.NewPoseFromPoint(r3.Vector{X: 150, Y: 0, Z: 120}),
		LeaderInputs: []referenceframe.Input{0, 1.5707963},
		LeaderActual: spatialmath.NewPoseFromPoint(r3.Vector{X: 148, Y: 1, Z: 119}),
		Mapped:       mapped,
		Residual:     ResidualBetween(mapped, follower),
	}
	m := al.PosesMap()
	_, err := structpb.NewStruct(m)
	require.NoError(t, err)

	fp := m["follower_pose"].(map[string]interface{})
	assert.Equal(t, 300.0, fp["x"])
	assert.Equal(t, -12.4, fp["y"])
	assert.Equal(t, []interface{}{0.0, 90.0}, m["leader_inputs_deg"])
	res := m["residual"].(map[string]interface{})
	assert.Equal(t, 5.0, res["x"])
	assert.InDelta(t, 5.0, res["rotation_deg"].(float64), 0.11)
	for _, k := range []string{"leader_target", "leader_actual", "mapped"} {
		assert.Contains(t, m, k)
	}
}
