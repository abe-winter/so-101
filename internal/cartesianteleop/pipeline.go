package cartesianteleop

import (
	"context"
	"fmt"
	"strings"
	"sync/atomic"

	commonpb "go.viam.com/api/common/v1"
	pb "go.viam.com/api/service/motion/v1"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"google.golang.org/protobuf/encoding/protojson"
)

// Motion is the slice of motion.Service the pipeline needs. motion.Service satisfies it, and
// so does a gRPC client to a remote machine's motion service.
type Motion interface {
	GetPose(ctx context.Context, componentName, destinationFrame string,
		supplementalTransforms []*referenceframe.LinkInFrame, extra map[string]interface{},
	) (*referenceframe.PoseInFrame, error)
	DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error)
}

// WorldFrame is the frame every pipeline pose is expressed in.
const WorldFrame = "world"

// Pipeline is a client for the builtin motion service's teleop DoCommands
// (rdk services/motion/builtin: teleop_start, teleop_move, teleop_stop, teleop_status).
// Payloads are protojson of the real request messages rather than hand-formatted strings, so
// a field the server adds or renames fails loudly here rather than being silently ignored.
type Pipeline struct {
	motion     Motion
	motionName string // the motion service's resource name, "builtin" for the default
	component  string // the frame-system component the pipeline positions (usually the gripper)
	seq        atomic.Int64
}

// NewPipeline returns a Pipeline that positions component through motion.
func NewPipeline(m Motion, motionName, component string) *Pipeline {
	return &Pipeline{motion: m, motionName: motionName, component: component}
}

// Component is the frame-system name the pipeline positions.
func (p *Pipeline) Component() string { return p.component }

// CurrentPose reads the component's pose in the world frame.
func (p *Pipeline) CurrentPose(ctx context.Context) (spatialmath.Pose, error) {
	pif, err := p.motion.GetPose(ctx, p.component, WorldFrame, nil, nil)
	if err != nil {
		return nil, fmt.Errorf("get pose of %q: %w", p.component, err)
	}
	return pif.Pose(), nil
}

// Start registers the component with the pipeline, with destination as its initial target.
// Pass the component's current pose so registration itself causes no motion.
func (p *Pipeline) Start(ctx context.Context, destination spatialmath.Pose) error {
	req := &pb.MoveRequest{
		Name:          p.motionName,
		ComponentName: p.component,
		Destination:   referenceframe.PoseInFrameToProtobuf(referenceframe.NewPoseInFrame(WorldFrame, destination)),
	}
	payload, err := protojson.Marshal(req)
	if err != nil {
		return err
	}
	_, err = p.motion.DoCommand(ctx, map[string]interface{}{"teleop_start": string(payload)})
	if err != nil {
		return fmt.Errorf("teleop_start: %w", err)
	}
	return nil
}

// Move replaces the component's target. The pipeline keeps only the latest pose, so a caller
// should keep at most one Move in flight and send its freshest pose when the previous returns.
func (p *Pipeline) Move(ctx context.Context, target spatialmath.Pose) error {
	pif := referenceframe.PoseInFrameToProtobuf(referenceframe.NewPoseInFrame(WorldFrame, target))
	payload, err := protojson.Marshal((*commonpb.PoseInFrame)(pif))
	if err != nil {
		return err
	}
	_, err = p.motion.DoCommand(ctx, map[string]interface{}{
		"teleop_move":    string(payload),
		"component_name": p.component,
		"seq":            float64(p.seq.Add(1)),
	})
	if err != nil {
		return fmt.Errorf("teleop_move: %w", err)
	}
	return nil
}

// Stop removes the component from the pipeline. The follower holds where it is.
func (p *Pipeline) Stop(ctx context.Context) error {
	_, err := p.motion.DoCommand(ctx, map[string]interface{}{
		"teleop_stop":    true,
		"component_name": p.component,
	})
	if err != nil {
		return fmt.Errorf("teleop_stop: %w", err)
	}
	return nil
}

// Status returns the pipeline's teleop_status map (running, plan/exec timings, error).
func (p *Pipeline) Status(ctx context.Context) (map[string]interface{}, error) {
	resp, err := p.motion.DoCommand(ctx, map[string]interface{}{"teleop_status": true})
	if err != nil {
		return nil, fmt.Errorf("teleop_status: %w", err)
	}
	status, _ := resp["teleop_status"].(map[string]interface{})
	return status, nil
}

// IsSessionLost reports whether err means the server-side pipeline is gone, which no retry of
// Move can fix: the session must be restarted.
func IsSessionLost(err error) bool {
	return err != nil && strings.Contains(err.Error(), "not running")
}
