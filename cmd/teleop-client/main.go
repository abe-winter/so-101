// Command teleop-client runs the Cartesian leader→follower mirror from a laptop, against live
// machines over gRPC, using exactly the code the devrel:so101:teleop-cartesian service runs
// on a machine. It exists to try a mapping, scale or hold mode without reconfiguring a robot.
//
//	go run ./cmd/teleop-client -config left.json -config right.json -leader-addr <machine>.viam.cloud
//
// Each -config is one leader/follower pair, all run in this one process over one connection.
// The file takes the same attributes as the service (see docs/teleop-cartesian.md). The
// follower may live on a second machine (-follower-addr); credentials come from flags or the
// VIAM_API_KEY_ID / VIAM_API_KEY environment variables, and apply to both machines unless
// -follower-key-id / -follower-key are given. A leader served by a local viam-server with no
// cloud config (so no TLS and no auth) is dialled with -leader-insecure.
package main

import (
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"os/signal"
	"path/filepath"
	"strings"
	"syscall"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/utils/rpc"

	engine "so_arm/internal/cartesianteleop"
)

func main() {
	if err := run(); err != nil {
		fmt.Fprintln(os.Stderr, "error:", err)
		os.Exit(1)
	}
}

func run() error {
	var configPaths stringList
	flag.Var(&configPaths, "config", "JSON file with the service attributes; repeat for several leader/follower pairs in one process (required)")
	var (
		leaderAddr       = flag.String("leader-addr", "", "address of the machine with the leader arm (required)")
		followerAddr     = flag.String("follower-addr", "", "address of the machine with the follower and its motion service (default: same as leader)")
		keyID            = flag.String("key-id", os.Getenv("VIAM_API_KEY_ID"), "API key id (or VIAM_API_KEY_ID)")
		key              = flag.String("key", os.Getenv("VIAM_API_KEY"), "API key (or VIAM_API_KEY)")
		followerKeyID    = flag.String("follower-key-id", "", "API key id for the follower machine (default: -key-id)")
		followerKey      = flag.String("follower-key", "", "API key for the follower machine (default: -key)")
		leaderInsecure   = flag.Bool("leader-insecure", false, "dial the leader machine without TLS or credentials (a local viam-server with no cloud config, e.g. localhost:8080)")
		followerInsecure = flag.Bool("follower-insecure", false, "dial the follower machine without TLS or credentials")
		statusEvery      = flag.Duration("status-every", 2*time.Second, "how often to print status; 0 disables")
		alignOnly        = flag.Bool("align-only", false, "align the leader to the follower, print the residual, and exit without mirroring")
		debug            = flag.Bool("debug", false, "debug logging")
	)
	flag.Parse()
	if len(configPaths) == 0 || *leaderAddr == "" {
		flag.Usage()
		return fmt.Errorf("-config and -leader-addr are required")
	}

	type side struct {
		name     string
		settings engine.Settings
		cfg      engine.Config
	}
	sides := make([]side, 0, len(configPaths))
	for _, path := range configPaths {
		raw, err := os.ReadFile(path)
		if err != nil {
			return err
		}
		var settings engine.Settings
		if err := json.Unmarshal(raw, &settings); err != nil {
			return fmt.Errorf("parse %s: %w", path, err)
		}
		cfg, err := settings.Resolve()
		if err != nil {
			return fmt.Errorf("%s: %w", path, err)
		}
		name := strings.TrimSuffix(filepath.Base(path), filepath.Ext(path))
		sides = append(sides, side{name: name, settings: settings, cfg: cfg})
	}

	logger := logging.NewLogger("teleop-client")
	if *debug {
		logger.SetLevel(logging.DEBUG)
	}
	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer cancel()

	leaderRobot, err := dial(ctx, *leaderAddr, *keyID, *key, *leaderInsecure, logger)
	if err != nil {
		return fmt.Errorf("connect to leader machine: %w", err)
	}
	defer leaderRobot.Close(context.Background())
	followerRobot := leaderRobot
	if *followerAddr != "" && *followerAddr != *leaderAddr {
		fkid, fk := *followerKeyID, *followerKey
		if fkid == "" {
			fkid, fk = *keyID, *key
		}
		if followerRobot, err = dial(ctx, *followerAddr, fkid, fk, *followerInsecure, logger); err != nil {
			return fmt.Errorf("connect to follower machine: %w", err)
		}
		defer followerRobot.Close(context.Background())
	}

	type running struct {
		name     string
		engine   *engine.Engine
		pipeline *engine.Pipeline
	}
	var engines []running
	stopAll := func() error {
		var firstErr error
		for _, r := range engines {
			stopCtx, stopCancel := context.WithTimeout(context.Background(), 10*time.Second)
			if err := r.engine.Stop(stopCtx); err != nil && firstErr == nil {
				firstErr = fmt.Errorf("%s: %w", r.name, err)
			}
			stopCancel()
		}
		return firstErr
	}

	for _, sd := range sides {
		leader, err := arm.FromRobot(leaderRobot, sd.settings.LeaderArm)
		if err != nil {
			return err
		}
		motionSvc, err := motion.FromRobot(followerRobot, sd.settings.MotionName())
		if err != nil {
			return err
		}
		pipeline := engine.NewPipeline(motionSvc, sd.settings.MotionName(), sd.settings.FollowerComponent)
		deps := engine.Deps{Leader: leader, Pipeline: pipeline, Logger: logger.Sublogger(sd.name)}
		if sd.settings.LeaderGripper != "" {
			g, err := gripper.FromRobot(leaderRobot, sd.settings.LeaderGripper)
			if err != nil {
				return err
			}
			deps.LeaderGripper = g
		}
		if sd.settings.FollowerGripper != "" {
			g, err := gripper.FromRobot(followerRobot, sd.settings.FollowerGripper)
			if err != nil {
				return err
			}
			deps.FollowerGripper = g
		}
		e := engine.New(deps, sd.cfg)
		if err := e.Start(ctx); err != nil {
			// One side failing (an align the leader cannot reach, say) should not take the
			// other side down with it; report it and carry on with whatever started.
			fmt.Printf("[%s] NOT STARTED: %v\n", sd.name, err)
			continue
		}
		engines = append(engines, running{name: sd.name, engine: e, pipeline: pipeline})
		al := e.Align()
		if al.Aligned {
			fmt.Printf("[%s] aligned: leader within %.1f mm / %.1f deg of the follower; leader joints %v\n",
				sd.name, al.PositionErrorMM, al.OrientationErrorDeg, al.LeaderInputs)
		} else {
			fmt.Printf("[%s] captured: mirroring deltas from the leader's pose (%.0f mm / %.0f deg offset absorbed)\n",
				sd.name, al.PositionErrorMM, al.OrientationErrorDeg)
		}
	}
	if len(engines) == 0 {
		return fmt.Errorf("no side started")
	}
	if *alignOnly {
		return stopAll()
	}
	fmt.Println("mirroring; Ctrl-C to stop")

	var ticker *time.Ticker
	var tick <-chan time.Time
	if *statusEvery > 0 {
		ticker = time.NewTicker(*statusEvery)
		defer ticker.Stop()
		tick = ticker.C
	}
	for {
		select {
		case <-ctx.Done():
			return stopAll()
		case <-tick:
			allStopped := true
			for _, r := range engines {
				st := r.engine.Status()
				// The pipeline's own view: a goal the planner cannot reach, or a collision
				// with a world obstacle, shows up ONLY here -- the client's sends succeed.
				pipelineErr, planMs := "", 0.0
				if ps, err := r.pipeline.Status(ctx); err == nil {
					pipelineErr, _ = ps["error"].(string)
					planMs, _ = ps["last_plan_ms"].(float64)
				} else {
					pipelineErr = "status: " + err.Error()
				}
				fmt.Printf("[%s] running=%v cycles=%d sends=%d dropped=%d deadzoned=%d rejected=%d rpc=%.1fms plan=%.0fms gripper=%.0f%% err=%q pipeline=%q\n",
					r.name, st.Running, st.Cycles, st.Sends, st.Dropped, st.Deadzoned, st.Rejected, st.LastRPCMs, planMs, st.GripperPct, st.LastError, pipelineErr)
				if st.Running {
					allStopped = false
				} else {
					fmt.Printf("[%s] mirror loop stopped: %s (leader left held; Ctrl-C releases it)\n", r.name, st.LastError)
				}
			}
			if allStopped {
				return fmt.Errorf("every mirror loop stopped (stop: %v)", stopAll())
			}
		}
	}
}

// stringList is a repeatable string flag.
type stringList []string

func (l *stringList) String() string     { return strings.Join(*l, ",") }
func (l *stringList) Set(v string) error { *l = append(*l, v); return nil }

func dial(ctx context.Context, addr, keyID, key string, insecure bool, logger logging.Logger) (*client.RobotClient, error) {
	opts := []client.RobotClientOption{client.WithRefreshEvery(0)}
	switch {
	case insecure:
		opts = append(opts, client.WithDialOptions(rpc.WithInsecure()))
	case keyID != "":
		opts = append(opts, client.WithDialOptions(rpc.WithEntityCredentials(keyID, rpc.Credentials{
			Type:    rpc.CredentialsTypeAPIKey,
			Payload: key,
		})))
	}
	return client.New(ctx, addr, logger, opts...)
}
