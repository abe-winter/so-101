[← Part of the SO-101 module](../README.md)

# Model devrel:so101:teleop-cartesian

A generic service that mirrors a hand-guided **leader** SO-101's end-effector pose onto a
**follower** arm of different kinematics, such as a uFactory xArm. Where
[`devrel:so101:teleop`](teleop.md) copies joint positions and so needs two identical arms,
this model works in Cartesian space: the leader's tool pose is mapped into the follower's
world and handed to viam-server's builtin motion service **teleop pipeline** (the
`teleop_start` / `teleop_move` / `teleop_stop` DoCommands), which plans IK and streams joints
to the follower under its own velocity and acceleration limits. The leader is read through
the standard arm API; nothing here is specific to what the follower is.

The same code runs as a standalone client, `cmd/teleop-client`, for trying a mapping from a
laptop before putting it in a machine config.

## What happens on start

Nothing moves. By default the service **captures** both arms where they are: the follower's
current pose and the leader's current pose become the session's reference, the gap between
them is absorbed as a constant offset, and from then on the leader's motion is mirrored as
scaled deltas from the follower's pose. The leader is handed to the driver in the configured
`leader_hold`. There is no clutch and no IK on this side.

### Optional: `align_on_start`

With `"align_on_start": true` the service instead **drives the leader to the follower's pose**
before mirroring, so the two arms correspond absolutely:

1. The follower's current pose is read from the motion service (`GetPose` of
   `follower_component` in `world`).
2. That pose is mapped back into the leader's base frame (the inverse of the mapping below)
   and solved for leader joints by a best-effort IK: a 5-DoF arm usually cannot reach a
   6-DoF pose exactly, so the solver returns the closest configuration and reports the
   residual (`align_position_error_mm`, `align_orientation_error_deg` in `status`).
3. The leader is driven there with a normal profiled move, blocking.
4. The leader is handed to the driver according to `leader_hold` (default: the arm's manual
   mode, torque on at reduced gain, so it holds itself against gravity but yields to a hand).
5. Unless `compensate_align_residual` is off, the leftover gap from step 2 is captured as a
   constant correction, so the first target sent to the follower is **exactly where the
   follower already is**. The follower does not move on start.
6. The follower is registered with the pipeline at its current pose, and mirroring begins.

Align is best effort: a leader that cannot reach the mapped follower pose goes as close as it
can and the gap is absorbed into the offset (and logged when it is large). Set
`align_max_position_error_mm` to refuse instead. A joint that stops short of the IK target (the arm clamps
to its *calibrated* range, which the model does not know) is accepted and absorbed into the
offset; only an arm that does not move at all is refused.

On **stop**, the follower is removed from the pipeline and holds where it is; the leader is
released from its hold (manual mode exits and the arm holds its current pose; in
`torque_off` mode the goal is seeded at the present position *before* torque is re-enabled,
so the arm does not snap).

## The mapping

```
follower_pose = origin ∘ scale(leader_pose) ∘ tool_orientation
```

- `leader_pose` is the leader arm's `EndPosition`, in its own base frame.
- `scale` multiplies the leader's translation about its base, so a desktop arm can span a
  large follower workspace.
- `origin_translation_mm` / `origin_orientation` place the leader's base in the follower's
  world frame.
- `tool_orientation` is a fixed rotation in the tool frame that reconciles the two arms'
  tool-frame conventions.

Orientation mirroring is relative to this mapping and limited by the leader: a 5-DoF wrist
cannot express every rotation, and those it cannot are simply never requested. Set
`rotation_enabled: false` to hold the follower's starting orientation and mirror position
only, which many drivers prefer.

## Configuration

```json
{
  "leader_arm": "leader-arm",
  "leader_gripper": "leader-gripper",
  "follower_component": "xarm-gripper",
  "follower_gripper": "xarm-gripper",
  "scale": 2.0,
  "origin_translation_mm": { "x": 0, "y": 0, "z": 0 },
  "origin_orientation": { "type": "ov_degrees", "value": { "x": 0, "y": 0, "z": 1, "th": 0 } },
  "rotation_enabled": true,
  "leader_hold": "manual"
}
```

The follower arm, its gripper and the motion service must all be in this viam-server's
frame system: either on the same machine, or on a machine configured as a remote.

### Attributes

| Name | Type | Inclusion | Description |
| --- | --- | --- | --- |
| `leader_arm` | string | **Required** | Leader arm to read (any arm with `EndPosition`; an SO-101 for `leader_hold: manual`). |
| `follower_component` | string | **Required** | Frame-system name the pipeline positions, normally the follower gripper so its TCP tracks the leader. |
| `motion_service` | string | Optional | Motion service name. Default `builtin`. |
| `leader_gripper` / `follower_gripper` | string | Optional | Set both to mirror the gripper. |
| `scale` | number | Optional | Translation scale, leader → follower. Default `1`. |
| `origin_translation_mm` | `{x,y,z}` | Optional | Leader base position in follower world. Default origin. |
| `origin_orientation` | orientation | Optional | Leader base orientation in follower world (frame-config syntax, e.g. `ov_degrees`). Default identity. |
| `tool_orientation` | orientation | Optional | Fixed tool-frame rotation. Default identity. |
| `rotation_enabled` | bool | Optional | Mirror orientation as well as position. Default `true`. |
| `rate_hz` | number | Optional | Leader sampling rate. Default `60`. Sends are at most one in flight ("latest wins"), so the send rate is bounded by the RPC round trip. |
| `smooth_tau_ms` | number | Optional | EMA/SLERP time constant. Default `100`; `0` disables. |
| `pos_deadzone_mm` / `rot_deadzone_deg` | number | Optional | Skip sends that move less than this. Default `1` / `1`. |
| `outlier_pos_mm_per_s` / `outlier_rot_deg_per_s` | number | Optional | Reject samples implying faster motion; accepted after 5 in a row. Default `4500` / `1800`. |
| `move_timeout_ms` | number | Optional | Per-send RPC timeout. Default `500`. |
| `max_consecutive_errors` | int | Optional | Cycle failures before the loop stops. Default `10`. |
| `align_on_start` | bool | Optional | Drive the leader to the follower's pose before mirroring (IK + a blocking move). Default `false`: capture both arms where they are. |
| `leader_hold` | string | Optional | `manual` (default), `stiff`, or `torque_off`. See below. |
| `manual_mode` | object | Optional | Overrides passed to the leader's `enter_manual_mode` (`p_gain`, `torque_limit`, `pos_deadband_deg`, `loop_hz`). |
| `compensate_align_residual` | bool | Optional | Capture the alignment gap so the follower does not move on start. Default `true`. |
| `align_orientation_weight_mm_per_rad` | number | Optional | How the align IK trades orientation for position: 1 rad of orientation error costs this many mm. Default `50`. |
| `align_max_position_error_mm` | number | Optional | With `align_on_start`, refuse to start if the leader ends up further than this from the mapped follower pose. Default `0`: best effort, the gap is absorbed and logged. |
| `follower_gripper_mode` | string | Optional | `xarm` (default; DoCommand `{"set": 0..850}` on a `viam:ufactory` gripper), `open_close` (gripper API with hysteresis), or `off`. |
| `gripper_invert` | bool | Optional | Flip the leader percentage. The SO-101 gripper reports `0` closed / `~95` open, and the leader handle's trigger reads near `0` at rest, so without inversion the follower sits closed and squeezing opens it. Set `true` for squeeze-to-close. |
| `gripper_range` | number | Optional | Steps at 100 % in `xarm` mode. Default `850`. |
| `gripper_in_min_pct` / `gripper_in_max_pct` | number | Optional | Rescale the leader's raw reading before inversion: values at or below `min` map to 0 %, at or above `max` to 100 %. Default `0` / `100`. Lowering `max` (the examples use 70) makes full opening need less trigger travel. |
| `gripper_deadband_pct` | number | Optional | Leader change needed before a new gripper command. Default `3`. Gripper writes are asynchronous and latest-wins, because the xArm gripper's `set` blocks until the jaw arrives. |
| `auto_start` | bool | Optional | Start on configure. Default `false`. |

### `leader_hold`

| Value | Leader after alignment | When |
| --- | --- | --- |
| `manual` | Torque on at reduced gain with a torque cap; holds against gravity, follows the hand, stays where left. | Default. Requires an SO-101 leader. |
| `stiff` | Full stiffness; the driver cannot move it. | Checking alignment without a hand on the arm. |
| `torque_off` | Servos unpowered. The arm **falls** unless held, and the fall is mirrored. | Only with a hand already on the leader. |

## DoCommand

```json
{ "command": "start" }
{ "command": "stop" }
{ "command": "status" }
```

`status` returns `running`, `cycles`, `sends`, `dropped` (a send was due while one was in
flight), `deadzoned`, `rejected`, `consecutive_errors`, `last_error`, `last_rpc_ms`,
`rate_hz`, `align_position_error_mm`, `align_orientation_error_deg`, `align_leader_joints`
and `gripper_pct`.

If the loop stops itself (error threshold, or the pipeline reports it is no longer running)
the leader is deliberately **left held**, because a hand may be on it; `stop` releases it, and
a `start` on a session that ended that way releases it first.

## Running as a client

```sh
VIAM_API_KEY_ID=... VIAM_API_KEY=... \
go run ./cmd/teleop-client -config teleop.json -leader-addr so101-station.viam.cloud \
  [-follower-addr xarm-cell.viam.cloud] [-align-only]
```

`teleop.json` holds the attribute block above. `-align-only` aligns the leader, prints the
residual, and exits, which is the quickest way to tune `scale` and `origin_*`. The leader can
be served by a cloud-less local viam-server on the laptop (`-leader-addr localhost:8080
-leader-insecure`); `examples/cartesian-teleop/` has a complete local config, settings file
and the two scripts for that setup. Expect tens of
sends per second from a laptop (two network RPCs per cycle); on the machine itself the
pipeline's round trip is the limit.

## Requirements

- The follower machine's viam-server needs the motion teleop DoCommands (rdk ≥ v0.119.0) and
  benefits from the streamed teleop executor (newer rdk plus a follower module that implements
  `MoveThroughJointPositionsStreamed`; the uFactory xArm module does).
- The motion service's teleop joint velocity/acceleration limits are the hard bound on how
  fast the follower moves. Tune them on the follower before the first session, with nobody
  in reach.
