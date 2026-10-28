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

On **stop**, the follower is removed from the pipeline and holds where it is. Manual mode exits
and leaves the leader holding its current pose. A `torque_off` leader deliberately stays limp:
teleop never enables torque for an arm used only as an input device, including during service
replacement and shutdown.

With `wrist_clutch: true`, turning the SO-101 leader's wrist-roll joint (joint 5) to 60°
or further pauses pose targets. The follower holds its last target while the leader is
repositioned. Returning the wrist within 45° rebases the mapping and resumes without a jump.
The fixed 60°/45° hysteresis is intentional for this demo version. Gripper mirroring continues
while pose mirroring is clutched. The option defaults off.

## The mapping

```
follower_pose = origin ∘ scale(leader_pose ∘ leader_tool_offset) ∘ tool_orientation
```

- `leader_pose` is the leader arm's `EndPosition`, in its own base frame.
- `leader_tool_offset_mm` moves the mirrored point from the leader's `EndPosition` frame out
  to its physical tip, in the tool frame, **before** scaling. On an SO-101 `EndPosition` is
  the wrist `tool` frame, about 96 mm behind the far end of the leader handle, while
  `follower_component` is normally a gripper frame at the jaw tip. Without the offset the two
  arms are mirrored at different points along the tool axis: translations still match, but a
  leader rotation pivots the follower about a point `scale × 96 mm` off its tip, so a handle
  tip held on the table maps to a follower tip hovering `scale × 96 mm` above it. With it, tip
  maps to tip at every scale. Measured from the leader gripper meshes; `{"z": 96}` for the
  stock SO-101 leader handle.
- `scale` multiplies the leader's (offset) translation about its base, so a desktop arm can
  span a large follower workspace.
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
  "leader_tool_offset_mm": { "x": 0, "y": 0, "z": 96 },
  "origin_translation_mm": { "x": 0, "y": 0, "z": 0 },
  "origin_orientation": { "type": "ov_degrees", "value": { "x": 0, "y": 0, "z": 1, "th": 0 } },
  "rotation_enabled": true,
  "leader_hold": "manual"
}
```

The follower arm, its gripper and the motion service must all be in this viam-server's
frame system: either on the same machine, or on a machine configured as a remote.

### Several leader/follower pairs in one service

One service drives every pair on a machine, like the vive module's `hands` list. Top-level
attributes become defaults shared by every pair; each entry of `pairs` has a `name` and
overrides whatever it needs, normally just the arm names and the origin. A key set in a pair
wins; a key it omits is inherited. Without `pairs`, the top-level attributes are the one pair,
named `main`.

```json
{
  "scale": 2.5,
  "leader_tool_offset_mm": { "x": 0, "y": 0, "z": 96 },
  "align_on_start": true,
  "leader_hold": "manual",
  "gripper_in_max_pct": 70,
  "pairs": [
    {
      "name": "left",
      "leader_arm": "left-leader-arm", "leader_gripper": "left-leader-gripper",
      "follower_component": "left-gripper", "follower_gripper": "left-gripper",
      "origin_translation_mm": { "x": 0, "y": 1003, "z": 0 }
    },
    {
      "name": "right",
      "leader_arm": "right-leader-arm", "leader_gripper": "right-leader-gripper",
      "follower_component": "right-gripper", "follower_gripper": "right-gripper"
    }
  ]
}
```

Every pair registers with the same motion service teleop pipeline, which plans all registered
followers in **one joint plan**. So a target the planner rejects on one side (a collision with
the table, a wall, or the other arm) stalls the other side too. That is a property of the
pipeline, and running the pairs as separate services does not change it.

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
| `leader_tool_offset_mm` | `{x,y,z}` | Optional | Leader tip in its `EndPosition` (tool) frame, applied before `scale`. Default zero (mirror the reported frame itself). Use `{"z": 96}` for the stock SO-101 leader handle. |
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
| `wrist_clutch` | bool | Optional | Enable the demo SO-101 joint-5 clutch: pause at 60°, rebase and resume within 45°. Default `false`. |
| `follower_gripper_mode` | string | Optional | `xarm` (default; DoCommand `{"set": 0..850}` on a `viam:ufactory` gripper), `open_close` (gripper API with hysteresis), or `off`. |
| `gripper_invert` | bool | Optional | Flip the leader percentage. The SO-101 gripper reports `0` closed / `~95` open, and the leader handle's trigger reads near `0` at rest, so without inversion the follower sits closed and squeezing opens it. Set `true` for squeeze-to-close. |
| `gripper_range` | number | Optional | Steps at 100 % in `xarm` mode. Default `850`. |
| `gripper_in_min_pct` / `gripper_in_max_pct` | number | Optional | Rescale the leader's raw reading before inversion: values at or below `min` map to 0 %, at or above `max` to 100 %. Default `0` / `100`. Lowering `max` (the examples use 70) makes full opening need less trigger travel. |
| `gripper_lock` / `gripper_lock_ms` | bool / number | Optional | Freeze the pair while its follower gripper moves: after every follower-gripper command, no pose targets go to the follower and the leader is held where it is (`manual_hold` at `feedback_torque_limit`, default 250; needs `leader_hold: manual` to hold the leader) for `gripper_lock_ms` (default `1500`, max `4500`), extended by each further gripper command. For followers whose controller pauses arm motion during a gripper move, such as the Fairino FR5, so the leader cannot run ahead of a follower that is standing still. Default off. |
| `gripper_deadband_pct` | number | Optional | Leader change needed before a new gripper command. Default `3`. Gripper writes are asynchronous and latest-wins, because the xArm gripper's `set` blocks until the jaw arrives. |
| `blocked_feedback` | string | Optional | `spring` pulls the leader back when its follower cannot follow (see [Blocked-follower feedback](#blocked-follower-feedback)); `off` (default) does not. Needs `leader_hold: manual`. |
| `feedback_torque_limit` | int | Optional | Leader torque cap (0-1000) while it pulls back. Default `250`; manual mode's own cap is `50`. |
| `feedback_max_speed_deg_per_sec` | number | Optional | How fast the leader is pulled back. Default `45`. |
| `feedback_lag_mm` | number | Optional | How far the follower must sit from its target, without moving, to count as stalled. Default `40`. |
| `follower_arm` | string | Optional | The follower ARM's component name. Lets a planner error that names the arm's own geometry (not just the gripper's) be attributed to this pair at once. |
| `pairs` | list | Optional | Leader/follower pairs, each a `name` plus any of the attributes above, overlaid on the top-level defaults. See [Several pairs](#several-leaderfollower-pairs-in-one-service). Default: the top-level attributes are one pair named `main`. |
| `auto_start` | bool | Optional | Start every pair on configure. Default `false`. |

### `leader_hold`

| Value | Leader after alignment | When |
| --- | --- | --- |
| `manual` | Torque on at reduced gain with a torque cap; holds against gravity, follows the hand, stays where left. | Default. Requires an SO-101 leader. |
| `stiff` | Full stiffness; the driver cannot move it. | Checking alignment without a hand on the arm. |
| `torque_off` | Servos unpowered. The arm **falls** unless held, and the fall is mirrored. | Only with a hand already on the leader. |

## Blocked-follower feedback

With `"blocked_feedback": "spring"`, when a pair's follower cannot follow its leader, the
leader pushes back toward the last pose the follower actually reached. It is the leader-arm
counterpart of the vive controller's and the phone's haptic pulse. The follower is judged
blocked when either:

- the motion pipeline's planner error names this pair's follower (`follower_component`, or
  `follower_arm` when set): a collision with the table, a wall, or the other arm; or
- the follower has sat more than `feedback_lag_mm` from its target, without moving, for
  300 ms. This catches goals the planner calls unreachable (which name no component, so
  cannot be attributed by name) and anything else that stops the follower short.

The pull-back target is the recorded leader pose whose mapped target lay nearest the
follower's actual position, not simply the most recent one: a failed plan takes up to a
second to report, and by then the latest leader poses are already deep in the unreachable
region. The leader is driven there at no more than `feedback_max_speed_deg_per_sec` with its
torque cap raised to `feedback_torque_limit`, so it pushes back firmly without snapping into
the hand. The pull is refreshed every 100 ms and expires on its own within 300 ms if
nothing refreshes it. When the follower is following again, the leader is released, and
its goals re-anchor where the hand has it.

Because the pull eases the leader back to where the follower is, the pipeline gets a target
it can reach, its error clears, and the leader lets go. Pushing through the pull simply keeps
it on. `status` reports `blocked` and `blocked_reason` per pair.

## DoCommand

```json
{ "command": "start" }
{ "command": "stop" }
{ "command": "status" }
{ "command": "start", "side": "left" }
```

Each command acts on every pair, or on one pair when given `"side": "<pair name>"`. `start`
attempts every pair even if one fails (an align the leader cannot reach, say), and returns the
failure as an error; `status` then shows which pairs are running. `stop` likewise releases
every leader it can.

Every command returns `{"running": <any pair running>, "pairs": {"<name>": {...}}}`. Each
pair reports `running`, `cycles`, `sends`, `dropped` (a send was due while one was in
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
