# PX4 SITL + AirSim + ROS 2 Offboard: Required Parameters

Paste these into the running `pxh>` shell (started via `make px4_sitl none_iris`).
After applying, run `param save` once to persist across restarts.

## Time sync: why we tolerate the drift with a large COM_OF_LOSS_T

The offboard "signal lost" rejections are a **clock-domain mismatch**: the ROS 2 node stamps messages with the WSL wall clock while PX4 samples freshness against AirSim's simulated clock (`hrt_absolute_time`). AirSim's Unreal loop runs far below real time, so PX4's sim clock lags the wall clock by a large, ever-growing amount. Measured live: `timesync_status.observed_offset` was about **-141 s** shortly after start and keeps growing.

We tried the clean fix - `use_sim_time=true` on the node plus launching the agent with `--timesync` - but it does not work with this stack: **MicroXRCE-DDS-Agent v2.4.3 uses `--timesync` only to adjust PX4's own clock; it does not republish a ROS `/clock` topic.** Verified directly: `ros2 topic info /clock` shows **Publisher count: 0**. With `use_sim_time=true` and no `/clock`, the node clock stays at 0 and offboard setpoints never get a valid timestamp, so the node deadlocks (stuck at `0/10` setpoints, never engages).

Until either a `/clock` bridge node is added or AirSim is put in true lockstep, the working approach is to stamp with the wall clock and tolerate the drift via `COM_OF_LOSS_T`.

## Offboard-mode entry

```
param set COM_OF_LOSS_T 300.0
```

Widens the OffboardControlMode freshness window (default 1 s) to cover the wall-clock vs sim-clock offset. `observed_offset` grows over a session (already ~141 s early), so choose a value larger than your longest expected drift. 300 s is comfortable headroom; raise to 600 for long sessions. The durable fix is keeping AirSim near real time (render caps / lockstep) so the offset stays small.

## Battery failsafe bypass

```
param set CBRK_SUPPLY_CHK 894281
```

AirSim's default battery model drains voltage very fast in sim time, so PX4's `battery_unhealthy` health flag trips within seconds of arming (drone takes off, failsafe fires, drone auto-lands, disarms). `894281` is the magic circuit-breaker value defined in `src/lib/circuit_breaker/circuit_breaker.h` (`CBRK_SUPPLY_CHK_KEY`) - required so nobody flips safety checks by accident.

## Persist

```
param save
```

Verify after next boot:

```
param show COM_OF_LOSS_T
param show CBRK_SUPPLY_CHK
```

Expected: `300.00000` and `894281`.

---

## One-liner to apply all at once

Paste into `pxh>`:

```
param set COM_OF_LOSS_T 300.0 ; param set CBRK_SUPPLY_CHK 894281 ; param save
```

---

## Useful pxh> diagnostic commands

```
listener offboard_control_mode 1     # latest OffboardControlMode uORB sample
listener failsafe_flags 1            # which health/failsafe flag is blocking
listener timesync_status 1           # agent <-> PX4 clock offset
listener vehicle_status 1            # current nav_state, arming_state
uorb top                             # live topic publication rates
commander mode offboard              # switch mode via shell (bypasses the node)
commander mode auto:loiter           # toggle offboard OFF
commander check                      # re-run preflight checks
```

Note: uORB topic names are lowercase snake_case, not CamelCase.

---

## Node-side code changes (already applied in airsim_offboard_control.py)

| Location | Change | Purpose |
|---|---|---|
| Line 47 | Subscribe to `/fmu/out/vehicle_status_v1` (was `/fmu/out/vehicle_status`) | PX4 v1.16 renamed the topic due to `MESSAGE_VERSION=1` on VehicleStatus |
| `engage_offboard_mode()` | Poll `nav_state == 14` for 2 s instead of unconditional success print | Detect commander rejection with a real pass/fail |
| `main()` | `while rclpy.ok() and self.running: rclpy.spin_once(...)` instead of `rclpy.spin(...)` | ESC key now actually exits (rclpy.spin blocks forever on flag changes) |

---

## If offboard still won't engage after all this

Order to try:

1. `param show COM_OF_LOSS_T` returns `300.0` (not the `1.0` default from a fresh boot, and not a stale persisted value that is smaller than your session drift).
2. `listener timesync_status 1` - read `observed_offset` (microseconds). `COM_OF_LOSS_T` must be larger than this magnitude, or OffboardControlMode samples will be rejected as stale. If the offset is huge and growing, the session drift exceeds the tolerance: raise `COM_OF_LOSS_T` or reduce AirSim render load.
3. `listener offboard_control_mode 1` while the node is publishing - confirm the `timestamp` field matches the wall-clock (`get_clock().now()`) domain and is being refreshed at ~20 Hz.
4. `listener failsafe_flags 1` - if `offboard_control_signal_lost: True` persists, the offset exceeded the tolerance. Bump it: `param set COM_OF_LOSS_T 600.0`.
5. Long-term: the node deliberately runs on the wall clock (`use_sim_time=false`) because this agent build publishes no `/clock`. The real root fix is either a `/clock` bridge node or keeping AirSim at real-time framerate (render caps / lockstep) so `observed_offset` stays small.
