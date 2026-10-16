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

### Choosing the smallest correct value (measure, don't guess)

Do not just pick a big number: `COM_OF_LOSS_T` is also the delay before PX4's **real** offboard-loss failsafe fires, so an oversized value disables that protection for minutes. The accurate setting is the smallest value that still exceeds the accumulated clock offset for your longest session. Offset grows at roughly `(1 - RTF)` s/s (RTF = AirSim real-time factor).

Measure the drift instead of guessing. With the node + agent running, watch the offset live:

```
ros2 topic echo /fmu/out/timesync_status
```

Take two `observed_offset` samples (microseconds) `dt` seconds apart, convert to seconds (`/1e6`), and compute `drift_s_per_min = (|offset2| - |offset1|) / dt * 60`. Then set `COM_OF_LOSS_T >= |observed_offset_now|(s) + drift_s_per_min * <longest session minutes>`, rounded up with ~25% margin.

## Battery failsafe bypass

```
param set CBRK_SUPPLY_CHK 894281
```

AirSim's default battery model drains voltage very fast in sim time, so PX4's `battery_unhealthy` health flag trips within seconds of arming (drone takes off, failsafe fires, drone auto-lands, disarms). `894281` is the magic circuit-breaker value defined in `src/lib/circuit_breaker/circuit_breaker.h` (`CBRK_SUPPLY_CHK_KEY`) - required so nobody flips safety checks by accident.

### Battery LEVEL failsafe (COM_LOW_BAT_ACT + thresholds)

`CBRK_SUPPLY_CHK` only bypasses the **supply / unhealthy** preflight check. AirSim's fast drain ALSO trips the **battery-level** failsafe, which lands/RTS the drone even with the supply breaker set:

```
WARN  [failsafe] Failsafe activated
INFO  [tone_alarm] battery warning (fast)
ERROR [flight_mode_manager] Matching flight task was not able to run, Nav state: 1, Task: 1
```

The `flight_mode_manager` ERROR is downstream noise from the mid-failsafe mode change, not a separate bug. Disable the level failsafe action and raise its capacity thresholds (PX4 v1.16 params):

```
param set COM_LOW_BAT_ACT 0      # 0 = Warning only (was 3); 1=Return, 2=Land, 3=RTGS
param set BAT_LOW_THR 0          # default 0.15 -> capacity warning level
param set BAT_CRIT_THR 0         # default 0.07 -> Return level
param set BAT_EMERGEN_THR 0      # default 0.05 -> Land (immediate) level
param save
```

Note: `COM_BAT_ENABLE` / `COM_BAT_FS_ACT` seen in older guides were removed in PX4 v1.14+ - use the params above. Diagnose the tripping flag with `listener battery_status 1` and `listener failsafe_flags 1` (look for `battery_warning` / `battery_critical`).

## Persist

```
param save
```

Verify after next boot:

```
param show COM_OF_LOSS_T
param show CBRK_SUPPLY_CHK
param show COM_LOW_BAT_ACT
```

Expected: `300.00000`, `894281`, and `0.00000`.

---

## One-liner to apply all at once

Paste into `pxh>`:

```
param set COM_OF_LOSS_T 300.0 ; param set CBRK_SUPPLY_CHK 894281 ; param set COM_LOW_BAT_ACT 0 ; param set BAT_LOW_THR 0 ; param set BAT_CRIT_THR 0 ; param set BAT_EMERGEN_THR 0 ; param save
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

## Silencing the `timesync` warning churn (optional)

The recurring PX4-console lines

```
WARN  [timesync] time jump detected. Resetting time synchroniser.
WARN  [uxrce_dds_client] time sync no longer converged
INFO  [uxrce_dds_client] time sync converged
```

are the uXRCE-DDS time synchronizer trying (and failing) to pull PX4's `hrt` toward the agent's host wall clock while AirSim keeps advancing `hrt` in slow, stepped increments. Mostly cosmetic, but each reset yanks `hrt` and can feed irregular `dt` into `ekf2`.

To stop the churn, disable the client-side time sync so PX4 keeps its own (AirSim-driven) clock:

```
param set UXRCE_DDS_SYNCT 0 ; param save
```

Caveat: this removes the log noise and the `hrt` jumps, but does **not** align the two clock domains -- the node still stamps setpoints with the wall clock, so the `COM_OF_LOSS_T` tolerance above is still required. Only a real `/clock` bridge (or AirSim lockstep) makes that value small again. Reference: PX4 ROS2 Issue #22522.

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
