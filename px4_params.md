# PX4 SITL + AirSim + ROS 2 Offboard: Required Parameters

Paste these into the running `pxh>` shell (started via `make px4_sitl none_iris`).
After applying, run `param save` once to persist across restarts.

## Offboard-mode entry

```
param set COM_OF_LOSS_T 120.0
```

Widens the OffboardControlMode freshness window (default 1 s). AirSim's Unreal sim loop runs below real-time framerate, so PX4's `hrt_absolute_time()` lags the ROS 2 wall clock. `timesync_status.observed_offset` grows to 240+ seconds during a session, making OffboardControlMode messages appear 6+ seconds stale by the time commander samples them. 120 s gives comfortable headroom. Bump to 300 or 600 for long sessions.

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

Expected: `120.00000` and `894281`.

---

## One-liner to apply all at once

Paste into `pxh>`:

```
param set COM_OF_LOSS_T 120.0 ; param set CBRK_SUPPLY_CHK 894281 ; param save
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

1. `param show COM_OF_LOSS_T` returns `120.0` (not `1.0` default from a fresh boot)
2. `listener offboard_control_mode 1` while the node is publishing - check that the msg `timestamp` is not more than a few seconds old
3. `listener failsafe_flags 1` - if `offboard_control_signal_lost: True` persists, the AirSim offset has grown beyond the tolerance. Bump it: `param set COM_OF_LOSS_T 300.0`
4. Long-term fix on the Windows side: reduce AirSim render quality so Unreal hits real-time framerate. The root cause of the entire time-sync drift is AirSim running slower than 60 FPS.
