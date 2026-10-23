#!/usr/bin/env python3

import math
import queue
import select
import signal
import sys
import termios
import threading
import time
import tty

import airsim
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
)

# Default AirSim RPC endpoint (single source of truth; full ROS2 parameterization is tracked separately)
AIRSIM_IP = "192.168.224.1"
AIRSIM_PORT = 41451


class AirSimOffboardControl(Node):
    """Node for controlling AirSim drone using ROS2 px4_msgs with keyboard input."""

    PUBLISH_PERIOD_S = 0.05        # 20 Hz timer period; single source for timer + velocity-decay math
    OFFBOARD_MIN_SETPOINTS = 10    # setpoints required before engage/takeoff/land are allowed
    # key -> (velocity axis, step sign) for the table-driven movement dispatch
    MOVEMENT_KEYS = {'w': (0, 1), 's': (0, -1), 'a': (1, -1), 'd': (1, 1), 'r': (2, -1), 'f': (2, 1)}

    def __init__(self) -> None:
        super().__init__('airsim_offboard_control')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.vehicle_status = VehicleStatus()
        self.offboard_mode_enabled = False
        self.offboard_setpoint_counter = 0
        
        # Control variables
        self.target_yaw = 0.0
        self.yaw_step = 0.2  # Yaw step in radians
        self.takeoff_altitude = 2.0  # Target altitude for auto takeoff (meters above ground)
        
        # Velocity control variables
        self.target_velocity = [0.0, 0.0, 0.0]
        self._ctrl_lock = threading.Lock()  # Guards target_velocity/target_yaw across timer + keyboard threads
        self.max_velocity_horizontal = 5.0  # Maximum horizontal velocity in m/s (WASD)
        self.max_velocity_vertical = 10.0   # Maximum vertical velocity in m/s (RF)
        self.velocity_step = 0.5  # Velocity increment
        self.velocity_decay = 1.0  # Velocity decay per second
        
        # Initialize AirSim connection
        self.airsim_client = None
        self.airsim_ip = AIRSIM_IP
        self.airsim_port = AIRSIM_PORT
        self.connect_airsim()

        # Create a timer to publish control commands
        self.timer = self.create_timer(self.PUBLISH_PERIOD_S, self.timer_callback)  # 20Hz
        
        # Keyboard input handling
        self.running = True
        self.use_simple_input = False  # Fallback to simple input if needed
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_handler, daemon=True)
        self.keyboard_thread.start()

        # Video feed state
        self.video_running = False
        self._video_stop = threading.Event()
        self.video_fetch_thread = None
        self.video_display_thread = None
        self.frame_queue = queue.Queue(maxsize=2)
        self.fetch_fps_smoothed = 0.0
        self.render_fps_smoothed = 0.0
        self._last_fetch_err_log = 0.0
        self._last_render_err_log = 0.0
        # Dedicated AirSim client for the video pipeline so its RPC session/IOLoop
        # is never shared with (or raced by) the main-thread control client.
        self.video_client = None
        self.video_target_fps = 30.0
        
        print('Controls:')
        print('  W/S: Forward/Backward (max 5 m/s)')
        print('  A/D: Left/Right (max 5 m/s)')
        print('  R/F: Up/Down (max 10 m/s)')
        print('  Q/E: Rotate Left/Right')
        print('  O: Toggle Offboard Mode')
        print('  V: Toggle AirSim Video Feed')
        print('  T: Takeoff')
        print('  L: Land')
        print('  X: Stop (zero velocity)')
        print('  ESC: Exit')
        print()
        print(f'IMPORTANT: Wait for {self.OFFBOARD_MIN_SETPOINTS} setpoints to be sent before pressing "O"')
        print('Note: Make sure this terminal has focus for keyboard input!')
        print('Velocity decays automatically when keys are released.')

    def connect_airsim(self):
        """Connect to AirSim running on Windows."""
        try:
            print(f'Connecting to AirSim at {self.airsim_ip}:{self.airsim_port}...')
            self.airsim_client = airsim.MultirotorClient(ip=self.airsim_ip, port=self.airsim_port)
            self.airsim_client.confirmConnection()
            self.airsim_client.enableApiControl(True)
            print('✓ Connected to AirSim successfully')
        except Exception as e:
            print(f'✗ Failed to connect to AirSim: {e}')
            print(f'Make sure AirSim is running at {AIRSIM_IP}:{AIRSIM_PORT}')
            self.airsim_client = None

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def engage_offboard_mode(self):
        """Switch to offboard mode and verify that PX4 actually accepted it."""
        if self.offboard_setpoint_counter < self.OFFBOARD_MIN_SETPOINTS:
            print(f"⚠ Need to send more setpoints before offboard mode ({self.offboard_setpoint_counter}/{self.OFFBOARD_MIN_SETPOINTS})")
            return

        print("🚁 Engaging offboard mode...")

        # Arm only if not already armed (avoids resetting an existing auto mode)
        if self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            time.sleep(0.1)

        # Send the mode change command (PX4 command to switch to offboard)
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

        # Poll vehicle_status_v1 for up to 2 s to confirm commander accepted OFFBOARD
        for _ in range(40):
            time.sleep(0.05)
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.offboard_mode_enabled = True
                print("✓ Offboard mode engaged (verified via nav_state=14)")
                return

        self.offboard_mode_enabled = False
        print(f"✗ Offboard NOT engaged. nav_state={self.vehicle_status.nav_state} "
              f"(expected {VehicleStatus.NAVIGATION_STATE_OFFBOARD}), "
              f"arming_state={self.vehicle_status.arming_state}")
        print("  Likely cause: OffboardControlMode freshness failing (wall-clock vs sim-clock drift).")
        print("  See px4_params.md: COM_OF_LOSS_T must exceed the timesync observed_offset "
              "(300.0 typical, 600.0 for long sessions). Verify with `param show COM_OF_LOSS_T`.")

    def disable_offboard_mode(self):
        """Exit offboard by commanding AUTO LOITER and stop applying movement setpoints."""
        self.offboard_mode_enabled = False
        with self._ctrl_lock:
            self.target_velocity = [0.0, 0.0, 0.0]  # Stop movement
        # Leave OFFBOARD: param1=1 custom base, param2=4 AUTO main mode,
        # param3=3 AUTO_LOITER sub-mode (PX4_CUSTOM_SUB_MODE_AUTO_LOITER; 2 would be TAKEOFF).
        # (CLI equivalent: `commander mode auto:loiter`, cf. px4_params.md diagnostics)
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=4.0, param3=3.0)

        # Poll vehicle_status_v1 up to 1 s to confirm the commander accepted AUTO LOITER,
        # mirroring the OFFBOARD acceptance check in engage_offboard_mode.
        for _ in range(20):
            time.sleep(0.05)
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                print("🛑 Offboard exited: AUTO LOITER engaged (hover)")
                return
        print(f"⚠ Offboard exit NOT confirmed. nav_state={self.vehicle_status.nav_state} "
              f"(expected {VehicleStatus.NAVIGATION_STATE_AUTO_LOITER}); vehicle remains in OFFBOARD/hover")

    def takeoff(self):
        """Command PX4 to perform an auto takeoff to takeoff_altitude."""
        if self.offboard_setpoint_counter < self.OFFBOARD_MIN_SETPOINTS:
            print(f"⚠ Need to send more setpoints before takeoff ({self.offboard_setpoint_counter}/{self.OFFBOARD_MIN_SETPOINTS})")
            return

        # Arm only if not already armed (AUTO_TAKEOFF requires arming)
        if self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.arm()
            time.sleep(0.1)

        self.offboard_mode_enabled = False  # Leaving offboard for the auto-takeoff sequence
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param7=self.takeoff_altitude)
        print(f"🛫 Takeoff commanded to {self.takeoff_altitude:.1f} m")

    def land(self):
        """Command PX4 to land and exit offboard control."""
        if self.offboard_setpoint_counter < self.OFFBOARD_MIN_SETPOINTS:
            print(f"⚠ Need to send more setpoints before landing ({self.offboard_setpoint_counter}/{self.OFFBOARD_MIN_SETPOINTS})")
            return
        with self._ctrl_lock:
            self.target_velocity = [0.0, 0.0, 0.0]  # Stop horizontal motion
        self.offboard_mode_enabled = False  # Movement keys no longer apply once landing
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        print("🛬 Landing commanded")

    def publish_offboard_control_mode(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True  # Use velocity control for more responsive movement
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()

        # Read velocity/yaw under the lock so the timer never observes a list
        # half-updated by the keyboard thread; release before publishing.
        with self._ctrl_lock:
            vx, vy, vz = self.target_velocity[0], self.target_velocity[1], self.target_velocity[2]
            yaw = self.target_yaw

        # Use velocity control for more responsive movement
        msg.position = [float('nan'), float('nan'), float('nan')]  # Don't control position directly
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float(yaw)
        msg.yawspeed = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        # Apply velocity decay
        self.apply_velocity_decay()
        
        # Always publish offboard control mode and setpoints for PX4 to be ready
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()
        
        # Monotonic counter for the PX4 offboard-mode readiness requirement
        self.offboard_setpoint_counter += 1

    def apply_velocity_decay(self):
        """Apply velocity decay to gradually stop the drone."""
        decay_rate = self.velocity_decay * self.PUBLISH_PERIOD_S  # timer period single-sourced via PUBLISH_PERIOD_S

        with self._ctrl_lock:
            for i in range(3):
                if self.target_velocity[i] > 0:
                    self.target_velocity[i] = max(0, self.target_velocity[i] - decay_rate)
                elif self.target_velocity[i] < 0:
                    self.target_velocity[i] = min(0, self.target_velocity[i] + decay_rate)

    def _apply_axis_step(self, axis, sign):
        """Advance one velocity axis by one step in `sign` direction under the lock."""
        vmax = self.max_velocity_vertical if axis == 2 else self.max_velocity_horizontal
        with self._ctrl_lock:
            new = self.target_velocity[axis] + sign * self.velocity_step
            new = max(-vmax, min(vmax, new))
            self.target_velocity[axis] = new
            v = self.target_velocity[axis]
        self._report_axis(axis, v)

    def _report_axis(self, axis, v):
        """Reproduce the sign-based direction feedback (unchanged from the pre-refactor output)."""
        if axis == 0:
            if v >= 0:
                print(f'⬆ Forward: {v:.1f} m/s')
            else:
                print(f'⬇ Backward: {abs(v):.1f} m/s')
        elif axis == 1:
            if v <= 0:
                print(f'⬅ Left: {abs(v):.1f} m/s')
            else:
                print(f'➡ Right: {v:.1f} m/s')
        elif axis == 2:
            if v <= 0:
                print(f'🔼 Up: {abs(v):.1f} m/s')
            else:
                print(f'🔽 Down: {v:.1f} m/s')

    def _apply_yaw_step(self, sign):
        """Rotate yaw by one step, wrapping to [-pi, pi], under the lock."""
        with self._ctrl_lock:
            self.target_yaw += sign * self.yaw_step
            if self.target_yaw < -math.pi:
                self.target_yaw += 2 * math.pi
            elif self.target_yaw > math.pi:
                self.target_yaw -= 2 * math.pi
            yaw = self.target_yaw
        if sign < 0:
            print(f'↺ Yaw: {math.degrees(yaw):.0f}°')
        else:
            print(f'↻ Yaw: {math.degrees(yaw):.0f}°')

    def get_char(self):
        """Get a single character from stdin."""
        try:
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setcbreak(fd)  # Use setcbreak instead of cbreak
                if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                    ch = sys.stdin.read(1)
                else:
                    ch = None
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch
        except (AttributeError, OSError) as e:
            # Fallback for systems where tty.setcbreak doesn't work
            if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                return sys.stdin.read(1)
            return None

    def keyboard_input_handler(self):
        """Handle keyboard input for drone control."""
        error_count = 0
        max_errors = 10
        
        while self.running:
            try:
                if not self.use_simple_input:
                    char = self.get_char()
                else:
                    # Simple input fallback
                    try:
                        char = input("Enter command (w/a/s/d/r/f/q/e/o/x/exit): ").lower()
                        if char == 'exit':
                            char = '\x1b'
                    except (EOFError, KeyboardInterrupt):
                        char = '\x1b'
                        
                if char is None:
                    continue
                    
                char = char.lower()
                
                if char == '\x1b' or char == 'exit':  # ESC key or exit command
                    print('👋 Exiting...')
                    self.running = False
                    break
                elif char == 'o':
                    if not self.offboard_mode_enabled:
                        self.engage_offboard_mode()
                    else:
                        self.disable_offboard_mode()
                elif char == 'x':
                    # Stop all movement
                    with self._ctrl_lock:
                        self.target_velocity = [0.0, 0.0, 0.0]
                    print('⏹ Stopping movement')
                elif char == 't':
                    self.takeoff()
                elif char == 'l':
                    self.land()
                elif char == 'v':
                    self.toggle_video_feed()
                elif self.offboard_mode_enabled:
                    # Movement controls (only work in offboard mode); table-driven dispatch
                    # centralizes the per-key clamp/feedback that was previously duplicated.
                    if char in self.MOVEMENT_KEYS:
                        axis, sign = self.MOVEMENT_KEYS[char]
                        self._apply_axis_step(axis, sign)
                    elif char == 'q':
                        self._apply_yaw_step(-1)
                    elif char == 'e':
                        self._apply_yaw_step(1)
                else:
                    if char in ['w', 'a', 's', 'd', 'r', 'f', 'q', 'e']:
                        print('⚠ Movement commands only work in offboard mode. Press "o" to enable.')
                        
                error_count = 0  # Reset error count on successful input
                
            except Exception as e:
                error_count += 1
                if error_count > max_errors:
                    print('⚠ Too many keyboard input errors, switching to simple input mode')
                    self.use_simple_input = True
                    error_count = 0
                    continue
                time.sleep(0.1)

    def shutdown(self):
        """Clean shutdown."""
        self.running = False
        # Stop video feed if running
        try:
            self.stop_video_feed()
        except Exception:
            pass
        if self.offboard_mode_enabled:
            self.disable_offboard_mode()
        if self.airsim_client:
            try:
                self.airsim_client.enableApiControl(False)
                print('✓ AirSim API control disabled')
            except Exception:
                pass

    def toggle_video_feed(self):
        """Toggle the AirSim video feed window."""
        if self.video_running:
            self.stop_video_feed()
        else:
            self.start_video_feed()

    def start_video_feed(self):
        """Start producer/consumer threads to display live video from AirSim."""
        if self.airsim_client is None:
            print('⚠ Cannot start video: not connected to AirSim')
            return
        if self.video_running:
            return
        # Create a dedicated AirSim client for the video pipeline so its RPC
        # session/IOLoop is used only by the video threads, never contending
        # with the main-thread control client (self.airsim_client).
        try:
            self.video_client = airsim.MultirotorClient(ip=self.airsim_ip, port=self.airsim_port)
            self.video_client.confirmConnection()
        except Exception as e:
            print(f'\u26a0 Cannot start video: failed to connect dedicated client: {e}')
            self.video_client = None
            return
        self.video_running = True
        self._video_stop.clear()
        self.fetch_fps_smoothed = 0.0
        self.render_fps_smoothed = 0.0
        self._last_fetch_err_log = 0.0
        self._last_render_err_log = 0.0
        # Drop any frames left over from a previous session so a restart shows fresh video
        while True:
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                break
        # Start producer and consumer threads
        self.video_fetch_thread = threading.Thread(target=self.video_fetch_loop, daemon=True)
        self.video_display_thread = threading.Thread(target=self.video_display_loop, daemon=True)
        self.video_fetch_thread.start()
        self.video_display_thread.start()
        print('📺 AirSim video feed ON (press V to stop)')

    def stop_video_feed(self):
        """Stop the video feed and close windows."""
        if not self.video_running:
            return
        self.video_running = False
        # Signal both video threads to exit, then join before touching the client so the
        # fetch loop can no longer be mid-RPC on self.video_client (TOCTOU elimination).
        self._video_stop.set()
        if self.video_fetch_thread is not None:
            self.video_fetch_thread.join(timeout=1.0)
        if self.video_display_thread is not None:
            self.video_display_thread.join(timeout=1.0)
        self.video_fetch_thread = None
        self.video_display_thread = None
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        # Close the dedicated video client (separate session from the main control client)
        if self.video_client is not None:
            try:
                self.video_client.close()
            except Exception:
                pass
            self.video_client = None
        print('🖥 AirSim video feed OFF')

    def video_fetch_loop(self):
        """Producer thread: fetch compressed frames via the dedicated video client,
        decode them with OpenCV, and push the latest to the queue at a capped rate."""
        alpha = 0.9
        prev_time = time.perf_counter()
        interval = 1.0 / self.video_target_fps if self.video_target_fps > 0 else 0.0
        next_tick = time.perf_counter()
        while self.running and not self._video_stop.is_set():
            try:
                client = self.video_client
                if client is None:
                    break
                responses = client.simGetImages([
                    airsim.ImageRequest("0", airsim.ImageType.Scene, pixels_as_float=False, compress=True)
                ])
                if not responses or responses[0] is None or responses[0].width == 0:
                    continue
                arr = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
                frame_bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame_bgr is None:
                    continue
                # Update fetch FPS
                now = time.perf_counter()
                dt = now - prev_time
                prev_time = now
                if dt > 0:
                    inst = 1.0 / dt
                    self.fetch_fps_smoothed = inst if self.fetch_fps_smoothed == 0.0 else (alpha * self.fetch_fps_smoothed + (1 - alpha) * inst)
                # Push latest frame, dropping oldest if full
                try:
                    self.frame_queue.put_nowait(frame_bgr)
                except queue.Full:
                    try:
                        _ = self.frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        self.frame_queue.put_nowait(frame_bgr)
                    except queue.Full:
                        pass
            except Exception as e:
                # Rate-limited log so a persistent RPC/decode failure is visible but not spammy
                now = time.monotonic()
                if now - self._last_fetch_err_log >= 1.0:
                    self.get_logger().warning(f'Video fetch error: {e}')
                    self._last_fetch_err_log = now
                # Reset the pacing baseline after a stall to avoid a sleep burst
                next_tick = time.perf_counter()
            finally:
                # Pace the loop to video_target_fps (runs on success, continue and error paths)
                if interval > 0:
                    next_tick += interval
                    delay = next_tick - time.perf_counter()
                    if delay > 0:
                        time.sleep(delay)

    def video_display_loop(self):
        """Consumer thread: display latest frames from queue with FPS overlay."""
        window_name = 'AirSim Camera'
        try:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        except Exception:
            pass
        alpha = 0.9
        prev_time = time.perf_counter()
        last_frame = None
        while self.running and not self._video_stop.is_set():
            try:
                frame = None
                # Get at least one frame (blocking up to 100 ms)
                try:
                    frame = self.frame_queue.get(timeout=0.1)
                except queue.Empty:
                    if last_frame is None:
                        continue
                    frame = last_frame
                # Drain remaining frames to keep only the most recent
                while True:
                    try:
                        frame = self.frame_queue.get_nowait()
                    except queue.Empty:
                        break
                last_frame = frame
                # Update render FPS
                now = time.perf_counter()
                dt = now - prev_time
                prev_time = now
                if dt > 0:
                    inst = 1.0 / dt
                    self.render_fps_smoothed = inst if self.render_fps_smoothed == 0.0 else (alpha * self.render_fps_smoothed + (1 - alpha) * inst)
                # Overlay FPS: fetch and render
                try:
                    text = f"F:{self.fetch_fps_smoothed:.1f} | R:{self.render_fps_smoothed:.1f} FPS"
                    cv2.putText(frame, text, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                except Exception:
                    pass
                cv2.imshow(window_name, frame)
                cv2.waitKey(1)
            except Exception as e:
                now = time.monotonic()
                if now - self._last_render_err_log >= 1.0:
                    self.get_logger().warning(f'Video render error: {e}')
                    self._last_render_err_log = now
                time.sleep(0.005)
        # Window closed in stop_video_feed


def main(args=None) -> None:
    print('Starting AirSim Offboard Control Node...')
    print(f'Make sure AirSim is running and reachable at {AIRSIM_IP}:{AIRSIM_PORT}')
    
    rclpy.init(args=args)
    
    try:
        offboard_control = AirSimOffboardControl()

        def cleanup():
            offboard_control.shutdown()

        def _handle_signal(signum, frame):
            # Only flag the loop to stop; cleanup runs once from the finally below.
            offboard_control.running = False

        # Handle SIGINT/SIGTERM (e.g. `ros2 launch` shutdown, `kill`) gracefully while
        # preserving rclpy's own handlers by restoring them afterward.
        prev_sigint = signal.signal(signal.SIGINT, _handle_signal)
        prev_sigterm = signal.signal(signal.SIGTERM, _handle_signal)
        try:
            # Poll spin_once so we exit cleanly when the keyboard thread or a signal sets running=False
            while rclpy.ok() and offboard_control.running:
                rclpy.spin_once(offboard_control, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            signal.signal(signal.SIGINT, prev_sigint)
            signal.signal(signal.SIGTERM, prev_sigterm)
            cleanup()
            offboard_control.destroy_node()
            
    except Exception as e:
        print(f'Error: {e}')
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main() 