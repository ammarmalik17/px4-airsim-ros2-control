#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import airsim
import threading
import queue
import time
import math
import sys
import termios
import tty
import select

# Optional dependencies for video display
try:
    import cv2  # type: ignore
    import numpy as np  # type: ignore
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore
    np = None  # type: ignore


class AirSimOffboardControl(Node):
    """Node for controlling AirSim drone using ROS2 px4_msgs with keyboard input."""

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
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_mode_enabled = False
        self.offboard_setpoint_counter = 0
        
        # Control variables
        self.target_position = [0.0, 0.0, -2.0]  # Start 2 meters above ground (NED frame)
        self.target_yaw = 0.0
        self.control_step = 0.5  # Movement step in meters
        self.yaw_step = 0.2  # Yaw step in radians
        
        # Velocity control variables
        self.target_velocity = [0.0, 0.0, 0.0]
        self.max_velocity_horizontal = 5.0  # Maximum horizontal velocity in m/s (WASD)
        self.max_velocity_vertical = 10.0   # Maximum vertical velocity in m/s (RF)
        self.velocity_step = 0.5  # Velocity increment
        self.velocity_decay = 1.0  # Velocity decay per second
        
        # Initialize AirSim connection
        self.airsim_client = None
        self.connect_airsim()

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        
        # Keyboard input handling
        self.running = True
        self.use_simple_input = False  # Fallback to simple input if needed
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_handler, daemon=True)
        self.keyboard_thread.start()

        # Video feed state
        self.video_running = False
        self.video_fetch_thread = None
        self.video_display_thread = None
        self.frame_queue = queue.Queue(maxsize=2)
        self.fetch_fps_smoothed = 0.0
        self.render_fps_smoothed = 0.0
        
        print('Controls:')
        print('  W/S: Forward/Backward (max 5 m/s)')
        print('  A/D: Left/Right (max 5 m/s)')
        print('  R/F: Up/Down (max 10 m/s)')
        print('  Q/E: Rotate Left/Right')
        print('  O: Toggle Offboard Mode')
        print('  V: Toggle AirSim Video Feed')
        print('  X: Stop (zero velocity)')
        print('  ESC: Exit')
        print('')
        print('IMPORTANT: Wait for 10 setpoints to be sent before pressing "O"')
        print('Note: Make sure this terminal has focus for keyboard input!')
        print('Velocity decays automatically when keys are released.')

    def connect_airsim(self):
        """Connect to AirSim running on Windows."""
        try:
            print('Connecting to AirSim at 192.168.224.1:41451...')
            self.airsim_client = airsim.MultirotorClient(ip="192.168.224.1", port=41451)
            self.airsim_client.confirmConnection()
            self.airsim_client.enableApiControl(True)
            print('✓ Connected to AirSim successfully')
        except Exception as e:
            print(f'✗ Failed to connect to AirSim: {e}')
            print('Make sure AirSim is running on Windows at 192.168.224.1:41451')
            self.airsim_client = None

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

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
        """Switch to offboard mode."""
        if self.offboard_setpoint_counter < 10:
            print(f"⚠ Need to send more setpoints before offboard mode ({self.offboard_setpoint_counter}/10)")
            return
            
        print("🚁 Engaging offboard mode...")
        
        # Send the mode change command (PX4 command to switch to offboard)
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        time.sleep(0.2)
        
        # Send arm command
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        
        self.offboard_mode_enabled = True
        print("✓ Offboard mode engaged - Ready to fly!")

    def disable_offboard_mode(self):
        """Disable offboard mode."""
        self.offboard_mode_enabled = False
        self.target_velocity = [0.0, 0.0, 0.0]  # Stop movement
        print("🛑 Offboard mode disabled")

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
        
        # Use velocity control for more responsive movement
        msg.position = [float('nan'), float('nan'), float('nan')]  # Don't control position directly
        msg.velocity = [float(self.target_velocity[0]), float(self.target_velocity[1]), float(self.target_velocity[2])]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float(self.target_yaw)
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
        
        # Increment counter for PX4 offboard mode requirements
        if self.offboard_setpoint_counter < 10:
            self.offboard_setpoint_counter += 1

    def apply_velocity_decay(self):
        """Apply velocity decay to gradually stop the drone."""
        decay_rate = self.velocity_decay * 0.05  # Timer runs at 20Hz (0.05s interval)
        
        for i in range(3):
            if self.target_velocity[i] > 0:
                self.target_velocity[i] = max(0, self.target_velocity[i] - decay_rate)
            elif self.target_velocity[i] < 0:
                self.target_velocity[i] = min(0, self.target_velocity[i] + decay_rate)

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
                    self.target_velocity = [0.0, 0.0, 0.0]
                    print('⏹ Stopping movement')
                elif char == 'v':
                    self.toggle_video_feed()
                elif self.offboard_mode_enabled:
                    # Movement controls (only work in offboard mode)
                    if char == 'w':  # Forward (North in NED)
                        self.target_velocity[0] = min(self.target_velocity[0] + self.velocity_step, self.max_velocity_horizontal)
                        if self.target_velocity[0] >= 0:
                            print(f'⬆ Forward: {self.target_velocity[0]:.1f} m/s')
                        else:
                            print(f'⬇ Backward: {abs(self.target_velocity[0]):.1f} m/s')
                    elif char == 's':  # Backward (South in NED)
                        self.target_velocity[0] = max(self.target_velocity[0] - self.velocity_step, -self.max_velocity_horizontal)
                        if self.target_velocity[0] <= 0:
                            print(f'⬇ Backward: {abs(self.target_velocity[0]):.1f} m/s')
                        else:
                            print(f'⬆ Forward: {self.target_velocity[0]:.1f} m/s')
                    elif char == 'a':  # Left (West in NED)
                        self.target_velocity[1] = max(self.target_velocity[1] - self.velocity_step, -self.max_velocity_horizontal)
                        if self.target_velocity[1] <= 0:
                            print(f'⬅ Left: {abs(self.target_velocity[1]):.1f} m/s')
                        else:
                            print(f'➡ Right: {self.target_velocity[1]:.1f} m/s')
                    elif char == 'd':  # Right (East in NED)
                        self.target_velocity[1] = min(self.target_velocity[1] + self.velocity_step, self.max_velocity_horizontal)
                        if self.target_velocity[1] >= 0:
                            print(f'➡ Right: {self.target_velocity[1]:.1f} m/s')
                        else:
                            print(f'⬅ Left: {abs(self.target_velocity[1]):.1f} m/s')
                    elif char == 'r':  # Up (negative Z in NED)
                        self.target_velocity[2] = max(self.target_velocity[2] - self.velocity_step, -self.max_velocity_vertical)
                        if self.target_velocity[2] <= 0:
                            print(f'🔼 Up: {abs(self.target_velocity[2]):.1f} m/s')
                        else:
                            print(f'🔽 Down: {self.target_velocity[2]:.1f} m/s')
                    elif char == 'f':  # Down (positive Z in NED)
                        self.target_velocity[2] = min(self.target_velocity[2] + self.velocity_step, self.max_velocity_vertical)
                        if self.target_velocity[2] >= 0:
                            print(f'🔽 Down: {self.target_velocity[2]:.1f} m/s')
                        else:
                            print(f'🔼 Up: {abs(self.target_velocity[2]):.1f} m/s')
                    elif char == 'q':  # Rotate left
                        self.target_yaw -= self.yaw_step
                        if self.target_yaw < -math.pi:
                            self.target_yaw += 2 * math.pi
                        print(f'↺ Yaw: {math.degrees(self.target_yaw):.0f}°')
                    elif char == 'e':  # Rotate right
                        self.target_yaw += self.yaw_step
                        if self.target_yaw > math.pi:
                            self.target_yaw -= 2 * math.pi
                        print(f'↻ Yaw: {math.degrees(self.target_yaw):.0f}°')
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
            except:
                pass

    def toggle_video_feed(self):
        """Toggle the AirSim video feed window."""
        if self.video_running:
            self.stop_video_feed()
        else:
            self.start_video_feed()

    def start_video_feed(self):
        """Start producer/consumer threads to display live video from AirSim."""
        if cv2 is None or np is None:
            print('⚠ OpenCV (cv2) and NumPy are required for video. Install with: pip install opencv-python numpy')
            return
        if self.airsim_client is None:
            print('⚠ Cannot start video: not connected to AirSim')
            return
        if self.video_running:
            return
        self.video_running = True
        self.fetch_fps_smoothed = 0.0
        self.render_fps_smoothed = 0.0
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
        # Give the thread a moment to exit its loop
        time.sleep(0.05)
        try:
            if cv2 is not None:
                cv2.destroyAllWindows()
        except Exception:
            pass
        print('🖥 AirSim video feed OFF')

    def video_fetch_loop(self):
        """Producer thread: fetch frames from AirSim as uncompressed and push latest to queue."""
        alpha = 0.9
        prev_time = time.perf_counter()
        while self.running and self.video_running:
            try:
                responses = self.airsim_client.simGetImages([
                    airsim.ImageRequest("0", airsim.ImageType.Scene, pixels_as_float=False, compress=False)
                ])
                if not responses or responses[0] is None or responses[0].width == 0:
                    time.sleep(0.005)
                    continue
                img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)  # type: ignore
                width = responses[0].width
                height = responses[0].height
                expected_rgb = width * height * 3
                expected_rgba = width * height * 4
                frame_bgr = None
                if img1d.size == expected_rgba:
                    img_bgra = img1d
                    img_bgra.shape = (height, width, 4)
                    frame_bgr = img_bgra[:, :, :3]
                elif img1d.size == expected_rgb:
                    frame_bgr = img1d
                    frame_bgr.shape = (height, width, 3)
                else:
                    time.sleep(0.002)
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
            except Exception:
                time.sleep(0.005)

    def video_display_loop(self):
        """Consumer thread: display latest frames from queue with FPS overlay."""
        window_name = 'AirSim Camera'
        try:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)  # type: ignore
        except Exception:
            pass
        alpha = 0.9
        prev_time = time.perf_counter()
        last_frame = None
        while self.running and self.video_running:
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
                    cv2.putText(frame, text, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)  # type: ignore
                except Exception:
                    pass
                cv2.imshow(window_name, frame)  # type: ignore
                cv2.waitKey(1)  # type: ignore
            except Exception:
                time.sleep(0.005)
        # Window closed in stop_video_feed


def main(args=None) -> None:
    print('Starting AirSim Offboard Control Node...')
    print('Make sure AirSim is running on Windows at 192.168.100.10 and accessible from Jetson')
    
    rclpy.init(args=args)
    
    try:
        offboard_control = AirSimOffboardControl()
        
        # Handle shutdown gracefully
        def signal_handler():
            offboard_control.shutdown()
            
        try:
            rclpy.spin(offboard_control)
        except KeyboardInterrupt:
            pass
        finally:
            signal_handler()
            offboard_control.destroy_node()
            
    except Exception as e:
        print(f'Error: {e}')
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main() 