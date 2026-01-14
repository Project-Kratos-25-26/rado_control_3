#!/usr/bin/env python3
# =================================================================================================
# STAGE 1: SETUP & INIT
# =================================================================================================
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
import json
import math
import time

class ConeFollower(Node):
    def __init__(self):
        super().__init__('cone_follower')

        # -------------------------------------------------------------------------
        # 1.1: State Variables
        # These variables keep track of what the robot is doing and seeing.
        # -------------------------------------------------------------------------
        self.active = False               # Is the robot allowed to move?
        self.target_color = None          # What color cone are we chasing? (e.g., 'blue', 'orange')
        self.latest_detections = []       # List of cones currently visible to the camera
        self.image_width = 1280.0         # Camera resolution width (updates dynamically from JSON)
        
        # State machine for cache box sequence
        self.state = 'IDLE'               # States: IDLE, FOLLOWING, ROTATING, COUNTDOWN, DONE
        self.rotation_start_time = None   # Time when we started the 180 degree rotation
        self.countdown_start_time = None  # Time when we started the countdown
        self.countdown_duration = 10      # Countdown duration in seconds
        self.goal_type = None             # 'pickup' or 'dropoff' - only dropoff does rotation+countdown
        
        # Compass heading for precise rotation
        self.current_heading = None       # Current compass heading in degrees (0-360)
        self.target_heading = None        # Target heading for 180 degree rotation
        
        # -------------------------------------------------------------------------
        # 1.2: Tuning Parameters (Merged from Old Script)
        # Adjust these numbers to change how the robot behaves.
        # -------------------------------------------------------------------------

        # -------------------------------------------------------------------------
        # 1.2: Tuning Parameters (Merged from Old Script)
        # Adjust these numbers to change how the robot behaves.
        # -------------------------------------------------------------------------
        self.stop_distance = 1.0  # meters - Updated to match old script threshold (1.0m)
        self.linear_vel = 0.22    # Constant driving speed
        self.yaw_threshold = 40   # Logic threshold for "Drive Only" vs "Turn+Drive" (based on 640px width)
        self.conf_threshold = 0.3 # Minimum confidence to accept a detection
        self.rotation_speed = 0.5 # Angular speed for 180 degree rotation (rad/s)
        self.heading_tolerance = 5.0  # Degrees tolerance for heading-based rotation

        # -------------------------------------------------------------------------
        # 1.3: ROS 2 Communication
        # Setting up how we talk to other parts of the robot.
        # -------------------------------------------------------------------------
        # Publishers: Sending commands OUT
        self.velocity_pub = self.create_publisher(Twist, '/auto/cmd_vel', 10)          # Driving instructions
        self.status_pub = self.create_publisher(String, '/auto/cone_follow/status', 10) # Tell the world what we are doing

        # Subscribers: Listening for messages IN
        self.create_subscription(String, '/cone_detector/detections', self.detection_callback, 10) # Camera data
        self.create_subscription(String, '/auto/cone_follow/trigger', self.trigger_callback, 10)   # remote control / brain commands
        self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.compass_callback, 10)  # Compass heading

        # Timer: The heartbeat of the node. Runs the control loop 10 times a second (0.1s).
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f'Cone Follower Node Ready (Slave Mode - Turn-Then-Drive). Stop Dist: {self.stop_distance}m')

    # =================================================================================================
    # STAGE 2: INPUT HANDLING (Callbacks)
    # =================================================================================================

    def compass_callback(self, msg):
        """
        Received compass heading from mavros.
        Heading is in degrees (0-360, where 0 is North).
        """
        self.current_heading = msg.data

    def trigger_callback(self, msg):
        """
        Received a command from the main brain or user.
        Commands: 'stop' or 'color|type' (e.g., 'yellow|dropoff', 'blue|pickup').
        """
        command = msg.data.lower().strip()
        
        if command == 'stop':
            # Stop everything immediately
            self.active = False
            self.target_color = None
            self.goal_type = None
            self.state = 'IDLE'
            self.stop_rover()
            self.get_logger().info('Cone Follower STOPPED')
        else:
            # Parse color and goal type (format: "color|type" or just "color")
            parts = command.split('|')
            self.target_color = parts[0].strip()
            self.goal_type = parts[1].strip() if len(parts) > 1 else 'pickup'  # default to pickup
            self.active = True
            self.state = 'FOLLOWING'
            self.latest_detections = [] # Clear old data so we don't react to ghosts
            self.get_logger().info(f'Cone Follower ACTIVATED. Target: {self.target_color} ({self.goal_type})')
            self.status_pub.publish(String(data="BUSY"))

    def detection_callback(self, msg):
        """
        Received new data from the camera node (cone_detector).
        The message is a JSON string containing a list of detected cones.
        """
        try:
            data = json.loads(msg.data)
            dets = data.get('detections', [])
            
            # Robust filtering
            filtered = []
            for d in dets:
                # 1. Confidence Check
                conf = float(d.get('confidence', 1.0)) if d.get('confidence') is not None else 1.0
                if conf < self.conf_threshold:
                    continue
                
                # 2. Key Validation (Prevent Crashes)
                if 'center' not in d or 'depth_m' not in d:
                    continue
                    
                # 3. Type Conversion (Ensure floats)
                try:
                    d['depth_m'] = float(d.get('depth_m', 0.0))
                    d['center'] = [float(d['center'][0]), float(d['center'][1])]
                    d['confidence'] = float(conf)
                except (ValueError, TypeError):
                    continue
                    
                filtered.append(d)

            self.latest_detections = filtered
            
            # Update image width if the camera node sent it, so our math matches the camera resolution
            if 'width' in data:
                try:
                    self.image_width = float(data['width'])
                except (ValueError, TypeError):
                    pass
        except json.JSONDecodeError:
            pass

    def stop_rover(self):
        """Helper to act just stop the wheels."""
        self.velocity_pub.publish(Twist())

    def normalize_heading(self, heading):
        """Normalize heading to 0-360 range."""
        while heading < 0:
            heading += 360
        while heading >= 360:
            heading -= 360
        return heading
    
    def heading_error(self, target, current):
        """Calculate the shortest angular distance between two headings (-180 to 180)."""
        diff = target - current
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff

    # =================================================================================================
    # STAGE 3: MAIN CONTROL LOOP
    # This runs 10 times per second.
    # =================================================================================================
    def control_loop(self):
        # 3.1: Safety Check
        # If we aren't active or don't have a target color, do nothing.
        if not self.active or not self.target_color:
            return
        
        # =========================================================================================
        # STATE MACHINE
        # =========================================================================================
        
        # --- STATE: ROTATING (180 degree turn after reaching cone using compass) ---
        if self.state == 'ROTATING':
            if self.current_heading is None:
                # No compass data yet, wait
                self.get_logger().warn('Waiting for compass heading...', throttle_duration_sec=1.0)
                return
            
            # If target heading wasn't set (no compass at goal reach), set it now
            if self.target_heading is None:
                self.target_heading = self.normalize_heading(self.current_heading + 180.0)
                self.get_logger().info(f'Target heading set: {self.current_heading:.1f}° -> {self.target_heading:.1f}°')
            
            # Calculate heading error to target
            error = self.heading_error(self.target_heading, self.current_heading)
            
            if abs(error) <= self.heading_tolerance:
                # Rotation complete, start countdown
                self.stop_rover()
                self.state = 'COUNTDOWN'
                self.countdown_start_time = time.time()
                self.get_logger().info(f'Rotation complete! Heading: {self.current_heading:.1f}° (target: {self.target_heading:.1f}°)')
                self.get_logger().info('Starting cache box sequence...')
            else:
                # Keep rotating toward target heading
                twist = Twist()
                # Use proportional control for smoother approach, with min speed
                angular_speed = max(0.2, min(self.rotation_speed, abs(error) * 0.02))
                # Rotate in the direction of shorter path
                twist.angular.z = angular_speed if error > 0 else -angular_speed
                self.velocity_pub.publish(twist)
                self.get_logger().info(f'Rotating 180°... Current: {self.current_heading:.1f}° -> Target: {self.target_heading:.1f}° (error: {error:.1f}°)', throttle_duration_sec=0.5)
            return
        
        # --- STATE: COUNTDOWN (Opening cache box countdown) ---
        if self.state == 'COUNTDOWN':
            current_time = time.time()
            elapsed = current_time - self.countdown_start_time
            remaining = int(self.countdown_duration - elapsed)
            
            if remaining > 0:
                self.get_logger().info(f'Opening cache box: {remaining} seconds', throttle_duration_sec=1.0)
            else:
                # Countdown complete - SUCCESS
                self.get_logger().info('Opening cache box: 0 seconds')
                self.get_logger().info('Cache box opened successfully!')
                self.state = 'DONE'
                self.active = False
                self.status_pub.publish(String(data="SUCCESS"))
                self.get_logger().info(f'Mission complete for {self.target_color} cone!')
            return
        
        # --- STATE: DONE ---
        if self.state == 'DONE':
            return

        # --- STATE: FOLLOWING (Normal cone following behavior) ---
        # 3.2: Filter Candidates
        # Look through all detections and find the ones that match our target color.
        target_cone = None
        # Note: self.latest_detections is already filtered for valid depth/center in callback
        candidates = [d for d in self.latest_detections if d.get('color', '').lower() == self.target_color]
        
        # =============================================================================================
        # STAGE 4: SEARCH BEHAVIOR (No Cone Found)
        # =============================================================================================
        if not candidates:
            # If we don't see the cone, we spin slowly to look around.
            # This logic is retained from the original node as it is robust.
            self.get_logger().info("Searching for cone...", throttle_duration_sec=2)
            twist = Twist()
            twist.angular.z = 0.5  # Spin speed (positive = left/CCW)
            self.velocity_pub.publish(twist)
            return

        # =============================================================================================
        # STAGE 5: TARGET SELECTION (Cone Found)
        # =============================================================================================
        
        # Pick the closest cone (Smallest Depth). 
        # This effectively selects the "dominant" cone just like the Area logic of the old script.
        # We also treat depth < 0.1 as invalid (very close/noise) and push it to end
        candidates.sort(key=lambda x: x['depth_m'] if x['depth_m'] > 0.1 else 999.0)
        target_cone = candidates[0] # The winner is the first one in the sorted list

        # =============================================================================================
        # STAGE 6: CONTROL LOGIC (Calculate Movement) - MERGED LOGIC
        # =============================================================================================

        center_x = target_cone['center'][0]
        img_center = self.image_width / 2.0
        
        # 1. Calculate Error (Pixel based)
        # Logic: error = center - cx
        # If cone is to the Left (e.g. 100), error = 640 - 100 = 540 (Positive) -> Turn Left (Positive Z)
        error_px = img_center - center_x
        
        # 2. Scale Error
        # The original logic used hardcoded values for 640x480 resolution (center 320).
        # We normalize our error to match that scale so the PID constants work as intended.
        scale_factor = 640.0 / self.image_width 
        scaled_error = error_px * scale_factor

        # 3. Calculate Angular Velocity
        # Formula: angular = error * 0.2 / 80
        angular_z = scaled_error * 0.2 / 80.0
        
        # 4. Check Depth & Success
        depth = target_cone.get('depth_m')
        
        if depth and depth > 0.1 and depth < self.stop_distance:
             # =========================================================================================
             # STAGE 7: GOAL REACHED
             # =========================================================================================
             self.stop_rover()
             self.get_logger().info(f'Reached {self.target_color} cone! (Dist: {depth:.2f}m)')
             
             if self.goal_type == 'dropoff':
                 # DROPOFF: Do 180° rotation + cache box countdown
                 self.get_logger().info('Dropoff detected - Starting 180° rotation...')
                 self.state = 'ROTATING'
                 # Calculate target heading (current + 180 degrees, normalized to 0-360)
                 if self.current_heading is not None:
                     self.target_heading = self.normalize_heading(self.current_heading + 180.0)
                     self.get_logger().info(f'Current heading: {self.current_heading:.1f}° -> Target: {self.target_heading:.1f}°')
                 else:
                     # Fallback: estimate target heading once we get compass data
                     self.target_heading = None
                     self.get_logger().warn('No compass data yet, will calculate target once available')
             else:
                 # PICKUP: Immediate success, no rotation needed
                 self.get_logger().info(f'Pickup complete at {self.target_color} cone!')
                 self.state = 'DONE'
                 self.active = False
                 self.status_pub.publish(String(data="SUCCESS"))
             return

        # 5. Movement Decision (Yaw Threshold)
        twist = Twist()
        # Scale the threshold (40px on 640 width) to current resolution
        start_drive_threshold = self.yaw_threshold / scale_factor
        
        if abs(error_px) > start_drive_threshold:
             # Logic: "Align First"
             # Error is too high, so we ONLY turn. No forward movement.
             twist.linear.x = 0.0          # STOP forward motion
             twist.angular.z = angular_z   # TURN only
        else:
             # Logic: "Move Forward"
             # We are aligned enough. Drive forward.
             twist.linear.x = self.linear_vel
             #twist.angular.z = angular_z   # Keep fine corrections while driving
             
        # =============================================================================================
        # STAGE 8: EXECUTION
        # =============================================================================================
        self.velocity_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ConeFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    