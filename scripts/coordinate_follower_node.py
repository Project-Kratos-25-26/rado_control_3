#!/usr/bin/env python3
"""
Mission Manager with Nav2 Integration
Converts GPS waypoints to map coordinates and sends goals to Nav2
Falls back to cone following for final approach
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import os
import time
import math
from pathlib import Path


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # Parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('gps_origin_lat', 0.0)  # Set your map origin (0.0 = auto-detect)
        self.declare_parameter('gps_origin_lon', 0.0)  # Set your map origin (0.0 = auto-detect)
        self.declare_parameter('gps_stabilization_samples', 10)  # Number of GPS samples for stabilization
        self.declare_parameter('gps_stabilization_threshold', 0.00001)  # Max std dev for stabilization (degrees)
        
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.gps_origin_lat = self.get_parameter('gps_origin_lat').value
        self.gps_origin_lon = self.get_parameter('gps_origin_lon').value
        self.gps_stabilization_samples = self.get_parameter('gps_stabilization_samples').value
        self.gps_stabilization_threshold = self.get_parameter('gps_stabilization_threshold').value
        
        # GPS origin initialization state
        self.gps_origin_initialized = False
        self.gps_init_samples_lat = []
        self.gps_init_samples_lon = []
        
        # Check if origin was provided via parameters
        if self.gps_origin_lat != 0.0 or self.gps_origin_lon != 0.0:
            self.gps_origin_initialized = True
            self.get_logger().info(
                f'GPS origin set from parameters: ({self.gps_origin_lat:.6f}, {self.gps_origin_lon:.6f})'
            )

        # State machine
        self.internal_state = 'IDLE'
        self.mission_goals = []
        self.current_goal = None
        self.current_goal_index = -1
        self.queue_index = -1  # For GUI queue tracking
        
        # Mission queue from GUI (stores all waypoints for this PROCEED press)
        self.mission_queue = []  # List of goals to process
        self.mission_queue_index = 0  # Current position in queue
        
        # Navigation State
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_heading = 0.0
        
        # --- TUNING PARAMETERS ---
        self.cone_switch_distance = 0.5  # Meters - when to switch from Nav2 to cone following
        self.nav2_goal_tolerance = 2.0   # Nav2 goal tolerance
        self.nav2_timeout = 300.0        # Nav2 navigation timeout (seconds)
        # -------------------------

        # Path Setup - use rado_control_3 directly (Orin structure)
        home = str(Path.home())
        self.mission_file_path = os.path.join(
            home, 'ros2_ws/src/rado_control_3/config/mission_plan.txt'
        )

        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.cone_trigger_pub = self.create_publisher(
            String, '/auto/cone_follow/trigger', 10
        )
        # Publisher to notify GUI of mission progress
        self.mission_status_pub = self.create_publisher(
            String, '/mission/status', 10
        )

        # Subscribers
        self.create_subscription(
            String, '/rover_state', self.state_callback, 10
        )
        self.create_subscription(
            String, '/gcs/command', self.gcs_command_callback, 10
        )
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, 10
        )
        self.create_subscription(
            String, '/auto/cone_follow/status', self.cone_status_callback, 10
        )
        # Subscribe to GUI goal selection
        self.create_subscription(
            String, '/mission/set_goal', self.set_goal_callback, 10
        )

        # Nav2 Action Client
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav2_goal_handle = None

        # Timer
        self.timer = self.create_timer(0.2, self.control_loop)
        
        # Load mission and wait for Nav2
        self.load_mission()
        self.internal_state = 'WAITING_FOR_PROCEED'
        
        self.get_logger().info('Mission Manager (Nav2 Integrated) Initialized.')
        self.get_logger().info(f'Loaded {len(self.mission_goals)} mission goals.')
        
        # Wait for Nav2 action server
        self.create_timer(1.0, self.check_nav2_ready)

    def check_nav2_ready(self):
        """Check if Nav2 action server is available"""
        if not self.nav2_client.server_is_ready():
            self.get_logger().info('Waiting for Nav2 action server...', throttle_duration_sec=5)
        else:
            self.get_logger().info('Nav2 action server is ready!')

    def load_mission(self):
        """Load mission waypoints from file"""
        self.mission_goals = []
        try:
            if os.path.exists(self.mission_file_path):
                with open(self.mission_file_path, 'r') as f:
                    for line in f:
                        parts = line.strip().split(',')
                        if len(parts) == 4:
                            try:
                                goal = {
                                    'type': parts[0].lower().strip(),
                                    'color': parts[1].lower().strip(),
                                    'lat': float(parts[2]),
                                    'lon': float(parts[3])
                                }
                                self.mission_goals.append(goal)
                            except ValueError as e:
                                self.get_logger().warn(f'Invalid line in mission file: {line}')
            else:
                self.get_logger().error(f'Mission file not found: {self.mission_file_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load mission: {e}')

    def gcs_command_callback(self, msg):
        """Handle commands from ground control station"""
        command = msg.data.upper().strip()
        
        if command == 'PROCEED':
            if self.internal_state == 'WAITING_FOR_PROCEED':
                # Start processing the mission queue from the beginning
                self.mission_queue_index = 0
                
                # Clean up the queue (remove None entries)
                self.mission_queue = [g for g in self.mission_queue if g is not None]
                
                if len(self.mission_queue) > 0:
                    self.current_goal = self.mission_queue[0]
                    self.internal_state = 'WAITING_FOR_AUTONOMOUS'
                    self.get_logger().info(
                        f"Starting mission queue ({len(self.mission_queue)} waypoints). "
                        f"First: {self.current_goal['type']} {self.current_goal['color']} "
                        f"at ({self.current_goal['lat']:.6f}, {self.current_goal['lon']:.6f})"
                    )
                elif self.current_goal:
                    # Fallback to single goal mode
                    self.mission_queue = [self.current_goal]
                    self.mission_queue_index = 0
                    self.internal_state = 'WAITING_FOR_AUTONOMOUS'
                    self.get_logger().info(
                        f"Proceeding to goal: {self.current_goal['type']} "
                        f"{self.current_goal['color']} at ({self.current_goal['lat']:.6f}, "
                        f"{self.current_goal['lon']:.6f})"
                    )
                else:
                    self.get_logger().warn("No waypoints in queue. Add waypoints first.")
        elif command == 'MANUAL':
            self.internal_state = 'WAITING_FOR_PROCEED'
            self.cancel_nav2_goal()
            self.cone_trigger_pub.publish(String(data="STOP"))
            # Clear the queue on manual stop
            self.mission_queue = []
            self.mission_queue_index = 0
            self.get_logger().info("Mission stopped. Queue cleared.")
        elif command == 'CANCEL':
            self.cancel_nav2_goal()
            self.internal_state = 'WAITING_FOR_PROCEED'

    def set_goal_callback(self, msg):
        """Handle goal selection from GUI - adds to mission queue"""
        try:
            parts = msg.data.split('|')
            
            # New queue format: QUEUE|index|type|color|lat|lon
            if len(parts) >= 6 and parts[0] == 'QUEUE':
                queue_idx = int(parts[1])
                goal_type = parts[2].lower().strip()
                color = parts[3].lower().strip()
                lat = float(parts[4])
                lon = float(parts[5])
                
                goal = {
                    'type': goal_type,
                    'color': color,
                    'lat': lat,
                    'lon': lon,
                    'queue_index': queue_idx
                }
                
                # Add to mission queue (replace if same index exists, or append)
                while len(self.mission_queue) <= queue_idx:
                    self.mission_queue.append(None)
                self.mission_queue[queue_idx] = goal
                
                # Set as current goal for immediate reference
                self.current_goal = goal
                
                self.get_logger().info(
                    f"Queue Goal #{queue_idx + 1} Added: {goal_type} {color} at ({lat:.6f}, {lon:.6f}). Queue size: {len([g for g in self.mission_queue if g])}"
                )
                self.internal_state = 'WAITING_FOR_PROCEED'
            
            # Legacy format: GOAL|type|color|lat|lon
            elif len(parts) >= 5 and parts[0] == 'GOAL':
                goal_type = parts[1].lower().strip()
                color = parts[2].lower().strip()
                lat = float(parts[3])
                lon = float(parts[4])
                
                self.current_goal = {
                    'type': goal_type,
                    'color': color,
                    'lat': lat,
                    'lon': lon,
                    'queue_index': -1
                }
                
                # For legacy, just set as single goal
                self.mission_queue = [self.current_goal]
                self.mission_queue_index = 0
                
                self.get_logger().info(
                    f"GUI Goal Set: {goal_type} {color} at ({lat:.6f}, {lon:.6f})"
                )
                self.internal_state = 'WAITING_FOR_PROCEED'
            else:
                self.get_logger().warn(f"Invalid goal format: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse goal: {e}")

    def select_next_goal(self):
        """Select the next goal in mission sequence"""
        next_goal = None
        
        # If we just completed a pickup, find matching dropoff
        if self.current_goal and self.current_goal['type'] == 'pickup':
            color = self.current_goal['color']
            for g in self.mission_goals:
                if g['type'] == 'dropoff' and g['color'] == color:
                    next_goal = g
                    break
        else:
            # Find next pickup
            start_search = self.current_goal_index + 1
            for i in range(start_search, len(self.mission_goals)):
                if self.mission_goals[i]['type'] == 'pickup':
                    next_goal = self.mission_goals[i]
                    self.current_goal_index = i
                    break
        
        if next_goal:
            self.current_goal = next_goal
            self.get_logger().info(
                f"Selected Goal: {next_goal['type']} {next_goal['color']} "
                f"at ({next_goal['lat']:.6f}, {next_goal['lon']:.6f})"
            )
            self.internal_state = 'WAITING_FOR_AUTONOMOUS'
        else:
            self.get_logger().info("No more goals found. Mission complete!")
            self.internal_state = 'IDLE'

    def state_callback(self, msg):
        """Handle rover state changes"""
        state = msg.data.upper().strip()
        
        if state == 'AUTONOMOUS' and self.internal_state == 'WAITING_FOR_AUTONOMOUS':
            self.internal_state = 'NAV2_NAVIGATING'
            self.send_nav2_goal()
        elif state == 'MANUAL':
            if self.internal_state in ['NAV2_NAVIGATING', 'CONE_NAVIGATING']:
                self.internal_state = 'WAITING_FOR_PROCEED'
                self.cancel_nav2_goal()
                self.cone_trigger_pub.publish(String(data="STOP"))

    def gps_callback(self, msg):
        """Update current GPS position and initialize origin if needed"""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        
        # Auto-initialize GPS origin if not set
        if not self.gps_origin_initialized:
            self._collect_gps_for_origin(msg.latitude, msg.longitude)
    
    def _collect_gps_for_origin(self, lat, lon):
        """Collect GPS samples and set origin when stabilized"""
        # Skip invalid readings
        if lat == 0.0 and lon == 0.0:
            return
        
        # Collect samples
        self.gps_init_samples_lat.append(lat)
        self.gps_init_samples_lon.append(lon)
        
        num_samples = len(self.gps_init_samples_lat)
        
        # Wait for enough samples
        if num_samples < self.gps_stabilization_samples:
            self.get_logger().info(
                f'Collecting GPS for origin: {num_samples}/{self.gps_stabilization_samples} samples',
                throttle_duration_sec=2
            )
            return
        
        # Keep only recent samples
        if num_samples > self.gps_stabilization_samples:
            self.gps_init_samples_lat = self.gps_init_samples_lat[-self.gps_stabilization_samples:]
            self.gps_init_samples_lon = self.gps_init_samples_lon[-self.gps_stabilization_samples:]
        
        # Calculate standard deviation to check stability
        lat_mean = sum(self.gps_init_samples_lat) / len(self.gps_init_samples_lat)
        lon_mean = sum(self.gps_init_samples_lon) / len(self.gps_init_samples_lon)
        
        lat_variance = sum((x - lat_mean) ** 2 for x in self.gps_init_samples_lat) / len(self.gps_init_samples_lat)
        lon_variance = sum((x - lon_mean) ** 2 for x in self.gps_init_samples_lon) / len(self.gps_init_samples_lon)
        
        lat_std = math.sqrt(lat_variance)
        lon_std = math.sqrt(lon_variance)
        
        # Check if GPS is stable enough
        if lat_std <= self.gps_stabilization_threshold and lon_std <= self.gps_stabilization_threshold:
            self.gps_origin_lat = lat_mean
            self.gps_origin_lon = lon_mean
            self.gps_origin_initialized = True
            
            self.get_logger().info(
                f'GPS origin auto-set from stabilized readings: '
                f'({self.gps_origin_lat:.6f}, {self.gps_origin_lon:.6f}) '
                f'[std: lat={lat_std:.8f}, lon={lon_std:.8f}]'
            )
            
            # Clear samples to free memory
            self.gps_init_samples_lat = []
            self.gps_init_samples_lon = []
        else:
            self.get_logger().info(
                f'GPS not yet stable. std: lat={lat_std:.8f}, lon={lon_std:.8f} '
                f'(threshold: {self.gps_stabilization_threshold})',
                throttle_duration_sec=2
            )

    def cone_status_callback(self, msg):
        """Handle cone following status updates"""
        if self.internal_state == 'CONE_NAVIGATING' and msg.data == 'SUCCESS':
            self.handle_arrival()

    def gps_to_map_coords(self, lat, lon):
        """
        Convert GPS coordinates to map coordinates
        Uses simple equirectangular projection relative to origin
        For better accuracy, consider using robot_localization or a proper UTM converter
        """
        # If no origin set, log warning
        if not self.gps_origin_initialized:
            self.get_logger().warn(
                'GPS origin not yet initialized! Waiting for GPS to stabilize...',
                throttle_duration_sec=5
            )
            return 0.0, 0.0
        
        # Earth radius in meters
        R = 6371000.0
        
        # Convert to radians
        lat_rad = math.radians(lat)
        origin_lat_rad = math.radians(self.gps_origin_lat)
        
        # Calculate differences
        dlat = lat - self.gps_origin_lat
        dlon = lon - self.gps_origin_lon
        
        # Convert to meters (equirectangular approximation)
        x = -1.0 * (R * math.radians(dlon) * math.cos(origin_lat_rad))
        y = -1.0 * (R * math.radians(dlat))
        
        return x, y

    def send_nav2_goal(self):
        """Send navigation goal to Nav2"""
        if not self.nav2_client.server_is_ready():
            self.get_logger().error('Nav2 action server not available!')
            self.internal_state = 'WAITING_FOR_PROCEED'
            return
        
        # Check if GPS origin is initialized
        if not self.gps_origin_initialized:
            self.get_logger().warn('Cannot navigate: GPS origin not yet initialized. Waiting for GPS to stabilize...')
            self.internal_state = 'WAITING_FOR_AUTONOMOUS'
            return

        # Convert GPS to map coordinates
        x, y = self.gps_to_map_coords(
            self.current_goal['lat'],
            self.current_goal['lon']
        )

        # Create Nav2 goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Orientation (quaternion for yaw=0, can be calculated based on approach direction)
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Sending Nav2 goal to map coords: ({x:.2f}, {y:.2f})'
        )

        # Send goal
        send_goal_future = self.nav2_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav2_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav2_goal_response_callback)

    def nav2_goal_response_callback(self, future):
        """Handle Nav2 goal acceptance/rejection"""
        self.nav2_goal_handle = future.result()
        
        if not self.nav2_goal_handle.accepted:
            self.get_logger().error('Nav2 goal was rejected!')
            self.internal_state = 'WAITING_FOR_PROCEED'
            return

        self.get_logger().info('Nav2 goal accepted, navigating...')
        
        # Get result asynchronously
        result_future = self.nav2_goal_handle.get_result_async()
        result_future.add_done_callback(self.nav2_result_callback)

    def nav2_feedback_callback(self, feedback_msg):
        """Handle Nav2 navigation feedback"""
        feedback = feedback_msg.feedback
        
        # Calculate distance to goal
        if self.current_goal:
            dist, _ = self.get_distance_bearing(
                self.current_lat, self.current_lon,
                self.current_goal['lat'], self.current_goal['lon']
            )
            
            self.get_logger().info(
                f'Nav2 feedback - Distance remaining: {dist:.2f}m',
                throttle_duration_sec=2
            )
            
            # Switch to cone following when close enough
            if dist < self.cone_switch_distance:
                self.get_logger().info(
                    f'Within {dist:.2f}m. Canceling Nav2, switching to CONE FOLLOW.'
                )
                self.cancel_nav2_goal()
                self.internal_state = 'CONE_NAVIGATING'
                # Send color|type so cone follower knows if this is pickup or dropoff
                trigger_msg = f"{self.current_goal['color']}|{self.current_goal['type']}"
                self.cone_trigger_pub.publish(String(data=trigger_msg))

    def nav2_result_callback(self, future):
        """Handle Nav2 navigation result"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Nav2 navigation succeeded!')
            # Switch to cone following for final approach
            self.internal_state = 'CONE_NAVIGATING'
            # Send color|type so cone follower knows if this is pickup or dropoff
            trigger_msg = f"{self.current_goal['color']}|{self.current_goal['type']}"
            self.cone_trigger_pub.publish(String(data=trigger_msg))
        elif status == 5:  # CANCELED
            self.get_logger().info('Nav2 navigation was canceled')
        else:
            self.get_logger().error(f'Nav2 navigation failed with status: {status}')
            self.internal_state = 'WAITING_FOR_PROCEED'

    def cancel_nav2_goal(self):
        """Cancel current Nav2 navigation goal"""
        if self.nav2_goal_handle is not None:
            self.get_logger().info('Canceling Nav2 goal...')
            cancel_future = self.nav2_goal_handle.cancel_goal_async()
            self.nav2_goal_handle = None

    def handle_arrival(self):
        """Handle arrival at goal location - auto-proceed to next waypoint"""
        self.get_logger().info(
            f"Arrived at {self.current_goal['type']} ({self.current_goal['color']})"
        )
        
        queue_idx = self.current_goal.get('queue_index', -1)
        
        # Notify GUI of completion
        status_msg = String()
        status_msg.data = f"COMPLETE|{queue_idx}|{self.current_goal['type']}"
        self.mission_status_pub.publish(status_msg)
        
        self.get_logger().info(f"{self.current_goal['type'].capitalize()} complete at {self.current_goal['color']}")
        
        # Auto-proceed to next waypoint in queue
        self.proceed_to_next_waypoint()

    def proceed_to_next_waypoint(self):
        """Move to the next waypoint in the mission queue, or finish if done"""
        self.mission_queue_index += 1
        
        if self.mission_queue_index < len(self.mission_queue):
            # More waypoints to go
            self.current_goal = self.mission_queue[self.mission_queue_index]
            self.get_logger().info(
                f"Auto-proceeding to next waypoint ({self.mission_queue_index + 1}/{len(self.mission_queue)}): "
                f"{self.current_goal['type']} {self.current_goal['color']} "
                f"at ({self.current_goal['lat']:.6f}, {self.current_goal['lon']:.6f})"
            )
            
            # Notify GUI of progress
            status_msg = String()
            status_msg.data = f"NAVIGATING|{self.mission_queue_index}|{self.current_goal['type']}"
            self.mission_status_pub.publish(status_msg)
            
            # Start navigation to next goal
            self.internal_state = 'NAV2_NAVIGATING'
            self.send_nav2_goal()
        else:
            # All waypoints complete!
            self.get_logger().info(
                f"=== MISSION COMPLETE === All {len(self.mission_queue)} waypoints completed!"
            )
            
            # Notify GUI
            status_msg = String()
            status_msg.data = "MISSION_COMPLETE"
            self.mission_status_pub.publish(status_msg)
            
            # Now switch to manual mode
            self.internal_state = 'WAITING_FOR_PROCEED'
            self.mission_queue = []
            self.mission_queue_index = 0
            self.current_goal = None
            self.get_logger().info("Switched to MANUAL mode. Ready for next mission.")

    def get_distance_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate distance and bearing between two GPS coordinates"""
        R = 6371000  # Earth radius in meters
        
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        
        # Haversine formula
        a = (math.sin(dphi/2)**2 + 
             math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        dist = R * c
        
        # Bearing calculation
        y = math.sin(dlambda) * math.cos(phi2)
        x = (math.cos(phi1) * math.sin(phi2) - 
             math.sin(phi1) * math.cos(phi2) * math.cos(dlambda))
        bearing = math.degrees(math.atan2(y, x))
        
        return dist, (bearing + 360) % 360

    def control_loop(self):
        """Main control loop - minimal, most logic is event-driven"""
        # The control loop is now mostly event-driven via callbacks
        # This loop can be used for periodic status checks if needed
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
