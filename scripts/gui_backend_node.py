#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
from ament_index_python.packages import get_package_share_directory

class GuiBackend(Node):
    def __init__(self):
        super().__init__('gui_backend')
        
        # Subscribe to system commands from the Web GUI
        self.sys_sub = self.create_subscription(String, '/sys/command', self.command_callback, 10)
        
        # Point to the current scripts directory
        # Since this node is installed to lib/pkg/node.py, we need to find the share/pkg/scripts or similar if installed.
        # However, scripts/ is source. To work in both dev and install, usually we install scripts to lib/pkg/scripts too.
        # Our CMakeLists installs scripts to lib/project_name/
        # So we can look in the same directory as this file.
        
        # self.script_dir = os.path.dirname(os.path.abspath(__file__))
        # Update: We decided to point to the source path for now to satisfy "verify functionality without changing files" initially,
        # but now we are migrating. Let's assume standard ROS usage:
        # We installed the scripts to lib/${PROJECT_NAME} in CMakeLists already (via PROGRAMS)
        # Wait, CMakeLists listed:
        # install(PROGRAMS scripts/... DESTINATION lib/${PROJECT_NAME})
        # So if we add start_drive.sh etc to that list, they will be in the same dir as this node.
        
        # For now, keeping the source path as defined in the plan to be safe with the user's workspace structure
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Subscribe to log requests
        self.log_sub = self.create_subscription(String, '/gui/log_request', self.log_callback, 10)
        
        # Robust Path Resolution for mission_plan.txt
        # Check relative path first (Dev/Source Mode) to ensure user edits are synced
        # 1. Check relative to script (works if symlink-install or running from src)
        relative_path = os.path.abspath(os.path.join(self.script_dir, '../../config/mission_plan.txt'))
        
        # 2. Check known source location on Orin (rado_control_3 directly in ros2_ws/src/)
        home_dir = os.path.expanduser('~')
        source_path = os.path.join(home_dir, 'ros2_ws/src/rado_control_3/config/mission_plan.txt')
        
        if os.path.exists(relative_path):
            self.mission_file = relative_path
            self.get_logger().info(f"Dev Mode: Using relative mission_plan at {self.mission_file}")
        elif os.path.exists(source_path):
            self.mission_file = source_path
            self.get_logger().info(f"Dev Mode: Found source mission_plan at {self.mission_file}")
        else:
            try:
                share_dir = get_package_share_directory('rado_control_3')
                self.mission_file = os.path.join(share_dir, 'config', 'mission_plan.txt')
                self.get_logger().warn(f"Source file not found. Using install/share: {self.mission_file}")
            except Exception as e:
                self.get_logger().warn(f"Could not resolve share dir: {e}. Fallback to relative.")
                self.mission_file = relative_path
        
        self.get_logger().info(f"GUI Backend Node Started. Listening on /sys/command. Logging to: {self.mission_file}")

    def command_callback(self, msg):
        cmd = msg.data
        self.get_logger().info(f"Received command: {cmd}")
        
        if cmd == 'init_drive':
            self.run_script('start_drive.sh')
        elif cmd == 'init_servo':
            self.run_script('servo.sh')
        elif cmd == 'init_ld':
            self.run_script('start_ld.sh')
        elif cmd == 'init_arm':
            self.run_script('start_arm.sh')
        elif cmd == 'manual_mode' or cmd == 'MANUAL':
            self.run_script('switch_mode.sh', ['thrustmaster'])
        elif cmd == 'auto_mode' or cmd == 'PROCEED':
            self.run_script('switch_mode.sh', ['keyboard'])
        elif cmd == 'restart_mavros':
            self.get_logger().warn("Restart MAVROS not fully implemented in backend yet.")
        elif cmd == 'init_mission':
            self.run_script('start_mission.sh')

    def log_callback(self, msg):
        try:
            # Format: Type|Color|Lat|Lon (e.g., pickup|RED|12.345|67.890)
            parts = msg.data.split('|')
            if len(parts) >= 4:
                raw_type, raw_color, lat, lon = parts[0], parts[1], parts[2], parts[3]
                
                # Type is already 'pickup' or 'dropoff' from the new dropdown
                obj_type = raw_type.lower().strip()
                color = raw_color.lower().strip()
                
                # CSV Format: type,color,lat,lon
                line = f"{obj_type},{color},{lat},{lon}\n"
                
                # Append to file
                with open(self.mission_file, "a") as f:
                    f.write(line)
                
                self.get_logger().info(f"Logged to Mission Plan: {line.strip()} -> {self.mission_file}")
            else:
                self.get_logger().warn(f"Invalid log format: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Failed to write log: {e}")

    def run_script(self, script_name, args=[]):
        script_path = os.path.join(self.script_dir, script_name)
        
        if not os.path.exists(script_path):
            self.get_logger().error(f"Script not found: {script_path}")
            return

        try:
            # We execute it using subprocess
            cmd = [script_path] + args
            self.get_logger().info(f"Executing: {' '.join(cmd)}")
            
            # Pass environment with DISPLAY set for x-terminal-emulator
            env = os.environ.copy()
            if 'DISPLAY' not in env:
                env['DISPLAY'] = ':0'
            
            # Use Popen to run it with proper environment
            subprocess.Popen(cmd, cwd=self.script_dir, env=env)
            
        except Exception as e:
            self.get_logger().error(f"Failed to run script: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GuiBackend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
