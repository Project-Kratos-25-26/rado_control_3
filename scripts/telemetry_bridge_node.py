#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import json
import os
import time
import math

class TelemetryBridge(Node):
    def __init__(self):
        super().__init__('telemetry_bridge')
        
        # Joy0 = Thrustmaster (Drive)
        self.joy_sub = self.create_subscription(Joy, '/joy0', self.joy_callback, 10)
        # Joy = PS5 (LD)
        self.joy_ps5_sub = self.create_subscription(Joy, '/joy', self.joy_ps5_callback, 10)
        
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # Adjust data dir path since we are now in scripts/ or installed lib/
        # We want to match where the other scripts write data.
        # existing scripts (start_drive.sh) write to: $(dirname)/.data
        # If installed, that might be tricky. But let's assume relative to source for dev or absolute home path if needed.
        # For safety and consistency with web access, let's use the package share or sticking to the consistent .data 
        # path used by start_drive.sh which is relative to the script location.
        # In the context of "ros2 run", __file__ is in install/lib... 
        # The user's snippet for start_drive.sh creates .data relative to itself.
        # If we run this node via ros2 run, __file__ is the executable.
        # Let's try to find a common ground.
        # Or... since the web gui reads from .data, we need to know WHERE that is.
        # Drive GUI had .data inside drive_gui/.
        # Migration: We moved scripts to scripts/. So .data will be scripts/.data
        
        # Assuming self.gui_dir is defined elsewhere or will be added.
        # For now, let's define a placeholder for self.gui_dir to make the code syntactically correct.
        # In a real scenario, self.gui_dir would likely be determined based on package paths or environment variables.
        # For the purpose of this edit, we'll make a reasonable guess or placeholder.
        # Given the original path was relative to __file__, let's assume gui_dir is the parent of the script's directory.
        self.gui_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

        self.data_dir = os.path.join(self.gui_dir, 'data')
        os.makedirs(self.data_dir, exist_ok=True)
        self.json_path = os.path.join(self.data_dir, 'telemetry.json')
        self.microros_path = os.path.join(self.data_dir, 'microros.txt')
        # The user's snippet also included this line, moving it from write_data to __init__
        self.ping_path = os.path.join(self.data_dir, 'ping_status.json')


        self.joy_data = [0.0, 0.0, 0.0] # axes 0,1,2
        self.joy_ps5_data = {"axes": [], "buttons": []}
        self.vel_data = {"linear_x": 0.0, "angular_z": 0.0}
        
        # Local Drive Logic State
        self.rover_x = 0.0
        self.rover_z = 0.0
        self.wheel_speeds = [0.0] * 6

        # Write at 10Hz
        self.timer = self.create_timer(0.1, self.write_data)
        
        self.get_logger().info(f"Telemetry Bridge Started. Data Dir: {self.data_dir}")

    def joy_callback(self, msg):
        # Update raw joy storage for Thrustmaster
        if len(msg.axes) >= 3:
            self.joy_data = list(msg.axes[:3])
            
            # Run Local Logic
            if abs(msg.axes[1]) > 0.1 or abs(msg.axes[0]) > 0.1:
                self.teleop(msg.axes[1], msg.axes[0], msg.axes[2])
            else:
                self.rover_x = 0
                self.rover_z = 0
                self.wheel_speeds = [0.0]*6
                self.velx_disp = 0.0
                self.velz_disp = 0.0

    def joy_ps5_callback(self, msg):
        # Update raw joy storage for PS5
        self.joy_ps5_data = {
            "axes": list(msg.axes),
            "buttons": list(msg.buttons)
        }

    def cmd_callback(self, msg):
        self.vel_data["linear_x"] = msg.linear.x
        self.vel_data["angular_z"] = msg.angular.z

    # --- User Provided Logic ---
    def teleop(self, linear, rotational, speed):
        # speed is -1 to 1. (speed+1)/2 maps to 0 to 1.
        self.rover_x = (linear*((speed+1)/2) + rotational*((speed+1)/2))*100
        self.rover_z = (linear*((speed+1)/2) - rotational*((speed+1)/2))*100

        # Calculate magnitude of the vector
        magnitude = math.sqrt(self.rover_x**2 + self.rover_z**2)

        # Map to circular space if magnitude exceeds 100
        if magnitude > 100:
            scale_factor = 100 / magnitude
            self.rover_x *= scale_factor
            self.rover_z *= scale_factor
            
        # Calculate wheel speeds immediately
        self.move()

    def move(self):
        # User logic scaling
        velx = self.rover_x * 0.7
        velz = self.rover_z * 0.7

        self.velx_disp = self.rover_x
        self.velz_disp = self.rover_z

        right_wheel_front = 0
        left_wheel_front = 0
        right_wheel_mid = 0
        left_wheel_mid = 0
        right_wheel_back = 0
        left_wheel_back = 0

        if (velx >= 0 and velz >= 0):
            left_wheel_front = (((velx+velz)/2+(velx-velz)/2)*100.0)/151.0
            right_wheel_front = (((velx+velz)/2-(velx-velz)/2)*100.0)/144.0
            right_wheel_mid = (((velx+velz)/2+(velx-velz)/2)*100.0)/147.5
            left_wheel_mid = (((velx+velz)/2-(velx-velz)/2)*100.0)/158.0
            right_wheel_back = (((velx+velz)/2+(velx-velz)/2)*100.0)/76.0
            left_wheel_back = (((velx+velz)/2-(velx-velz)/2)*100.0)/164.0
        
        elif (velx <= 0 and velz <= 0):
            right_wheel_front = ((velx+velz)/2+(velx-velz)/2) * (100.0/148.0)
            left_wheel_front = ((velx+velz)/2-(velx-velz)/2) * (100.0/147.3)
            right_wheel_mid = ((velx+velz)/2+(velx-velz)/2) * (100.0/151.4)
            left_wheel_mid = ((velx+velz)/2-(velx-velz)/2) * (100.0/152.35)
            right_wheel_back = ((velx+velz)/2+(velx-velz)/2) * (100.0/86.0)
            left_wheel_back = ((velx+velz)/2-(velx-velz)/2) * (100.0/149.2)
        
        elif (velx >= 0 and velz <= 0):
            right_wheel_front = ((velx+velz)/2+(velx-velz)/2) * (100.0/147.3)
            left_wheel_front = ((velx+velz)/2-(velx-velz)/2) * (100.0/144.0)
            right_wheel_mid = ((velx+velz)/2+(velx-velz)/2) * (60.0/147.5)
            left_wheel_mid = ((velx+velz)/2-(velx-velz)/2) * (60.0/152.35)
            right_wheel_back = ((velx+velz)/2+(velx-velz)/2) * (100.0/76.0)
            left_wheel_back = ((velx+velz)/2-(velx-velz)/2) * (100.0/149.2)
        
        elif (velx <= 0 and velz >= 0):
            right_wheel_front = ((velx+velz)/2+(velx-velz)/2) * (100.0/151.0)
            left_wheel_front = ((velx+velz)/2-(velx-velz)/2) * (100.0/148.0)
            right_wheel_mid = ((velx+velz)/2+(velx-velz)/2) * (60.0/151.4)
            left_wheel_mid = ((velx+velz)/2-(velx-velz)/2) * (60.0/158.0)
            right_wheel_back = ((velx+velz)/2+(velx-velz)/2) * (100.0/86.0)
            left_wheel_back = ((velx+velz)/2-(velx-velz)/2) * (100.0/164.0)
        
        else: 
            pass

        wheel_speeds = [
            right_wheel_front, left_wheel_front,
            right_wheel_mid, left_wheel_mid,
            right_wheel_back, left_wheel_back,
        ]

        # Limit Check
        max_speed = max(abs(s) for s in wheel_speeds) if wheel_speeds else 0
        if max_speed > 70:
            scale_factor = 70 / max_speed
            wheel_speeds = [s * scale_factor for s in wheel_speeds]
        
        self.wheel_speeds = wheel_speeds

    def write_data(self):
        microros_status = "UNKNOWN"
        if os.path.exists(self.microros_path):
            try:
                with open(self.microros_path, 'r') as f:
                    microros_status = f.read().strip()
            except:
                pass

        # Read Ping Status (from start_server.sh loop)
        ping_raspi = "OFFLINE"
        ping_path = os.path.join(self.data_dir, 'ping_status.txt')
        if os.path.exists(ping_path):
            try:
                with open(ping_path, 'r') as f:
                    ping_raspi = f.read().strip()
            except:
                pass
        
        data = {
            "joy": self.joy_data,
            "joy_ps5": self.joy_ps5_data, # Add PS5 data
            "vel": self.vel_data,
            "microros": microros_status,
            "ping_raspi": ping_raspi, # Add Ping Data
            "local_calc": {
                "rover_x": self.rover_x,
                "rover_z": self.rover_z,
                "velx": getattr(self, 'velx_disp', 0.0),
                "velz": getattr(self, 'velz_disp', 0.0),
                "wheels": self.wheel_speeds
            },
            "timestamp": time.time()
        }
        
        tmp_path = self.json_path + ".tmp"
        try:
            with open(tmp_path, 'w') as f:
                json.dump(data, f)
            os.rename(tmp_path, self.json_path)
        except Exception as e:
            pass # ignore conflicts

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
