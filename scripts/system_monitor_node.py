#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Bool
from sensor_msgs.msg import NavSatFix, Image
from geometry_msgs.msg import Twist
import subprocess
import json
import time

# ==========================================
#      USER CONFIGURATION SECTION
# ==========================================

# 1. Hardware IPs to Ping (Name: IP)
PING_LIST = {
    'JETSON': '192.168.1.10',
    'RASPI':  '192.168.1.16'
}

# 2. ROS Topics to Watch (Name: [Topic, MessageType, Timeout_Seconds])
TOPIC_WATCHLIST = {
    'GPS_FIX':  ['/mavros/global_position/global', NavSatFix, 3.0],
    'CAMERA':   ['/zed/zed_node/rgb/image_rect_color', Image, 1.0],
    'MOTORS':   ['/cmd_vel', Twist, 2.0] # Checks if commands are flowing
}

# ==========================================

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        
        self.health_pub = self.create_publisher(String, '/gui/system_health', 10)
        self.error_pub = self.create_publisher(String, '/gui/error_log', 10)
        
        # Dictionary to store last message times
        self.last_msg_times = {key: 0.0 for key in TOPIC_WATCHLIST}
        self.topic_status = {key: "WAITING" for key in TOPIC_WATCHLIST}
        self.ping_status = {key: False for key in PING_LIST}

        # Dynamically create subscribers based on the list
        for name, config in TOPIC_WATCHLIST.items():
            topic, msg_type, _ = config
            # We use a lambda to capture the 'name' variable
            self.create_subscription(
                msg_type, 
                topic, 
                lambda msg, n=name: self.topic_callback(msg, n), 
                qos_profile_sensor_data
            )

        # Timer for the Watchdog (Runs every 1 second)
        self.create_timer(1.0, self.watchdog_loop)
        self.get_logger().info("System Monitor Running")

    def topic_callback(self, msg, name):
        """Called whenever a message arrives on a watched topic."""
        self.last_msg_times[name] = time.time()
        self.topic_status[name] = "OK"

    def watchdog_loop(self):
        """Checks Pings and Topic Timestamps."""
        current_time = time.time()

        # 1. Check Pings
        for name, ip in PING_LIST.items():
            try:
                # ping -c 1 -W 1 (1 packet, 1 sec timeout)
                ret = subprocess.call(['ping', '-c', '1', '-W', '1', ip], 
                                      stdout=subprocess.DEVNULL, 
                                      stderr=subprocess.DEVNULL)
                self.ping_status[name] = (ret == 0)
            except:
                self.ping_status[name] = False

        # 2. Check Topics (Heartbeat)
        for name, config in TOPIC_WATCHLIST.items():
            timeout = config[2]
            last_seen = self.last_msg_times[name]
            
            if last_seen == 0.0:
                self.topic_status[name] = "NO DATA"
            elif (current_time - last_seen) > timeout:
                self.topic_status[name] = "STALLED"
                # Optional: Publish an error alert
                # self.error_pub.publish(String(data=f"Warning: {name} topic stopped sending data!"))
            else:
                self.topic_status[name] = "OK"

        # 3. Publish Report to GUI
        report = {
            "pings": self.ping_status,
            "topics": self.topic_status
        }
        msg = String()
        msg.data = json.dumps(report)
        self.health_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()