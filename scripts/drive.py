#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Joy
import math

sub1 = ReentrantCallbackGroup()
sub2 = ReentrantCallbackGroup()

class Drive(Node):
    def __init__(self):
        super().__init__('drive_node')
        
        # Initialize variables
        self.rover_x = 0
        self.rover_z = 0

        self.one=0.0
        self.two=0.0
        self.three=0.0
        self.four=0.0
        self.five=0.0
        self.six=0.0
        self.seven=0.0
        self.eight=0.0
        self.msg=Float32MultiArray()
        self.msg.data = [0.0] * 14
        
        # Create subscription to joystick
        self.subscription = self.create_subscription(
            Joy,
            '/joy0',
            self.callback,
            10,
            callback_group=sub1
        )
        
        # Create subscription to joystick
        self.subscription1 = self.create_subscription(
            Float32MultiArray,
            '/camera_angles',
            self.callback_servo,
            10,
            callback_group=sub2
        )
        # Create publisher for rover commands
        self.values = Float32MultiArray()
        self.actualValues = [0.0] * 14  # Initialize with floats instead of integers
        self.values.data = self.actualValues
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/rover',
            10
        )
        
        # Create timer for periodic publishing
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz rate

    def timer_callback(self):
        self.actualValues = self.move()
        self.values.data = self.actualValues
        self.publisher.publish(self.values)
        
    def callback_servo(self, msg):
        # self.one = msg->.data[0]
        # self.two = msg.data[1]
        # self.three =msg.data[2]
        # self.four = msg.data[3]
        # self.five = msg.data[4]
        # self.six = msg.data[5]
        # self.seven =msg.data[6]
        # self.eight =msg.data[7]

        self.one ,self.two ,self.three ,self.four ,self.five ,self.six ,self.seven ,self.eight=msg.data

    def teleop(self, linear, rotational, speed):
        self.rover_x = (linear*((speed+1)/2) + rotational*((speed+1)/2))*100
        self.rover_z = (linear*((speed+1)/2) - rotational*((speed+1)/2))*100

        # Calculate magnitude of the vector
        magnitude = math.sqrt(self.rover_x**2 + self.rover_z**2)

        # Map to circular space if magnitude exceeds 100
        if magnitude > 100:
            scale_factor = 100 / magnitude
            self.rover_x *= scale_factor
            self.rover_z *= scale_factor

    def callback(self, msg):
        if msg.axes[1] > 0.1 or msg.axes[1] < -0.1 or msg.axes[0] > 0.1 or msg.axes[0] < -0.1:
            self.teleop(msg.axes[1], msg.axes[0], msg.axes[2])
        else:
            self.rover_x = 0
            self.rover_z = 0

    def move(self):
        velx = self.rover_x 
        velz = self.rover_z 

        print(velx, " ", velz)
        # right_wheel_front = 0.0

        left_wheel_front = 0.0
        right_wheel_front = 0.0
        right_wheel_mid = 0.0
        left_wheel_mid = 0.0
        right_wheel_back = 0.0
        left_wheel_back = 0.0

        if (velx >= 0 and velz >= 0):
            left_wheel_front = ((velx+velz)/2+(velx-velz)/2) * 0.56
            right_wheel_front = ((velx+velz)/2-(velx-velz)/2) * 0.62
            right_wheel_mid = ((velx+velz)/2+(velx-velz)/2) * 0.63
            left_wheel_mid = ((velx+velz)/2-(velx-velz)/2) * 0.55
            right_wheel_back = ((velx+velz)/2+(velx-velz)/2) * 0.64
            left_wheel_back = ((velx+velz)/2-(velx-velz)/2) * 0.59
        
        elif (velx <= 0 and velz <= 0):

            right_wheel_front = ((velx+velz)/2+(velx-velz)/2) * 0.59
            left_wheel_front = ((velx+velz)/2-(velx-velz)/2) * 0.59
            right_wheel_mid = ((velx+velz)/2+(velx-velz)/2) * 0.58
            left_wheel_mid = ((velx+velz)/2-(velx-velz)/2) * 0.59
            right_wheel_back = ((velx+velz)/2+(velx-velz)/2) * 0.63
            left_wheel_back = ((velx+velz)/2-(velx-velz)/2) * 0.61
        
        elif (velx >= 0 and velz <= 0):
            right_wheel_front = ((velx+velz)/2+(velx-velz)/2) * 0.62
            left_wheel_front = ((velx+velz)/2-(velx-velz)/2)  * 0.59
            right_wheel_mid = ((velx+velz)/2+(velx-velz)/2) * 0.63
            left_wheel_mid = ((velx+velz)/2-(velx-velz)/2)  * 0.59
            right_wheel_back = ((velx+velz)/2+(velx-velz)/2) * 0.64
            left_wheel_back = ((velx+velz)/2-(velx-velz)/2) * 0.61
         
        elif (velx <= 0 and velz >= 0):
            right_wheel_front = ((velx+velz)/2+(velx-velz)/2) * 0.59
            left_wheel_front = ((velx+velz)/2-(velx-velz)/2)  * 0.56 
            right_wheel_mid = ((velx+velz)/2+(velx-velz)/2) * 0.58
            left_wheel_mid = ((velx+velz)/2-(velx-velz)/2) *0.55
            right_wheel_back = ((velx+velz)/2+(velx-velz)/2)  * 0.63
            left_wheel_back = ((velx+velz)/2-(velx-velz)/2)  * 0.59
         
        else:  # velx == 0 and velz == 0
            right_wheel_front = 0
            left_wheel_front = 0
            right_wheel_mid = 0
            left_wheel_mid = 0
            right_wheel_back = 0
            left_wheel_back = 0

        # List of all wheel speeds
        wheel_speeds = [
            right_wheel_front,
            left_wheel_front,
            right_wheel_mid,
            left_wheel_mid,
            right_wheel_back,
            left_wheel_back,
        ]


        return right_wheel_front, left_wheel_front, right_wheel_mid, left_wheel_mid, right_wheel_back, left_wheel_back, self.one, self.two, self.three, self.four, self.five, self.six, self.seven, self.eight


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

   
    drive_node = Drive()
    executor.add_node(drive_node)
   
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        drive_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
