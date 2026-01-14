# RADO Control 3

## Installation

### Dependencies

Ensure you have the following ROS 2 packages installed:

```bash
sudo apt install ros-<distro>-rosbridge-server
sudo apt install ros-<distro>-web-video-server
sudo apt install ros-<distro>-joy
sudo apt install ros-<distro>-teleop-twist-joy
sudo apt install ros-<distro>-nav2-msgs
```

### Building

Clone the repository into your ROS 2 workspace and build:

```bash
cd ~/ros2_ws/src
git clone <repository_url>
cd ~/ros2_ws
colcon build --packages-select rado_control_3
source install/setup.bash
```

## Usage

### Launching the System

To start the entire control stack, including the web server and bridge:

```bash
ros2 launch rado_control_3 rado_mission.launch.py
```

### Web Interface

The web GUI is in the separate `rado_gui` folder (laptop only).
See the main README.md for setup instructions.


## Configuration

### Mission Plan
The mission is defined in `config/mission_plan.txt`.
**Format:** `action,color,latitude,longitude`

**Example:**
```csv
pickup,red,-0.0000008866206231913448,-0.00005354422565126319
dropoff,red,0,0
```

## ⚠️ Required Configuration Changes

Before running the package, you **must** update the following files to match your specific hardware and environment:

### 1. Set GPS Origin
**File:** `launch/rado_mission.launch.py`
Update the `gps_origin_lat` and `gps_origin_lon` parameters to match the GPS coordinates of your map's origin (0,0).
```python
    coordinate_follower = Node(
        ...
        parameters=[
            {'gps_origin_lat': 0.0},   # <--- UPDATE THIS
            {'gps_origin_lon': 0.0}    # <--- UPDATE THIS
        ]
    )
```

### 2. Mission File Path
**File:** `scripts/coordinate_follower_node.py`
The node looks for the mission plan at:
```
~/ros2_ws/src/rado_control_3/config/mission_plan.txt
```
This expands to `/home/kratos/ros2_ws/src/rado_control_3/config/mission_plan.txt` on Orin.

### 3. Configure System Monitor IPs
**File:** `scripts/system_monitor_node.py`
Update the IP addresses for the hardware you want to monitor (e.g., Jetson, Raspberry Pi, Base Station).
```python
PING_LIST = {
    'JETSON': '192.168.1.10', # <--- UPDATE IP
    'RASPI':  '192.168.1.16'  # <--- UPDATE IP
}
```

### 4. Verify Topic Names
**File:** `scripts/system_monitor_node.py`
Ensure the topics being monitored match your robot's configuration (e.g., camera namespace, GPS topic).
```python
TOPIC_WATCHLIST = {
    'GPS_FIX':  ['/mavros/global_position/global', NavSatFix, 3.0],
    'CAMERA':   ['/zed/zed_node/rgb/image_rect_color', Image, 1.0], # <--- CHECK CAMERA TOPIC
    ...
}
```

## Topics

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/Twist` | Final velocity command sent to motor controller. |
| `/rover_state` | `std_msgs/String` | Current state (MANUAL/AUTONOMOUS). |
| `/gcs/command` | `std_msgs/String` | Commands from the Web GUI (e.g., PROCEED, MANUAL). |
| `/manual/cmd_vel` | `geometry_msgs/Twist` | Velocity commands from joystick. |
| `/auto/cmd_vel` | `geometry_msgs/Twist` | Velocity commands from autonomous nodes. |
| `/gui/system_health` | `std_msgs/String` | JSON string containing system health status. |

## License

Apache-2.0
