import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_name = 'rado_control_3'

    # -------------------------
    # Web GUI / Remote Interface
    # -------------------------

    web_video = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen'
    )

    # -------------------------
    # Mission / Control Nodes
    # -------------------------

    state_manager = Node(
        package=pkg_name,
        executable='state_manager_node.py',
        name='state_manager',
        output='screen'
    )

    coordinate_follower = Node(
        package=pkg_name,
        executable='coordinate_follower_node.py',
        name='coordinate_follower',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'map_frame': 'map'},
            {'gps_origin_lat': 0.0},   # set as needed
            {'gps_origin_lon': 0.0}
        ]
    )


    return LaunchDescription([
        # Web GUI
        web_video,

        # Mission stack
        state_manager,
        coordinate_follower,
    ])

    
