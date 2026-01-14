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

    rosbridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            )
        )
    )

    web_video = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen'
    )

    # -------------------------
    # Mission / Control Nodes
    # -------------------------

    system_monitor = Node(
        package=pkg_name,
        executable='system_monitor_node.py',
        name='system_monitor',
        output='screen'
    )

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

    cone_follower = Node(
        package=pkg_name,
        executable='cone_follower_node.py',
        name='cone_follower',
        output='screen'
    )
    joy_node_ps5 = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'device_name': 'Sony Interactive Entertainment Wireless Controller'}]
    )

    joy_node_thrustmaster = Node(
        package='joy',
        executable='joy_node',
        name='joy_node_tm', # Distinct name
        output='screen',
        remappings=[('/joy', '/joy0')],
        parameters=[{'device_name': 'Thrustmaster T.Flight Hotas One'}]
    )

    telemetry_bridge = Node(
        package=pkg_name,
        executable='telemetry_bridge_node.py',
        name='telemetry_bridge',
        output='screen'
    )

    gui_backend = Node(
        package=pkg_name,
        executable='gui_backend_node.py',
        name='gui_backend',
        output='screen'
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
        remappings=[('/cmd_vel', '/manual/cmd_vel')]
    )


    return LaunchDescription([
        # Web GUI
        rosbridge,
        web_video,

        # Mission stack
        system_monitor,
        state_manager,
        coordinate_follower,
        cone_follower,
        joy_node_ps5,
        joy_node_thrustmaster,
        telemetry_bridge,
        gui_backend,
        teleop_node
    ])

    
