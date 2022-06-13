import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    ardusub_arg = LaunchConfiguration('ardusub')
    ardusub_arg = DeclareLaunchArgument(
        'ardusub',
        default_value='false'
    )

    mavros_url_arg = LaunchConfiguration('mavros_url')
    mavros_url_arg = DeclareLaunchArgument(
        'mavros_url',
        default_value='0.0.0.0:14550',
    )

    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={
           'ign_args': '-r room_walls.world'
        }.items(),
    )

    #TODO: Pass x, y, z, R, P and Y as parameter
    bluerov_spawn = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-world', 'walls',
            '-file', 'bluerov2_lidar',
            '-name', 'bluerov2',
            '-x', '10',
            '-y', '-6.0',
            '-z', '-19.5',
            '-Y', '-1.57']
    )

    # Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
        remappings=[('/lidar','/scan')],
        output='screen'
    )

    return LaunchDescription([
        ign_gazebo,
        bluerov_spawn,
        bridge,
        ardusub_arg,
        mavros_url_arg,
        ExecuteProcess(
            cmd=['sim_vehicle.py','-v', 'ArduSub', '-L', 'RATBeach', '--model=JSON','--out', LaunchConfiguration('mavros_url'), '--console'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('ardusub'))
        ),
    ])
