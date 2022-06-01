import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

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
    ])
