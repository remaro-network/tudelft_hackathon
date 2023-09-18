import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_tudelft_hackathon = get_package_share_directory('tudelft_hackathon')

    simulation_arg = LaunchConfiguration('simulation')
    fcu_url_arg = LaunchConfiguration('fcu_url')
    gcs_url_arg = LaunchConfiguration('gcs_url')
    tgt_system_arg = LaunchConfiguration('target_system_id')
    tgt_component_arg = LaunchConfiguration('target_component_id')

    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='true'
    )

    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='udp://@localhost:14550'
    )

    tgt_system_arg = DeclareLaunchArgument(
        'tgt_system',
        default_value='1'
    )

    tgt_component_arg = DeclareLaunchArgument(
        'tgt_component',
        default_value='1'
    )

    mavros_path = get_package_share_directory('mavros')
    mavros_launch_path = os.path.join(
        mavros_path, 'launch', 'node.launch')
    hackathon_apm_config_path = os.path.join(
        pkg_tudelft_hackathon, 'launch', 'apm_config.yaml')
    hackathon_apm_plugin_path = os.path.join(
        pkg_tudelft_hackathon, 'launch', 'apm_pluginlists.yaml')
    mavros_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_launch_path),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'tgt_system': LaunchConfiguration('tgt_system'),
            'tgt_component': LaunchConfiguration('tgt_component'),
            'config_yaml': hackathon_apm_config_path,
            'pluginlists_yaml': hackathon_apm_plugin_path,
            }.items()
        )

    agent_node = Node(
        package='tudelft_hackathon',
        executable='random_wall_avoidance.py',
        output='screen'
    )

    return LaunchDescription([
        simulation_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        # Real robot
        DeclareLaunchArgument(
            'fcu_url',
            default_value='udp://192.168.2.1:14550@192.168.2.2:14555',
            condition=UnlessCondition(LaunchConfiguration('simulation'))
        ),
        # Simulation
        DeclareLaunchArgument(
            'fcu_url',
            default_value='udp://:14551@:14555',
            condition=IfCondition(LaunchConfiguration('simulation'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tudelft_hackathon,
                             'launch',
                             'bluerov_ign_sim.launch.py')),
            condition=IfCondition(LaunchConfiguration('simulation'))
        ),
        mavros_node,
        Node(
            package='ping360_sonar',
            executable='ping360_node',
            parameters=[{
                'angle_sector': 90,
                'connection_type': 'udp',
                'udp_address': '192.168.2.2',
                'udp_port': 9092,
                'fallback_emulated': False,
                'publish_echo': True,
                'publish_scan': True,
                'publish_image': True,
            }],
            condition=UnlessCondition(LaunchConfiguration('simulation'))
        ),
        agent_node,
    ])
