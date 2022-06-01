import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    simulation_arg = LaunchConfiguration('simulation')
    fcu_url_arg = LaunchConfiguration('fcu_url')
    gcs_url_arg = LaunchConfiguration('gcs_url')
    system_id_arg = LaunchConfiguration('system_id')
    component_id_arg = LaunchConfiguration('component_id')
    tgt_system_arg = LaunchConfiguration('target_system_id')
    tgt_component_arg = LaunchConfiguration('target_component_id')

    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false'
    )

    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='udp://@localhost:14550'
    )

    system_id_arg = DeclareLaunchArgument(
        'system_id',
        default_value='255'
    )

    component_id_arg = DeclareLaunchArgument(
        'component_id',
        default_value='240'
    )

    tgt_system_arg = DeclareLaunchArgument(
        'target_system_id',
        default_value='1'
    )

    tgt_component_arg = DeclareLaunchArgument(
        'target_component_id',
        default_value='1'
    )

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'system_id': LaunchConfiguration('system_id'),
            'component_id': LaunchConfiguration('component_id'),
            'target_system_id': LaunchConfiguration('target_system_id'),
            'target_component_id': LaunchConfiguration('target_component_id'),
        }]
    )

    agent_node = Node(
        package='tudelft_hackathon',
        executable='bluerov_agent.py',
        output='screen'
    )

    pkg_tudelft_hackathon = get_package_share_directory('tudelft_hackathon')

    return LaunchDescription([
        simulation_arg,
        gcs_url_arg,
        system_id_arg,
        component_id_arg,
        tgt_system_arg,
        tgt_component_arg,
        # Real robot
        DeclareLaunchArgument(
            'fcu_url',
            default_value='udp://192.168.2.1:14550@192.168.2.2:14555',
            condition=UnlessCondition(LaunchConfiguration('simulation'))
        ),
        # # Simulation
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
