from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

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

    ping_360_node = Node(
        package='ping360_sonar',
        executable='ping360_node',
        parameters=[{
            'connection_type': 'udp',
            'udp_address': '192.168.2.2',
            'udp_port': 9092,
            'fallback_emulated': False,
            'publish_eco': True,
            'publish_scan': True,
            'publish_image': True,
        }],
    )

    agent_node = Node(
        package='tudelft_hackathon',
        executable='bluerov_agent.py',
        output='screen'
    )

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
        mavros_node,
        ping_360_node,
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'],
            output='screen'
        )
    ])
