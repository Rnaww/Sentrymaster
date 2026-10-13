import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    directory = PathJoinSubstitution([LaunchConfiguration('record_directory'), datetime.now().strftime('run_%Y%m%d_%H%M%S_%f')])
    topics = ['/clock', '/tf', '/tf_static', '/navigation_odom', '/navigation2mcu', '/decision2nav', '/decision2mcu', '/decision_state', '/referee_data', '/chassis_data', '/recorded_acceleration']
    telemetry = Node(
        package='sentry_record',
        executable='telemetry_node',
        name='sentry_telemetry',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'output_directory': directory}],
        output='screen',
    )
    bag = ['ros2', 'bag', 'record', '-o', PathJoinSubstitution([directory, 'bag']), '--storage', 'sqlite3']
    sim_record = ExecuteProcess(
        cmd=bag+['--use-sim-time']+topics,
        condition=IfCondition(LaunchConfiguration('use_sim_time')),
        output='screen',
    )
    real_record = ExecuteProcess(
        cmd=bag+topics,
        condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('record_directory', default_value=os.path.join(os.getcwd(),'recordings')),
        telemetry,
        sim_record,
        real_record,
    ])
