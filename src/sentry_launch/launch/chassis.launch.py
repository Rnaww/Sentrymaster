import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, Shutdown, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    uart_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uart_driver'), 
                'launch',
                'driver.launch.py'
            ])
        ])
    )

    sentry_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sentry_planner'), 
                'launch',
                'global_planning.launch.py'
            ])
        ])
    )

    odin1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('odin_ros_driver'), 
                'launch',
                'odin1_ros2.launch.py'
            ])
        ])
    )

    robot_description = Command(
        [
            "xacro ",
            os.path.join(
                get_package_share_directory("sentry_launch"),
                "urdf",
                "sentry_chassis.xacro",
            ),
        ]
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "publish_frequency": 500.0}
        ],
    )

    sentry_decision = Node(
        package='sentry_decision_sys',
        executable='sentry_decision_node',
        name='sentry_decision_node',
        parameters=[os.path.join(get_package_share_directory('sentry_decision_sys'), 'config', 'decision.yaml')],
        output='both',
    )

    odin_odometry = Node(
        package='sentry_planner',
        executable='odin_odometry_node',
        name='odin_odometry_node',
        output='screen',
    )

    recorder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('sentry_record'), 'launch', 'record.launch.py'])
        ]),
        condition=IfCondition(LaunchConfiguration('record')),
        launch_arguments={'use_sim_time': 'false', 'record_directory': LaunchConfiguration('record_directory')}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('record_directory', default_value=os.path.join(os.getcwd(),'recordings')),
        recorder,
        # 机器人固定坐标变换发布节点
        robot_state_publisher,
        # USB驱动
        # uart_driver,
        # Odin1驱动
        # odin1,
        # 导航节点
        odin_odometry,
        sentry_planner,
        # 决策节点
        sentry_decision,
    ])
