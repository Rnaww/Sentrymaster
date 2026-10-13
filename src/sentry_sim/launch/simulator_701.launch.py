import os
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sim_share = get_package_share_directory('sentry_sim')
    gazebo = PythonExpression(["'", LaunchConfiguration('physics'), "' == 'gazebo'"])
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py'])
        ]),
        condition=IfCondition(gazebo),
        launch_arguments={
            'world': os.path.join(sim_share, 'worlds', '701_navigation.world'),
        }.items(),
    )
    # 沿用旧工作空间直接启动 gzclient 的写法。
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('physics'), "' == 'gazebo' and '", LaunchConfiguration('gui'), "' == 'true'"
        ])),
        output='screen',
    )
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        condition=IfCondition(gazebo),
        arguments=['-entity', 'sentry_robot', '-file', os.path.join(sim_share, 'robots', 'sentry.urdf'), '-x', '7.25', '-y', '2.70', '-z', '0.6'],
        output='screen',
    )
    # 保留实车 Odin 安装 TF，车体、车轮和云台模型沿用旧工作空间。
    robot = ET.parse(os.path.join(get_package_share_directory('sentry_launch'), 'urdf', 'sentry_chassis.xacro')).getroot()
    model = ET.parse(os.path.join(sim_share, 'robots', 'sentry.urdf')).getroot()
    for element in model:
        if element.tag in ('link', 'joint'):
            robot.append(element)
    robot.append(ET.fromstring('<joint name="vehicle_to_sim_chassis" type="fixed"><parent link="vehicle"/><child link="sim_chassis_link"/></joint>'))
    robot_description = ET.tostring(robot, encoding='unicode').replace('model://', 'file://' + os.path.join(sim_share, 'models') + '/')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )
    odin_odometry = Node(
        package='sentry_planner',
        executable='odin_odometry_node',
        name='odin_odometry_node',
        parameters=[{'use_sim_time': True, 'align_initial_pose': False}],
        output='screen',
    )
    sentry_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('sentry_planner'), 'launch', 'global_planning.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    sentry_decision = Node(
        package='sentry_decision_sys',
        executable='sentry_decision_node',
        name='sentry_decision_node',
        parameters=[{'use_sim_time': True}],
        output='both',
    )
    simulator = Node(
        package='sentry_sim',
        executable='sentry_simulator',
        name='sentry_simulator',
        parameters=[{
            'physics': LaunchConfiguration('physics'),
            'realtime_factor': LaunchConfiguration('realtime_factor'),
            'output_file': LaunchConfiguration('output_file'),
        }],
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(sim_share, 'config', 'rviz_701.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('physics', default_value='kinematic'),
        DeclareLaunchArgument('realtime_factor', default_value='1.0'),
        DeclareLaunchArgument('output_file', default_value=''),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(sim_share, 'models') + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        gazebo_server,
        gazebo_client,
        spawn_robot,
        robot_state_publisher,
        odin_odometry,
        sentry_planner,
        sentry_decision,
        simulator,
        rviz,
    ])
