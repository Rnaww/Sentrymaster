import os, yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    planner_share = get_package_share_directory('sentry_planner')
    nav_params = os.path.join(planner_share, 'config', 'nav2.yaml')
    global_params = os.path.join(planner_share, 'config', 'global_planning.yaml')
    with open(global_params) as file:
        planner_config = yaml.safe_load(file)['/sentry_planner_node']['ros__parameters']
    map_dir = os.path.join(planner_share, 'map', '701_model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav_params, {'use_sim_time': use_sim_time, 'yaml_filename': os.path.join(map_dir, '701_nav_map.yaml')}],
    )
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav_params, {
            'use_sim_time': use_sim_time,
            'SentryMinco.globalPlanner.PreMapPath': os.path.join(map_dir, '701_navigation.png'),
            'SentryMinco.globalPlanner.PreCrossPath': os.path.join(map_dir, '701_navigation.png'),
            'SentryMinco.globalPlanner.PreDeepPath': os.path.join(map_dir, 'new_701_deep.png'),
            'SentryMinco.terrainSpeed': planner_config['terrainSpeed'],
            'SentryMinco.velocityTracker.planar_max_acc': planner_config['velocityTracker']['planar_max_acc'],
        }],
    )
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/cmd_vel_nav')],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav_params, {
            'use_sim_time': use_sim_time,
            'default_nav_to_pose_bt_xml': os.path.join(planner_share, 'config', 'navigation.xml'),
            'default_nav_through_poses_bt_xml': os.path.join(planner_share, 'config', 'navigation.xml'),
        }],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': ['map_server', 'planner_server', 'controller_server', 'bt_navigator']}],
    )
    sentry_planner = Node(
        package='sentry_planner',
        executable='sentry_planner_node',
        name='sentry_planner_node',
        output='screen',
        parameters=[global_params, {'use_sim_time': use_sim_time, 'globalPlanner.PreDeepPath': os.path.join(map_dir, 'new_701_deep.png')}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        map_server,
        planner_server,
        controller_server,
        bt_navigator,
        lifecycle_manager,
        sentry_planner,
    ])
