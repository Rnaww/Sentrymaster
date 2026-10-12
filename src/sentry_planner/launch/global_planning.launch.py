import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _create_planner_node(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    map_dir = os.path.join(get_package_share_directory("sentry_planner"), "map", "701_model")
    parameters = [
        params_file,
        {
            "globalPlanner.PreMapPath": os.path.join(map_dir, "701_navigation.png"),
            "globalPlanner.PreCrossPath": os.path.join(map_dir, "701_navigation.png"),
            "globalPlanner.PreDeepPath": os.path.join(map_dir, "new_701_deep.png"),
            "use_sim_time": LaunchConfiguration("use_sim_time").perform(context) == "true",
        },
    ]


    return [
        Node(
            package="sentry_planner",
            executable="sentry_planner_node",
            name="sentry_planner_node",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description():
    planner_share = FindPackageShare("sentry_planner")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [planner_share, "config", "global_planning.yaml"]
                ),
                description="Path to the planner parameter file.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_create_planner_node),
        ]
    )
