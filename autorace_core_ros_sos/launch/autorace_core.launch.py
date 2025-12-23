from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    pkg_name = "autorace_core_ros_sos"

    road_vision_node = Node(
        package=pkg_name,
        executable="road_vision_node",
        name="road_vision_node",
        parameters=[{
            "max_depth": LaunchConfiguration("max_depth"),
            "side_k": LaunchConfiguration("side_k"),
            "back_dist_threshold": LaunchConfiguration("back_dist_threshold")
        }]
    )

    motion_controller = Node(
        package=pkg_name,
        executable="motion_controller",
        name="motion_controller",
        parameters=[
            {
                "c_p": LaunchConfiguration("c_p"),
                "c_i": LaunchConfiguration("c_i"),
                "c_d": LaunchConfiguration("c_d"),
                "c_error_limit": LaunchConfiguration("c_error_limit"),
                "t_p": LaunchConfiguration("t_p"),
                "t_i": LaunchConfiguration("t_i"),
                "t_d": LaunchConfiguration("t_d"),
                "t_error_limit": LaunchConfiguration("t_error_limit"),
                "yaw_threshold": LaunchConfiguration("yaw_threshold")
            }
        ]
    )

    delayed_motion_controller = TimerAction(
        period=1.0,
        actions=[motion_controller]
    )

    traffic_light_node = Node(
        package=pkg_name,
        executable="traffic_light_node",
        name="traffic_light_node",
        parameters=[{
            "green_threshold": LaunchConfiguration("green_threshold")
        }]
    )

    delayed_traffic_light_node = TimerAction(
        period=1.0,
        actions=[traffic_light_node]
    )

    distance_node = Node(
        package=pkg_name,
        executable="distance_node",
        name="distance_node",
        parameters=[{
            "lidar_angle_view": LaunchConfiguration("lidar_angle_view"),
            "max_lidar_dist": LaunchConfiguration("max_lidar_dist"),
            "min_lidar_dist": LaunchConfiguration("min_lidar_dist")
        }]
    )

    delayed_distance_node = TimerAction(
        period=1.0,
        actions=[distance_node]
    )

    intersection_detector = Node(
        package=pkg_name,
        executable="intersection_detector",
        name="intersection_detector",
        parameters=[{
            "front_dist_threshold": LaunchConfiguration("front_dist_threshold")
        }]
    )

    delayed_intersection_detector = TimerAction(
        period=1.0,
        actions=[intersection_detector]
    )

    return LaunchDescription([
        DeclareLaunchArgument("c_p", default_value="2.0",
                              description="P coefficient in center PID"),
        DeclareLaunchArgument("c_i", default_value="0.0",
                              description="I coefficient in center PID"),
        DeclareLaunchArgument("c_d", default_value="0.05",
                              description="D coefficient in center PID"),
        DeclareLaunchArgument("c_error_limit", default_value="5.0",
                              description="Error limit for center PID regularization"),
        DeclareLaunchArgument("t_p", default_value="0.8",
                              description="P coefficient in turn PID"),
        DeclareLaunchArgument("t_i", default_value="0.0",
                              description="I coefficient in turn PID"),
        DeclareLaunchArgument("t_d", default_value="0.0",
                              description="D coefficient in turn PID"),
        DeclareLaunchArgument("t_error_limit", default_value="3.14",
                              description="Error limit for turn PID regularization"),
        DeclareLaunchArgument("yaw_threshold", default_value="5.0",
                              description="Yaw threshold in degrees for turning task"),
        DeclareLaunchArgument("max_depth", default_value="0.6",
                              description="Max depth fpr depth camera"),
        DeclareLaunchArgument("side_k", default_value="2.0",
                              description="Coef for x-priority"),
        DeclareLaunchArgument("green_threshold", default_value="500",
                              description="Num of pixels for green light detection"),
        DeclareLaunchArgument("lidar_angle_view", default_value="20.0",
                              description="Lidar view in degrees"),
        DeclareLaunchArgument("max_lidar_dist", default_value="2.0",
                              description="Max lidar distance"),
        DeclareLaunchArgument("min_lidar_dist", default_value="0.05",
                              description="Min lidar distance"),
        DeclareLaunchArgument("front_dist_threshold", default_value="0.8",
                              description="Treshold for turn sign"),
        DeclareLaunchArgument("back_dist_threshold", default_value="1.4",
                              description="Treshold for intersection exit"),
        road_vision_node,
        delayed_motion_controller,
        delayed_traffic_light_node,
        delayed_distance_node,
        delayed_intersection_detector
    ])
