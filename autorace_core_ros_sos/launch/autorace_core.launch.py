from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    road_vision_node = Node(
        package="autorace_core_ros_sos",
        executable="road_vision_node",
        name="road_vision_node",
        parameters=[
            {"turn_direction": "right"}
        ]
    )

    motion_controller = Node(
        package="autorace_core_ros_sos",
        executable="motion_controller",
        name="motion_controller",
        parameters=[
            {
                "kp": LaunchConfiguration("kp"),
                "ki": LaunchConfiguration("ki"),
                "kd": LaunchConfiguration("kd"),
                "error_limit": LaunchConfiguration("error_limit")
            }
        ]
    )

    delayed_motion_controller = TimerAction(
        period=1.0,
        actions=[motion_controller]
    )

    return LaunchDescription([
        DeclareLaunchArgument("kp", default_value="2.5",
                              description="P coefficient in PID"),
        DeclareLaunchArgument("ki", default_value="0.0",
                              description="I coefficient in PID"),
        DeclareLaunchArgument("kd", default_value="0.0",
                              description="D coefficient in PID"),
        DeclareLaunchArgument("error_limit", default_value="5.0",
                              description="Error limit for PID regularization"),
        road_vision_node,
        delayed_motion_controller
    ])
