from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                parameters=[{"use_sim_time": False}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="both",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro ",
                                get_package_share_directory("solid3_description")
                                + "/urdf/solid3.urdf.xacro",
                            ]
                        )
                    }
                ],
            ),
        ]
    )
