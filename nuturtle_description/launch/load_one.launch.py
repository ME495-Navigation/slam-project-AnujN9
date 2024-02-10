from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_rviz",
                default_value="false",
                choices=["true", "false"],
                description="Launch rviz",
            ),
            DeclareLaunchArgument(
                name="use_jsp",
                default_value="true",
                choices=["true", "false"],
                description="Enable joint_state_publisher_node",
            ),
            DeclareLaunchArgument(
                name="color",
                default_value="purple",
                choices=["purple", "red", "blue", "green", ""],
                description="Select the color of the turtlebot base model",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=LaunchConfiguration("color"),
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                TextSubstitution(text="xacro "),
                                PathJoinSubstitution(
                                    [
                                        FindPackageShare("nuturtle_description"),
                                        "urdf",
                                        "turtlebot3_burger.urdf.xacro",
                                    ]
                                ),
                                TextSubstitution(text=" color:="),
                                LaunchConfiguration("color"),
                            ]
                        )
                    },
                    {
                        "frame_prefix": [
                            LaunchConfiguration("color"),
                            TextSubstitution(text="/"),
                        ]
                    },
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                condition=LaunchConfigurationEquals("use_jsp", "true"),
                namespace=LaunchConfiguration("color"),
                arguments=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nuturtle_description"),
                            "urdf",
                            "turtlebot3_burger.urdf.xacro",
                        ]
                    )
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                condition=LaunchConfigurationEquals("use_rviz", "true"),
                namespace=LaunchConfiguration("color"),
                on_exit=Shutdown(),
                arguments=[
                    "-d",
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("nuturtle_description"),
                                "config",
                                "basic_",
                            ]
                        ),
                        LaunchConfiguration("color"),
                        TextSubstitution(text=".rviz"),
                    ],
                ],
            ),
        ]
    )
