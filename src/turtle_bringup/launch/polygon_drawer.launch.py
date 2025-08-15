from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package = "turtlesim",
        executable = "turtlesim_node",
    )

    polygon_drawer_node = Node(
        package="turtle_script",
        executable="polygon_drawer",
        parameters=[{
            "sides": 5,
            "side_length": 2.5
        }],
        output="screen"
    )


    ld.add_action(turtlesim_node)
    ld.add_action(polygon_drawer_node)

    return ld