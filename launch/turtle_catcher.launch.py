from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    turtlesim = Node(package="turtlesim", executable="turtlesim_node",
                     name="turtlesim")
    turtle_controller = Node(package="turtle_catcher", executable="turtle_controller",
                             name="turtle_controller",
                             parameters=[
                                 {"catch_closest_turtle_first": True}
                             ])
    turtle_spawner = Node(package="turtle_catcher", executable="turtle_spawner",
                          name="turtle_spawner",
                          parameters=[
                              {"spawn_frequency": 1.4}
                          ])

    ld.add_action(turtlesim)
    ld.add_action(turtle_controller)
    ld.add_action(turtle_spawner)

    return ld
