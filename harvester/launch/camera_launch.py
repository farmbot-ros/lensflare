import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    launch_description = LaunchDescription()

    node = Node(
        package='harvester',
        executable='camera_node',
        name='camera_node1',
        parameters=[
            {'camera_name':'11a'}
        ],
        output="screen",
        emulate_tty=True,
    )
    launch_description.add_action(node)
    node2 = Node(
        package='harvester',
        executable='camera_node',
        name='camera_node2',
        parameters=[
            {'camera_name':'13'},
            {'scan_cameras':False}
        ],
        output="screen",
        emulate_tty=True,
    )
    launch_description.add_action(node2)

    return launch_description
