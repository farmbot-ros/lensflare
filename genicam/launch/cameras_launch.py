import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    camera_info = Node(
        package='genicam',
        executable='camera_info',
        output="screen",
        emulate_tty=True,
    )
    camera_node = Node(
        package='genicam',
        executable='camera_node',
        output="screen",
        emulate_tty=True,
        arguments=['--managed', 'true'],
    )
    camera_manager = Node(
        package='genicam',
        executable='camera_manager',
        output="screen",
        emulate_tty=True,
    )


    ld.add_action(camera_info)
    ld.add_action(camera_node)
    ld.add_action(camera_manager)

    return ld
