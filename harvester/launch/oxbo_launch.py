import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    camera_node = Node(
        package='harvester',
        executable='camera_node',
        # name='camera_node',
        output="screen",
        emulate_tty=True,
    )
    flash_node = Node(
        package='harvester',
        executable='flash_node',
        # name='flash_node',
        output="screen",
        emulate_tty=True,
    )
    image_saver = Node(
        package='harvester',
        executable='image_saver',
        # name='image_saver',
        output="screen",
        emulate_tty=True,
    )
    trigger_node = Node(
        package='harvester',
        executable='trigger_node',
        # name='trigger_node',
        output="screen",
        emulate_tty=True,
    )


    # ld.add_action(camera_node)
    ld.add_action(flash_node)
    ld.add_action(image_saver)
    ld.add_action(trigger_node)

    return ld
