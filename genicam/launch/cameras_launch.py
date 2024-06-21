import launch
from launch import LaunchDescription
from launch_ros.actions import Node


cameras = ["9b",  "9a",  "9c",  "9d",  "9e",  "7a",  "7e",  "7c",  "7d",  "7b",  "1",   "4",   "3",   "12",  "10",   "13",   "11b",  "11a"]

def generate_launch_description():
    ld = LaunchDescription()

    camera_info = Node(
        package='genicam',
        executable='camera_info',
        output="screen",
        emulate_tty=True,
    )
    ld.add_action(camera_info)


    # camera_node = Node(
    #     package='genicam',
    #     executable='camera_node',
    #     output="screen",
    #     emulate_tty=True,
    #     arguments=['--managed', 'true'],
    # )
    # ld.add_action(camera_node)

    for camera in cameras:
        camera_node = Node(
            package='genicam',
            executable='camera_node',
            output="screen",
            emulate_tty=True,
            arguments=['--managed', camera],
        )
        ld.add_action(camera_node)

    camera_manager = Node(
        package='genicam',
        executable='camera_manager',
        output="screen",
        emulate_tty=True,
    )
    ld.add_action(camera_manager)



    return ld
