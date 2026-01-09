from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    talker = Node(
        package="assign5_660610840",
        executable="talker",
        name="talker",
        output="screen",
        remappings=[
            (
                '/gossip_660610840', '/assignment5'
            )
        ],
        parameters=[
            {
                'who': 'myROS',
                'speaking': 'say',
                'spk_msg': 'y so EZ'
            }              
        ]
    )

    listener1 = Node(
        package="assign5_660610840",
        executable="listener1",
        name="listener1",
        output="screen",
        remappings=[
            (
                '/gossip_660610840', '/assignment5'
            )
        ]
    )

    listener2 = Node(
        package="assign5_660610840",
        executable="listener2",
        name="listener2",
        output="screen",
        remappings=[
            (
                '/gossip_660610840', '/assignment5'
            )
        ],
    )

    ld.add_action(talker)
    ld.add_action(listener1)
    ld.add_action(listener2)

    return ld

if __name__ == '__main__':
    generate_launch_description()
