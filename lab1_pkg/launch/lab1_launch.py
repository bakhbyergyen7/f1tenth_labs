import launch
import launch_ros.actions

def generate_launch_description():
    talker_node = launch_ros.actions.Node(
        package = 'lab1_pkg',
        executable = 'talker',
        output = 'screen',
        parameters = [
            {'v':1.5},
            {'d':0.75}
        ]
    )

    relay_node = launch_ros.actions.Node(
        package = 'lab1_pkg',
        executable = 'relay',
        output = 'screen'
    )

    return launch.LaunchDescription([talker_node, relay_node])

if __name__ == '__main__':
    generate_launch_description()
