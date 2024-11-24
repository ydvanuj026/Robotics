import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node'),
            launch_ros.actions.Node(
                package='servoROS',
                executable='servoSerial',
                name='serialSender'
            ),
            launch_ros.actions.Node(
                package='my_robot_controller',
                executable='draw_circle',
                name='draw_circle'
            ),
  ])