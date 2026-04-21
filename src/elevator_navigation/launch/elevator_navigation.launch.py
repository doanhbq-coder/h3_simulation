import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Free space goal node
        launch_ros.actions.Node(
            package='elevator_navigation',
            executable='free_space_goal',
            name='free_space_goal',
            output='screen'
        ),

        # Visualizer node
        launch_ros.actions.Node(
            package='elevator_navigation',
            executable='elevator_visualizer',
            name='elevator_visualizer',
            output='screen'
        ),
    ])