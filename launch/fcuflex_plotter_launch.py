from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fcuflex_plotter',
            executable='fcuflex_plotter',
            name='fcuflex_plotter',
            output='screen'
        )
    ])