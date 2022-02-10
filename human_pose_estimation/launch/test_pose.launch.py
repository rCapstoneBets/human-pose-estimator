import launch
import launch.actions
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="image_tools",
            executable="showimage",
            name="processed_sub",
            output="screen",
            arguments=['--ros-args', '--log-level', 'WARN'],
            remappings = [
                ("/image", "/robot/signaling/overlay")
            ]
        ),

        launch_ros.actions.Node(
            package="image_tools",
            executable="cam2image",
            name="camera_pub",
            output="screen",
            arguments=['--ros-args', '--log-level', 'WARN'],
            remappings = [
                ("/image", "/robot/signaling/raw")
            ]
        ),
    ])