import launch
import launch.actions
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    model_path = PathJoinSubstitution([
        get_package_share_directory('human_pose_estimation'),
        'models',
        LaunchConfiguration("model")
    ])

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "model", 
            default_value="movenet_thunder.tflite",
            description="name of the model to use",
        ),

        launch_ros.actions.Node(
            package="human_pose_estimation",
            executable="human_body_pose",
            name="human_pose",
            output="screen",
            parameters=[
                {"model_name": model_path}
            ]
        ),
    ])