import launch
import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, GroupAction

def generate_launch_description():

    zed_launch = PathJoinSubstitution([
        get_package_share_directory('zed_wrapper'),
        'launch',
        'zed2.launch.py'
    ])

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "log_level", 
            default_value="INFO",
            description="log level to use",
        ),

        # start up the camera
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                zed_launch
            ),
            launch_arguments=[
                ('log_level', LaunchConfiguration("log_level"))
            ]
        ),

        # convert camera output to a TF frame
        Node(
            name="zed_translator",
            package="human_pose_estim",
            executable="zed_translator",
            respawn=True,
            arguments=['--ros-args', '--log-level', LaunchConfiguration("log_level")],
            output='screen',
        ),

        # These transforms are all in the ENU Frame (E and N not fully respected)
        Node(
            name="world_to_robot_ground",
            package="tf2_ros",
            executable="static_transform_publisher",
            respawn=True,
            output='screen',
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "robot_ground_link"]
        ),

        Node(
            name="robot_ground_to_base",
            package="tf2_ros",
            executable="static_transform_publisher",
            respawn=True,
            output='screen',
            arguments=["0", "0", "0.889", "0", "0", "0", "robot_ground_link", "robot_base_link"]
        ),

        # TODO link robot_base_link to turret_base_link via a tf broadcaster
        Node(
            name="joint_state_pub",
            package="human_pose_estim",
            executable="joint_tf",
            respawn=True,
            arguments=['--ros-args', '--log-level', LaunchConfiguration("log_level")],
            output='screen',
        ),

        Node(
            name="robot_turret_to_cam",
            package="tf2_ros",
            executable="static_transform_publisher",
            respawn=True,
            output='screen',
            arguments=["0.0635", "-0.1143", "0.08255", "0", "0", "0", "turret_base_link", "camera_link"] #TODO set the link offset
        ),

        Node(
            name="robot_turret_to_shoot",
            package="tf2_ros",
            executable="static_transform_publisher",
            respawn=True,
            output='screen',
            arguments=["0", "0", "0.3302", "0", "0", "0", "robot_base_link", "shoot_link"]
        ),
    ])