from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    zbot_franka_xacro_file = os.path.join(get_package_share_directory('zbot_franka_description'), 'urdf',
                                     'panda_main.urdf.xacro')
    
    robot_description_content = Command(
        [FindExecutable(name='xacro'), ' ', zbot_franka_xacro_file])
    
    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
        )
    
    ld = LaunchDescription()

    ld.add_action(robot_state_publisher_node)
    # ld.add_action(static_tf)

    
    return ld