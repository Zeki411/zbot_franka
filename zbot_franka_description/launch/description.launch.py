from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    load_gripper_declare = DeclareLaunchArgument('load_gripper', default_value='true', description='Use Franka Gripper as end-effector if true. Robot is loaded without end-effector otherwise')

    load_gripper = LaunchConfiguration('load_gripper')

    zbot_franka_xacro_file = os.path.join(get_package_share_directory('zbot_franka_description'), 'urdf',
                                     'panda_arm.urdf.xacro')
    
    robot_description_content = Command(
        [FindExecutable(name='xacro'), ' ', zbot_franka_xacro_file, ' hand:=', load_gripper])

    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
        )
    
    ld = LaunchDescription()

    ld.add_action(load_gripper_declare)
    ld.add_action(robot_state_publisher_node)

    return ld