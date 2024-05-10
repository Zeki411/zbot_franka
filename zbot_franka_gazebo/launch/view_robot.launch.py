from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    rviz_file = os.path.join(get_package_share_directory('zbot_franka_description'), 'rviz',
                             'visualize_franka.rviz')
    
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true')

    use_sim_time = LaunchConfiguration('use_sim_time')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 
                'launch', 
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world': PathJoinSubstitution([FindPackageShare('zbot_franka_gazebo'), 'worlds', 'empty.world']),
            'verbose': 'true',
            'paused': 'true',
            'gui': 'true',
            # 'physics': 'ode',
        }.items()
    )

    zbot_franka_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('zbot_franka_description'),
                'launch',
                'description.launch.py'
            ])
        ),
        launch_arguments={'load_gripper': 'true'}.items(),
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    
    rviz2_node = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # namespace=,
        arguments=['-entity', 'panda', 
                   '-topic', 'robot_description'],
        output="screen",
    )

    load_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('zbot_franka_control'),
                'launch',
                'load_controllers_gazebo.launch.py'
            ])
        )
    )


    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(gazebo_launch)
    ld.add_action(zbot_franka_description_launch)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz2_node)
    ld.add_action(spawn_entity)
    # ld.add_action(load_controllers_launch)

    return ld 