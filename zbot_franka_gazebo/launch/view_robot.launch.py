from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    rviz_file = os.path.join(get_package_share_directory('zbot_franka_description'), 'rviz',
                             'visualize_franka.rviz')
    
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='false', description='Use RViz if true')
    declare_use_controller_gui = DeclareLaunchArgument('use_controller_gui', default_value='true', description='Use GUI controller if true')

    use_rviz = LaunchConfiguration('use_rviz')
    use_controller_gui = LaunchConfiguration('use_controller_gui')

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

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='',
        arguments=['-entity', 'panda', 
                   '-topic', 'robot_description'],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_arm_controller', '-c', '/controller_manager'],
    )

    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_hand_controller', '-c', '/controller_manager'],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        remappings=[('/joint_states', '/joint_states_gui_commands')],
        condition=IfCondition(use_controller_gui),
    )
    joint_state_to_control_node = Node(
        package='zbot_franka_control',
        executable='joint_state_to_control_action_client',
        name='joint_state_to_control',
        condition=IfCondition(use_controller_gui),
    )

    rviz2_node = Node(package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_file],
        condition=IfCondition(use_rviz),
    )

    timer_action = TimerAction(
        period=5.0,
        actions=[joint_state_publisher_gui_node, joint_state_to_control_node],
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_controller_gui)

    ld.add_action(gazebo_launch)
    ld.add_action(zbot_franka_description_launch)
    ld.add_action(rviz2_node)
    ld.add_action(spawn_entity)

    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(arm_controller_spawner)
    ld.add_action(hand_controller_spawner)

    # ld.add_action(joint_state_publisher_gui_node)
    # ld.add_action(joint_state_to_control_node)
    ld.add_action(timer_action)

    return ld 