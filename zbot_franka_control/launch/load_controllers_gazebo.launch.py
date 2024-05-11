from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Declare the prefix argument
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Namespace of the panda'
    )

    # Use the prefix in controller names
    name_space = LaunchConfiguration('namespace')

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', '--set-state', 'active',
            Command(['echo ', '-c ', name_space, '/controller_manager', ' ' ,'panda_joint_state_broadcaster'])
        ],
        output='screen',
        shell=True  # Ensure that the command is executed in a shell environment
    )

    load_panda_joint_group_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', '--set-state', 'active',
            Command(['echo ', '-c ', name_space, '/controller_manager', ' ', 'panda_joint_group_controller'])
        ],
        output='screen',
        shell=True  # Ensure that the command is executed in a shell environment
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(load_joint_state_broadcaster)

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_panda_joint_group_controller],
            )
        )
    )

    return ld
