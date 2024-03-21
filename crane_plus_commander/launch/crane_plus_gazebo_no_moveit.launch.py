import os

from ament_index_python.packages import get_package_share_directory
from crane_plus_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.actions import SetParameter


def generate_launch_description():
    # PATHを追加で通さないとSTLファイルが読み込まれない
    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.environ['LD_LIBRARY_PATH'],
           'IGN_GAZEBO_RESOURCE_PATH': os.path.dirname(
               get_package_share_directory('crane_plus_description'))}
    world_file = os.path.join(
        get_package_share_directory('crane_plus_gazebo'), 'worlds', 'table.sdf')
    gui_config = os.path.join(
        get_package_share_directory('crane_plus_gazebo'), 'gui', 'gui.config')
    # -r オプションで起動時にシミュレーションをスタートしないと、コントローラが起動しない
    ign_gazebo = ExecuteProcess(
            cmd=['ign gazebo -r', world_file, '--gui-config', gui_config],
            output='screen',
            additional_env=env,
            shell=True
        )

    gazebo_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description',
                   '-name', 'crane_plus',
                   '-z', '1.015',
                   '-allow_renaming', 'true'],
    )

    description_loader = RobotDescriptionLoader()
    description_loader.use_gazebo = 'true'
    description_loader.use_ignition = 'true'
    description_loader.gz_control_config_package = 'crane_plus_control'
    description_loader.gz_control_config_file_path = 'config/crane_plus_controllers.yaml'
    description = description_loader.load()

    robot_description = {'robot_description': description}

    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[robot_description])

    spawn_joint_state_broadcaster = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner joint_state_broadcaster'],
                shell=True,
                output='screen',
            )

    spawn_arm_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner crane_plus_arm_controller'],
                shell=True,
                output='screen',
            )

    spawn_gripper_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner crane_plus_gripper_controller'],
                shell=True,
                output='screen',
            )

    bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
                output='screen'
            )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        ign_gazebo,
        robot_state_publisher,
        gazebo_spawn_entity,
        spawn_joint_state_broadcaster,
        spawn_arm_controller,
        spawn_gripper_controller,
        bridge,
    ])
