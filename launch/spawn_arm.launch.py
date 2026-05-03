import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'my_collaborative_arm'
    
    # Path to the robot description
    urdf_path = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'my_arm.urdf.xacro')
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Path to the controller settings
    controller_config = os.path.join(get_package_share_directory(pkg_name), 'config', 'arm_controllers.yaml')

    # Path to the new world file
    world_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'fruit_sorting.world')

    # 1. Force NVIDIA GPU Environment Variables
    nv_render = SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1')
    nv_glx = SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')

    # 2. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 3. Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )

    # 4. Spawn the robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'collaborative_arm'],
        output='screen'
    )

    # 5. Spawn Joint State Broadcaster (The "Senses")
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # 6. Spawn Arm Controller (The "Muscles")
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )

    # 7. Spawn Gripper Controller (The "Hands")
    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'],
        output='screen'
    )

    return LaunchDescription([
        nv_render,
        nv_glx,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        # Wait a few seconds for Gazebo to load before starting controllers
        TimerAction(period=5.0, actions=[load_joint_state_broadcaster]),
        TimerAction(period=7.0, actions=[load_arm_controller]),
        TimerAction(period=8.0, actions=[load_gripper_controller]),
    ])