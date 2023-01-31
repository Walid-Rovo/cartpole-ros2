import os

import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # Get pkg path
    pkg_cartpole_01 = get_package_share_directory("cartpole_01")

    # Ignition gazebo env variable to make pkg folders discoverable by it
    if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
        model_path = os.environ["IGN_GAZEBO_RESOURCE_PATH"] + ":" + pkg_cartpole_01
    else:
        model_path = pkg_cartpole_01

    # Robot description (cartpole.xacro -> xml)
    robot_description_file = os.path.join(pkg_cartpole_01, "urdf", "cartpole.xacro")
    robot_description_xacro = xacro.process_file(robot_description_file)
    robot_description = robot_description_xacro.toxml()

    # Robot state publisher
    params = {'use_sim_time': True, 'robot_description': robot_description}
    robot_state_publisher =  Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
            arguments=[])

    # Ignition gazebo
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={'ign_args': '-r sdf/cartpole_empty.sdf'}.items(),
    )

    # Spawner
    spawner = Node(package='ros_ign_gazebo', executable='create',
                 arguments=[
                    # '-world', 'cartpole_empty_world',
                    '-name', 'cartpole',
                    '-topic', '/robot_description'],
                 output='screen')

    # # Bridge
    # bridge = Node(
    #     package="ros_ign_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         "/world/empty/wrench@cartpole_msgs/EntityWrench@ignition.msgs.EntityWrench"
    #     ],
    #     output="screen",
    # )

    return LaunchDescription([
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=model_path),
        gazebo,
        robot_state_publisher,
        spawner,
        # bridge,
    ])
