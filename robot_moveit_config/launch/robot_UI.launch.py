from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import os

def generate_launch_description():
    
    share_dir = get_package_share_directory('robot_moveit_config')

    xacro_file = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    
    srdf_file = os.path.join(share_dir, 'config', 'robot.srdf')
    robot_srdf_config = xacro.process_file(srdf_file)
    robot_srdf = robot_srdf_config.toxml()
    
    kinematics = os.path.join(share_dir, 'config', 'kinematics.yaml')

    robot_state_publisher_node = Node(
    	package='robot_state_publisher',
    	executable='robot_state_publisher',
    	output='screen',
    	name='robot_state_publisher',
    	parameters=[
    		{'robot_description': robot_urdf}
    	]
    )

    # Load MoveIt configuration
    moveit_config = MoveItConfigsBuilder("robot", package_name="robot_moveit_config").to_moveit_configs()

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
        ),
    )

    # Define the nodes to launch from robot_control package
    move_home_node = Node(
        package='robot_control',
        executable='move_home',  # Assuming executable is named 'move_home'
        name='move_home_node',
        output='screen'
    )

    move_X_Z_node = Node(
        package='robot_control',
        executable='move_X_Z',  # Assuming executable is named 'move_X_Z'
        name='move_X_Z_node',
        output='screen'
    )
    
    joint_listener_node = Node(
        package='robot_control',
        executable='joint_listener',
        name='joint_listener_node',
        output='screen'
    )

    move_to_pose_node = Node(
        package='robot_control',
        executable='move_to_pose',
        name='move_to_pose_node',
        output='screen'
    )
    
    # Return the launch description with all the nodes and demo launch
    return LaunchDescription([
    	robot_state_publisher_node,
        rviz,
        move_home_node,
        move_X_Z_node,
        joint_listener_node,
        move_to_pose_node
    ])
