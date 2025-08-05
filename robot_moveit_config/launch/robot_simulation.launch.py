#    Copyright 2022 Christoph Hellmann Santos
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from moveit_configs_utils import MoveItConfigsBuilder
import xacro
import os
from ament_index_python.packages import get_package_share_directory


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
    
    #rviz = Node(
    #	package='rviz2',
    #	executable='rviz2',
    #	name='rviz2',
    #	output='screen',
   #	parameters=[
    #		{'robot_description': robot_urdf},
     #           {'robot_description_semantic': robot_srdf},
       #         {'robot_description_kinematics': kinematics}
      #  ],
        #arguments=['-d', os.path.join(share_dir, 'rviz', 'moveit.rviz')]
    #)
    
    # MoveIt
    moveit_config = MoveItConfigsBuilder("robot", package_name="robot_moveit_config").to_moveit_configs()

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
        ),
    )

    return LaunchDescription([robot_state_publisher_node, rviz])
