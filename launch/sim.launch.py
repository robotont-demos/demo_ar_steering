import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
	ld = LaunchDescription()

	# Arguments
	ld.add_action(DeclareLaunchArgument(
		'marker_id',
		default_value='0',
		description='ArUco marker ID to follow'
	))

	ld.add_action(DeclareLaunchArgument(
		'cmd_vel_topic',
		default_value='/cmd_vel',
		description='Velocity command topic'
	))

	# Nodes
	ld.add_action(Node(
		package='usb_cam',
		executable='usb_cam_node_exe',
	))

	ld.add_action(Node(
		package='ros2_aruco',
		executable='aruco_node',
		parameters=[{
			'image_topic': '/image_raw', 
			'camera_info_topic': '/camera_info'
		}]
	))

	ld.add_action(Node(
		package='demo_ar_steering',
		executable='ar_steering.py',
		parameters=[{
			'marker_id': LaunchConfiguration('marker_id'),
			'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic')
		}]
	))

	ld.add_action(Node(
		package='robotont_simple_simulator',
		executable='simple_driver_node',
		name='driver',
	))

	# Launch files
	ld.add_action(IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('robotont_description'), 'launch/display_robot_model.launch.py')
		),
		launch_arguments={
			'rviz_fixed_frame': 'odom',
			'rviz_config': os.path.join(get_package_share_directory('demo_ar_steering'), 'config', 'sim.rviz')
		}.items()
	))

	return ld