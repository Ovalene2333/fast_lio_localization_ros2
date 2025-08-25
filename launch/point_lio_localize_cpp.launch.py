#!/usr/bin/env python3
# coding: utf-8

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os 


def generate_launch_description():
	rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
	use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
	map_arg = DeclareLaunchArgument('map', default_value=PathJoinSubstitution([FindPackageShare('point_lio'), 'PCD', 'blue', 'map.pcd']))
	# Tunable parameters for global localization
	freq_localization_arg = DeclareLaunchArgument('freq_localization', default_value='0.5')
	localization_th_arg = DeclareLaunchArgument('localization_th', default_value='0.05')
	map_voxel_size_arg = DeclareLaunchArgument('map_voxel_size', default_value='0.2')
	scan_voxel_size_arg = DeclareLaunchArgument('scan_voxel_size', default_value='0.1')
	fov_arg = DeclareLaunchArgument('fov', default_value='6.28')
	fov_far_arg = DeclareLaunchArgument('fov_far', default_value='30.0')
	use_gicp_arg = DeclareLaunchArgument('use_gicp', default_value='false')

	# Point-LIO node
	point_lio_dir = get_package_share_directory("point_lio")
	point_lio_cfg = os.path.join(point_lio_dir, "config", "mid360.yaml")
	point_lio_params_list = []
	if os.path.exists(point_lio_cfg):
		point_lio_params_list.append(point_lio_cfg)

	point_lio_node = Node(
		package="point_lio",
		executable="pointlio_mapping",
		parameters=point_lio_params_list,
		remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
		output="screen",
	)

	# Map publisher (PCD) -> use C++ executable
	pcd_pub = Node(
		package='fast_lio_localization_ros2',
		executable='pcd_publisher',
		name='map_publisher',
		output='screen',
		parameters=[{
			'map': LaunchConfiguration('map'),
			'frame_id': 'map3d',
			'rate': 1.0,
			'use_sim_time': LaunchConfiguration('use_sim_time')
		}]
	)

	# Global localization -> use C++ executable
	global_loc = Node(
		package='fast_lio_localization_ros2',
		executable='global_localization',
		name='global_localization',
		output='screen',
		parameters=[{
			'map2odom_completed': False,
			'region': 0,
			'use_sim_time': LaunchConfiguration('use_sim_time'),
			'map_frame': 'map3d',
			'odom_frame': 'camera_init',
			'base_link_frame': 'base_link',
			'freq_localization': LaunchConfiguration('freq_localization'),
			'localization_th': LaunchConfiguration('localization_th'),
			'map_voxel_size': LaunchConfiguration('map_voxel_size'),
			'scan_voxel_size': LaunchConfiguration('scan_voxel_size'),
			'fov': LaunchConfiguration('fov'),
			'fov_far': LaunchConfiguration('fov_far'),
			'use_gicp': LaunchConfiguration('use_gicp')
		}]
	)

	# Transform fusion -> use C++ executable
	transform_fusion = Node(
		package='fast_lio_localization_ros2',
		executable='transform_fusion',
		name='transform_fusion',
		output='screen',
		parameters=[{
			'use_sim_time': LaunchConfiguration('use_sim_time'),
			'map_frame': 'map3d',
			'odom_frame': 'camera_init',
			'base_link_frame': 'base_link'
		}]
	)

	# Static TFs
	# body -> base_link
	tf_body2base = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='tf_body2base_link',
		arguments=['0.0', '0.12', '-0.28', '1.5707963267948966', '0.27', '0', 'body', 'base_link']
	)

	# map (2d) -> map3d
	tf_map3dto2d = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='tf_map3dto2d',
		arguments=['0', '0', '0.24', '-1.5707963267948966', '0', '0', 'map', 'map3d']
	)

	# base_link -> camera_link
	tf_base_link2realsense = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='tf_base_link2realsense',
		arguments=['0.3', '0', '0.28', '0', '0', '0', 'base_link', 'camera_link']
	)

	rviz_config_path = os.path.join(
		get_package_share_directory('point_lio'),
		'rviz_cfg',
		'localize.rviz'
	)

	rviz_node = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		arguments=['-d', rviz_config_path]
	)

	return LaunchDescription([
		rviz_arg,
		use_sim_time_arg,
		map_arg,
		freq_localization_arg,
		localization_th_arg,
		map_voxel_size_arg,
		scan_voxel_size_arg,
		fov_arg,
		fov_far_arg,
		use_gicp_arg,
		point_lio_node,
		pcd_pub,
		global_loc,
		transform_fusion,
		tf_body2base,
		tf_map3dto2d,
		tf_base_link2realsense,
		GroupAction([rviz_node], condition=IfCondition(LaunchConfiguration('rviz')))
	]) 