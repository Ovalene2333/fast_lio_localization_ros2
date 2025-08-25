#!/usr/bin/env python3
# coding: utf-8

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
	rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
	use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
	map_arg = DeclareLaunchArgument('map', default_value='/home/ovalene/xjtu_nav25/src/rm_localization/Point-LIO/PCD/red/map.pcd')

	# FAST_LIO mapping node is ROS1 in original launch; not included here.

	# Map publisher (PCD)
	pcd_pub = Node(
		package='fast_lio_localization_ros2',
		executable='pcd_publisher.py',
		name='map_publisher',
		output='screen',
		parameters=[{
			'map': LaunchConfiguration('map'),
			'frame_id': 'map3d',
			'rate': 1.0,
			'use_sim_time': LaunchConfiguration('use_sim_time')
		}]
	)

	# Global localization
	global_loc = Node(
		package='fast_lio_localization_ros2',
		executable='global_localization_ros2.py',
		name='global_localization',
		output='screen',
		parameters=[{
			'map2odom_completed': False,
			'region': 0,
			'use_sim_time': LaunchConfiguration('use_sim_time'),
			'map_frame': 'map3d',
			'odom_frame': 'camera_init',
			'base_link_frame': 'base_link'
		}]
	)

	# Transform fusion
	transform_fusion = Node(
		package='fast_lio_localization_ros2',
		executable='transform_fusion_ros2.py',
		name='transform_fusion',
		output='screen',
		parameters=[{
			'use_sim_time': LaunchConfiguration('use_sim_time'),
			'map_frame': 'map3d',
			'odom_frame': 'camera_init',
			'base_link_frame': 'base_link'
		}]
	)

	# Static TFs (replicating ROS1 args; ROS2 tool doesn't need frequency)
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

	rviz_node = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		arguments=['-d', '$(ros2 pkg prefix fast_lio_localization_ros2)/share/fast_lio_localization_ros2/rviz_cfg/localization_test.rviz']
	)

	return LaunchDescription([
		rviz_arg,
		use_sim_time_arg,
		map_arg,
		pcd_pub,
		global_loc,
		transform_fusion,
		tf_body2base,
		tf_map3dto2d,
		tf_base_link2realsense,
		GroupAction([rviz_node], condition=IfCondition(LaunchConfiguration('rviz')))
	]) 