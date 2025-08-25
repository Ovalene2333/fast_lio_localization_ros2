#!/usr/bin/env python3
# coding: utf-8

from __future__ import print_function, division, absolute_import

import copy
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_matrix, translation_from_matrix, quaternion_from_matrix


class TransformFusionNode(Node):
	def __init__(self) -> None:
		super().__init__('transform_fusion')

		self.declare_parameter('publish_rate', 100.0)
		self.declare_parameter('map_frame', 'map3d')
		self.declare_parameter('odom_frame', 'camera_init')
		self.declare_parameter('base_link_frame', 'base_link')

		self.cur_map_to_odom = None  # Odometry
		self.cur_odom_to_baselink = None  # Odometry

		# Publisher and TF broadcaster
		self.pub_localization = self.create_publisher(Odometry, '/localization', 10)
		self.tf_broadcaster = TransformBroadcaster(self)

		# Subscriptions
		reliable_qos = QoSProfile(depth=10)
		reliable_qos.reliability = ReliabilityPolicy.RELIABLE
		self.create_subscription(Odometry, '/odom', self.cb_save_cur_odom, reliable_qos)
		self.create_subscription(Odometry, '/map_to_odom', self.cb_save_map_to_odom, reliable_qos)

		# Timer for publishing
		publish_rate = float(self.get_parameter('publish_rate').value)
		self.timer = self.create_timer(1.0 / max(1e-6, publish_rate), self.on_timer)

		self.get_logger().info('Transform Fusion Node Inited (ROS2) ...')

	def pose_to_mat(self, odom_msg: Odometry) -> np.ndarray:
		p = odom_msg.pose.pose.position
		q = odom_msg.pose.pose.orientation
		T = quaternion_matrix([q.x, q.y, q.z, q.w])
		T[:3, 3] = [p.x, p.y, p.z]
		return T

	def on_timer(self) -> None:
		map_frame = self.get_parameter('map_frame').value
		odom_frame = self.get_parameter('odom_frame').value
		base_link_frame = self.get_parameter('base_link_frame').value

		if self.cur_map_to_odom is not None:
			T_map_to_odom = self.pose_to_mat(self.cur_map_to_odom)
		else:
			T_map_to_odom = np.eye(4)

		# Publish TF map->odom
		xyz = translation_from_matrix(T_map_to_odom)
		quat = quaternion_from_matrix(T_map_to_odom)
		transform = TransformStamped()
		transform.header.stamp = self.get_clock().now().to_msg()
		transform.header.frame_id = map_frame
		transform.child_frame_id = odom_frame
		transform.transform.translation.x = float(xyz[0])
		transform.transform.translation.y = float(xyz[1])
		transform.transform.translation.z = float(xyz[2])
		transform.transform.rotation.x = float(quat[0])
		transform.transform.rotation.y = float(quat[1])
		transform.transform.rotation.z = float(quat[2])
		transform.transform.rotation.w = float(quat[3])
		self.tf_broadcaster.sendTransform(transform)

		if self.cur_odom_to_baselink is not None:
			cur_odom = copy.copy(self.cur_odom_to_baselink)
			T_odom_to_base_link = self.pose_to_mat(cur_odom)
			T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)
			xyz2 = translation_from_matrix(T_map_to_base_link)
			quat2 = quaternion_from_matrix(T_map_to_base_link)

			localization = Odometry()
			localization.pose.pose = Pose(Point(*xyz2), Quaternion(*quat2))
			localization.twist = cur_odom.twist
			localization.header.stamp = cur_odom.header.stamp
			localization.header.frame_id = map_frame
			localization.child_frame_id = base_link_frame
			self.pub_localization.publish(localization)

	def cb_save_cur_odom(self, odom_msg: Odometry) -> None:
		self.cur_odom_to_baselink = odom_msg

	def cb_save_map_to_odom(self, odom_msg: Odometry) -> None:
		self.cur_map_to_odom = odom_msg


def main():
	rclpy.init()
	node = TransformFusionNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main() 