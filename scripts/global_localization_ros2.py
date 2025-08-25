#!/usr/bin/env python3
# coding: utf-8

from __future__ import print_function, division, absolute_import

import copy
import threading
import time
from typing import Optional

import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from sensor_msgs_py import point_cloud2 as pc2
from tf_transformations import quaternion_matrix, quaternion_from_matrix, translation_from_matrix


class GlobalLocalizationNode(Node):
	def __init__(self) -> None:
		super().__init__('fast_lio_localization')

		# Parameters (declare and allow override via launch)
		self.declare_parameter('map2odom_completed', False)
		self.declare_parameter('region', 0)
		self.declare_parameter('freq_localization', 0.5)
		self.declare_parameter('localization_th', 0.997)
		self.declare_parameter('map_voxel_size', 0.2)
		self.declare_parameter('scan_voxel_size', 0.1)
		self.declare_parameter('fov', 6.28)
		self.declare_parameter('fov_far', 30.0)
		self.declare_parameter('map_frame', 'map3d')
		self.declare_parameter('odom_frame', 'camera_init')
		self.declare_parameter('base_link_frame', 'base_link')

		# Internal state
		self.global_map: Optional[o3d.geometry.PointCloud] = None
		self.initialized: bool = False
		self.T_map_to_odom: np.ndarray = np.eye(4)
		self.cur_odom: Optional[Odometry] = None
		self.cur_scan: Optional[o3d.geometry.PointCloud] = None

		# QoS profiles
		transient_local_qos = QoSProfile(depth=1)
		transient_local_qos.reliability = ReliabilityPolicy.RELIABLE
		transient_local_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

		reliable_qos = QoSProfile(depth=10)
		reliable_qos.reliability = ReliabilityPolicy.RELIABLE

		# Publishers
		self.pub_pc_in_map = self.create_publisher(PointCloud2, '/cur_scan_in_map', 1)
		self.pub_submap = self.create_publisher(PointCloud2, '/submap', 1)
		self.pub_map_to_odom = self.create_publisher(Odometry, '/map_to_odom', 1)
		self.pub_initialpose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', transient_local_qos)

		# Subscriptions
		self.create_subscription(PointCloud2, '/cloud_registered', self.cb_save_cur_scan, 1)
		self.create_subscription(Odometry, '/odom', self.cb_save_cur_odom, 1)
		self.create_subscription(PointCloud2, '/map3d', self.cb_init_global_map, reliable_qos)

		# Kickoff initialization thread (waits for map and initial pose, then starts periodic localization)
		self._init_thread = threading.Thread(target=self.initialize_and_run, daemon=True)
		self._init_thread.start()

		self.get_logger().info('Localization Node Inited (ROS2) ...')

	def get_param(self, name: str):
		return self.get_parameter(name).get_parameter_value()

	def pose_to_mat(self, odom_or_pose_msg) -> np.ndarray:
		p = odom_or_pose_msg.pose.pose.position
		q = odom_or_pose_msg.pose.pose.orientation
		T = quaternion_matrix([q.x, q.y, q.z, q.w])
		T[:3, 3] = [p.x, p.y, p.z]
		return T

	def msg_to_array(self, pc_msg: PointCloud2) -> np.ndarray:
		# Read x,y,z and optional intensity robustly regardless of field ordering
		has_intensity = any(f.name == 'intensity' for f in pc_msg.fields)
		if has_intensity:
			points = list(pc2.read_points(pc_msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True))
			if len(points) == 0:
				return np.zeros((0, 4), dtype=np.float64)
			return np.asarray(points, dtype=np.float64)
		else:
			points = list(pc2.read_points(pc_msg, field_names=('x', 'y', 'z'), skip_nans=True))
			if len(points) == 0:
				return np.zeros((0, 3), dtype=np.float64)
			return np.asarray(points, dtype=np.float64)

	def array_to_msg(self, frame_id: str, stamp, pc: np.ndarray) -> PointCloud2:
		if pc.size == 0:
			header = Header()
			header.frame_id = frame_id
			header.stamp = stamp
			return pc2.create_cloud_xyz32(header, [])
		
		header = Header()
		header.frame_id = frame_id
		header.stamp = stamp
		if pc.shape[1] >= 4:
			fields = [
				pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
				pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
				pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
				pc2.PointField(name='intensity', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
			]
			points32 = pc[:, :4].astype(np.float32)
			return pc2.create_cloud(header, fields, points32.tolist())
		else:
			return pc2.create_cloud_xyz32(header, pc[:, :3].astype(np.float32).tolist())

	def voxel_down_sample(self, pcd: o3d.geometry.PointCloud, voxel_size: float) -> o3d.geometry.PointCloud:
		try:
			return pcd.voxel_down_sample(voxel_size)
		except Exception:
			return o3d.geometry.voxel_down_sample(pcd, voxel_size)

	def crop_global_map_in_FOV(self, global_map: o3d.geometry.PointCloud, pose_estimation: np.ndarray, cur_odom: Odometry) -> o3d.geometry.PointCloud:
		T_odom_to_base_link = self.pose_to_mat(cur_odom)
		T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
		T_base_link_to_map = self.inverse_se3(T_map_to_base_link)

		global_map_in_map = np.asarray(global_map.points)
		global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
		global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

		FOV = float(self.get_parameter('fov').value)
		FOV_FAR = float(self.get_parameter('fov_far').value)

		if FOV > 3.14:
			indices = np.where(
				(global_map_in_base_link[:, 0] < FOV_FAR) &
				(np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
			)
		else:
			indices = np.where(
				(global_map_in_base_link[:, 0] > 0) &
				(global_map_in_base_link[:, 0] < FOV_FAR) &
				(np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
			)

		global_map_in_FOV = o3d.geometry.PointCloud()
		global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

		header_stamp = self.get_clock().now().to_msg()
		map_frame = self.get_parameter('map_frame').value
		msg = self.array_to_msg(map_frame, header_stamp, np.asarray(global_map_in_FOV.points)[::10])
		self.pub_submap.publish(msg)
		return global_map_in_FOV

	def inverse_se3(self, trans: np.ndarray) -> np.ndarray:
		trans_inverse = np.eye(4)
		trans_inverse[:3, :3] = trans[:3, :3].T
		trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
		return trans_inverse

	def publish_initial_pose(self) -> None:
		pose_msg = PoseWithCovarianceStamped()
		pose_msg.header.frame_id = self.get_parameter('map_frame').value
		pose_msg.header.stamp = self.get_clock().now().to_msg()
		pose_msg.pose.pose.position.x = 0.0
		pose_msg.pose.pose.position.y = 0.0
		pose_msg.pose.pose.position.z = 0.0
		pose_msg.pose.pose.orientation.w = 1.0
		pose_msg.pose.pose.orientation.x = 0.0
		pose_msg.pose.pose.orientation.y = 0.0
		pose_msg.pose.pose.orientation.z = 0.0
		self.pub_initialpose.publish(pose_msg)

	def registration_at_scale(self, pc_scan: o3d.geometry.PointCloud, pc_map: o3d.geometry.PointCloud, initial: np.ndarray, scale: float):
		result_icp = o3d.pipelines.registration.registration_icp(
			self.voxel_down_sample(pc_scan, float(self.get_parameter('scan_voxel_size').value) * scale),
			self.voxel_down_sample(pc_map, float(self.get_parameter('map_voxel_size').value) * scale),
			1.0 * scale,
			initial,
			o3d.pipelines.registration.TransformationEstimationPointToPoint(),
			o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
		)
		return result_icp.transformation, result_icp.fitness

	# Callbacks
	def cb_save_cur_odom(self, odom_msg: Odometry) -> None:
		self.cur_odom = odom_msg

	def cb_save_cur_scan(self, pc_msg: PointCloud2) -> None:
		# fastlio: scan is already in odom frame; publish passthrough for debug
		pc = self.msg_to_array(pc_msg)
		self.cur_scan = o3d.geometry.PointCloud()
		self.cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])

		# republish with frame override
		odom_frame = self.get_parameter('odom_frame').value
		header_stamp = self.get_clock().now().to_msg()
		msg = self.array_to_msg(odom_frame, header_stamp, pc)
		self.pub_pc_in_map.publish(msg)

	def cb_init_global_map(self, pc_msg: PointCloud2) -> None:
		if self.global_map is not None:
			return
		pc = self.msg_to_array(pc_msg)
		self.global_map = o3d.geometry.PointCloud()
		self.global_map.points = o3d.utility.Vector3dVector(pc[:, :3])
		self.global_map = self.voxel_down_sample(self.global_map, float(self.get_parameter('map_voxel_size').value))
		self.get_logger().info('Global map received.')

	# Logic
	def global_localization(self, pose_estimation: np.ndarray) -> bool:
		if self.global_map is None or self.cur_scan is None or self.cur_odom is None:
			return False

		self.get_logger().info('Global localization by scan-to-map matching ...')
		scan_tobe_mapped = copy.copy(self.cur_scan)
		start_t = time.time()

		global_map_in_FOV = self.crop_global_map_in_FOV(self.global_map, pose_estimation, self.cur_odom)
		transformation, _ = self.registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5.0)
		transformation, fitness = self.registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation, scale=1.0)
		self.get_logger().info('Time: {:.3f}'.format(time.time() - start_t))

		map2odom_completed = bool(self.get_parameter('map2odom_completed').value)
		region_id = int(self.get_parameter('region').value)

		if map2odom_completed or region_id in [2, 4, 5, 6, 7]:
			self.get_logger().warn(f'LOCALIZATION DISABLED: region={region_id}, map2odom_completed={map2odom_completed}')
			return True

		if fitness > float(self.get_parameter('localization_th').value):
			self.T_map_to_odom = transformation

			map_to_odom = Odometry()
			xyz = translation_from_matrix(self.T_map_to_odom)
			quat = quaternion_from_matrix(self.T_map_to_odom)
			map_to_odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
			map_to_odom.header.stamp = self.cur_odom.header.stamp
			map_to_odom.header.frame_id = self.get_parameter('map_frame').value
			self.pub_map_to_odom.publish(map_to_odom)
			self.get_logger().info('relocalization fitness: {:.6f}'.format(fitness))
			return True
		else:
			self.get_logger().warn('Not match (fitness={:.6f})'.format(fitness))
			return False

	def initialize_and_run(self) -> None:
		# wait global map
		while rclpy.ok() and self.global_map is None:
			self.get_logger().warn('Waiting for global map ...')
			time.sleep(0.5)

		# publish and wait initial pose
		self.publish_initial_pose()
		while rclpy.ok() and not self.initialized:
			self.get_logger().warn('Waiting for initial pose ...')
			# Instead of blocking wait_for_message, use the initial guess from current T_map_to_odom if scan exists
			if self.cur_scan is not None:
				self.initialized = self.global_localization(self.T_map_to_odom)
			else:
				self.get_logger().warn('First scan not received!')
			time.sleep(1.0)

		self.get_logger().info('Initialize successfully!')

		# periodic localization
		freq = float(self.get_parameter('freq_localization').value)
		period = 1.0 / max(1e-6, freq)
		while rclpy.ok():
			self.global_localization(self.T_map_to_odom)
			time.sleep(period)


def main():
	rclpy.init()
	node = GlobalLocalizationNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main() 