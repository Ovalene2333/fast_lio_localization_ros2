#include <memory>
#include <optional>
#include <string>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

class TransformFusionNode : public rclcpp::Node {
public:
	TransformFusionNode() : rclcpp::Node("transform_fusion") {
		this->declare_parameter<double>("publish_rate", 100.0);
		this->declare_parameter<std::string>("map_frame", "map3d");
		this->declare_parameter<std::string>("odom_frame", "camera_init");
		this->declare_parameter<std::string>("base_link_frame", "base_link");

		pub_localization_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization", 10);
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		rclcpp::QoS qos(10);
		qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
		sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", qos, std::bind(&TransformFusionNode::cbSaveCurOdom, this, std::placeholders::_1));
		sub_map_to_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/map_to_odom", qos, std::bind(&TransformFusionNode::cbSaveMapToOdom, this, std::placeholders::_1));

		double publish_rate = this->get_parameter("publish_rate").as_double();
		if (publish_rate <= 0.0) publish_rate = 100.0;
		timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate), std::bind(&TransformFusionNode::onTimer, this));

		RCLCPP_INFO(this->get_logger(), "Transform Fusion Node Inited (C++) ...");
	}

private:
	static Eigen::Matrix4d poseToMat(const nav_msgs::msg::Odometry &odom) {
		const auto &p = odom.pose.pose.position;
		const auto &q = odom.pose.pose.orientation;
		Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
		Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
		T.block<3,3>(0,0) = quat.normalized().toRotationMatrix();
		T(0,3) = p.x;
		T(1,3) = p.y;
		T(2,3) = p.z;
		return T;
	}

	static Eigen::Matrix4d inverseSE3(const Eigen::Matrix4d &T) {
		Eigen::Matrix4d inv = Eigen::Matrix4d::Identity();
		inv.block<3,3>(0,0) = T.block<3,3>(0,0).transpose();
		inv.block<3,1>(0,3) = -inv.block<3,3>(0,0) * T.block<3,1>(0,3);
		return inv;
	}

	void onTimer() {
		const std::string map_frame = this->get_parameter("map_frame").as_string();
		const std::string odom_frame = this->get_parameter("odom_frame").as_string();
		const std::string base_link_frame = this->get_parameter("base_link_frame").as_string();

		Eigen::Matrix4d T_map_to_odom = Eigen::Matrix4d::Identity();
		if (cur_map_to_odom_) {
			T_map_to_odom = poseToMat(*cur_map_to_odom_);
		}

		// Publish TF map->odom
		auto now = this->now();
		geometry_msgs::msg::TransformStamped transform;
		transform.header.stamp = now;
		transform.header.frame_id = map_frame;
		transform.child_frame_id = odom_frame;
		transform.transform.translation.x = T_map_to_odom(0,3);
		transform.transform.translation.y = T_map_to_odom(1,3);
		transform.transform.translation.z = T_map_to_odom(2,3);
		Eigen::Quaterniond q(T_map_to_odom.block<3,3>(0,0));
		q.normalize();
		transform.transform.rotation.x = q.x();
		transform.transform.rotation.y = q.y();
		transform.transform.rotation.z = q.z();
		transform.transform.rotation.w = q.w();
		tf_broadcaster_->sendTransform(transform);

		if (cur_odom_to_baselink_) {
			Eigen::Matrix4d T_odom_to_base = poseToMat(*cur_odom_to_baselink_);
			Eigen::Matrix4d T_map_to_base = T_map_to_odom * T_odom_to_base;
			Eigen::Quaterniond q2(T_map_to_base.block<3,3>(0,0));
			q2.normalize();

			nav_msgs::msg::Odometry localization;
			localization.header.stamp = cur_odom_to_baselink_->header.stamp;
			localization.header.frame_id = map_frame;
			localization.child_frame_id = base_link_frame;
			localization.pose.pose.position.x = T_map_to_base(0,3);
			localization.pose.pose.position.y = T_map_to_base(1,3);
			localization.pose.pose.position.z = T_map_to_base(2,3);
			localization.pose.pose.orientation.x = q2.x();
			localization.pose.pose.orientation.y = q2.y();
			localization.pose.pose.orientation.z = q2.z();
			localization.pose.pose.orientation.w = q2.w();
			localization.twist = cur_odom_to_baselink_->twist;
			pub_localization_->publish(localization);
		}
	}

	void cbSaveCurOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
		cur_odom_to_baselink_ = *msg;
	}

	void cbSaveMapToOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
		cur_map_to_odom_ = *msg;
	}

	std::optional<nav_msgs::msg::Odometry> cur_map_to_odom_;
	std::optional<nav_msgs::msg::Odometry> cur_odom_to_baselink_;

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_localization_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_map_to_odom_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TransformFusionNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
} 