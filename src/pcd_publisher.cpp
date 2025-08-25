#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PcdPublisherNode : public rclcpp::Node {
public:
	PcdPublisherNode()
		: rclcpp::Node("pcd_publisher") {
		this->declare_parameter<std::string>("map", "");
		this->declare_parameter<std::string>("frame_id", "map3d");
		this->declare_parameter<double>("rate", 5.0);

		rclcpp::QoS qos(1);
		qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
		qos.transient_local();
		publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map3d", qos);

		std::string path = this->get_parameter("map").as_string();
		if (path.empty()) {
			RCLCPP_ERROR(this->get_logger(), "Invalid PCD path: (empty)");
		} else {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
			if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1) {
				RCLCPP_ERROR(this->get_logger(), "Failed to load PCD: %s", path.c_str());
			} else {
				cloud_ = cloud;
				RCLCPP_INFO(this->get_logger(), "Loaded PCD: %s with %zu points", path.c_str(), cloud_->points.size());
			}
		}

		double rate = this->get_parameter("rate").as_double();
		if (rate <= 0.0) rate = 1.0;
		timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / rate), std::bind(&PcdPublisherNode::onTimer, this));
	}

private:
	void onTimer() {
		if (!cloud_) {
			// publish empty
			sensor_msgs::msg::PointCloud2 msg;
			msg.header.frame_id = this->get_parameter("frame_id").as_string();
			msg.header.stamp = this->now();
			publisher_->publish(msg);
			return;
		}
		sensor_msgs::msg::PointCloud2 msg;
		pcl::toROSMsg(*cloud_, msg);
		msg.header.frame_id = this->get_parameter("frame_id").as_string();
		msg.header.stamp = this->now();
		publisher_->publish(msg);
	}

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<PcdPublisherNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
} 