cmake_minimum_required(VERSION 3.5)
project(fast_lio_localization_ros2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# C++ deps
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS io filters registration)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(pcd_publisher src/pcd_publisher.cpp)
ament_target_dependencies(pcd_publisher rclcpp sensor_msgs std_msgs pcl_conversions)

target_include_directories(pcd_publisher PRIVATE ${PCL_INCLUDE_DIRS})
ament_export_include_directories(include)

target_link_libraries(pcd_publisher ${PCL_LIBRARIES})

add_executable(transform_fusion src/transform_fusion.cpp)
ament_target_dependencies(transform_fusion rclcpp geometry_msgs nav_msgs tf2_ros)

target_link_libraries(transform_fusion Eigen3::Eigen)

add_executable(global_localization src/global_localization.cpp)
ament_target_dependencies(global_localization rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs pcl_conversions)

target_include_directories(global_localization PRIVATE ${PCL_INCLUDE_DIRS})

target_link_libraries(global_localization ${PCL_LIBRARIES} Eigen3::Eigen)

install(TARGETS
	pcd_publisher
	transform_fusion
	global_localization
	DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
	scripts/global_localization_ros2.py
	scripts/transform_fusion_ros2.py
	scripts/pcd_publisher.py
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
	launch rviz_cfg
	DESTINATION share/${PROJECT_NAME}
)

ament_package() 