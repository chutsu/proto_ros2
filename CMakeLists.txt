CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(proto_ros2)
SET(CMAKE_BUILD_TYPE Release)

# DEPENDENCIES
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
# -- ROS2 DEPENDENCIES
FIND_PACKAGE(ament_cmake REQUIRED)
FIND_PACKAGE(ament_cmake_python REQUIRED)
FIND_PACKAGE(builtin_interfaces REQUIRED)
FIND_PACKAGE(eigen3_cmake_module REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(realsense2 REQUIRED)
FIND_PACKAGE(std_msgs REQUIRED)
FIND_PACKAGE(sensor_msgs REQUIRED)
FIND_PACKAGE(geometry_msgs REQUIRED)
FIND_PACKAGE(rclpy REQUIRED)
FIND_PACKAGE(rclcpp REQUIRED)
FIND_PACKAGE(rclcpp_components REQUIRED)
FIND_PACKAGE(image_transport REQUIRED)
FIND_PACKAGE(message_filters REQUIRED)
FIND_PACKAGE(cv_bridge REQUIRED)
# -- PROTO DEPENDENCIES
FIND_PACKAGE(BLAS REQUIRED)
FIND_PACKAGE(LAPACK REQUIRED)

# INCLUDES AND LIB DIRECTORIES
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Eigen3_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${realsense2_INCLUDE_DIR})
INCLUDE_DIRECTORIES(/opt/yac/include)
INCLUDE_DIRECTORIES(/opt/proto/include)
LINK_DIRECTORIES(/opt/yac/lib)
LINK_DIRECTORIES(/opt/proto/lib)
LINK_DIRECTORIES(/usr/local/lib)

# EXPORT DEPENDENCIES
AMENT_EXPORT_DEPENDENCIES(
	ament_cmake
	rclcpp
	rclcpp_components
	rosidl_default_runtime
	eigen3_cmake_module
	Eigen3
  realsense2
  std_msgs
	sensor_msgs
	geometry_msgs
  message_filters
  cv_bridge
  image_transport
)

# ROS NODES
# -- SBGC NODE
ADD_EXECUTABLE(sbgc_node src/sbgc_node.cpp)
AMENT_TARGET_DEPENDENCIES(sbgc_node rclcpp std_msgs geometry_msgs)
INSTALL(TARGETS sbgc_node DESTINATION lib/${PROJECT_NAME})

# -- RealSense NODE
ADD_EXECUTABLE(rs_node src/rs_node.cpp)
TARGET_LINK_LIBRARIES(
  rs_node
  rclcpp::rclcpp
  image_transport::image_transport
  cv_bridge::cv_bridge
  realsense2
  ${OpenCV_LIBS}
)
INSTALL(TARGETS rs_node DESTINATION lib/${PROJECT_NAME})

# -- Calib Camera Record
ADD_EXECUTABLE(calib_camera_record src/calib_camera_record.cpp)
TARGET_LINK_LIBRARIES(
  calib_camera_record
  realsense2
  apriltags
  ${OpenCV_LIBS}
)
INSTALL(TARGETS calib_camera_record DESTINATION lib/${PROJECT_NAME})

# -- Calib Camera-IMU Record
ADD_EXECUTABLE(calib_camimu_record src/calib_camimu_record.cpp)
TARGET_LINK_LIBRARIES(
  calib_camimu_record
  realsense2
  apriltags
  ${OpenCV_LIBS}
)
INSTALL(TARGETS calib_camimu_record DESTINATION lib/${PROJECT_NAME})

# -- Calib Gimbal
ADD_EXECUTABLE(calib_gimbal_record src/calib_gimbal_record.cpp)
TARGET_LINK_LIBRARIES(
  calib_gimbal_record
  realsense2
  apriltags
  ${OpenCV_LIBS}
)
INSTALL(TARGETS calib_gimbal_record DESTINATION lib/${PROJECT_NAME})

# INSTALL LAUNCH FILES
INSTALL(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# FORM ROS2 Package
AMENT_PACKAGE()
