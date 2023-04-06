CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(proto_ros2)

# ROS2 DEPENDENCIES
FIND_PACKAGE(ament_cmake REQUIRED)
FIND_PACKAGE(builtin_interfaces REQUIRED)
FIND_PACKAGE(eigen3_cmake_module REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(std_msgs REQUIRED)
FIND_PACKAGE(sensor_msgs REQUIRED)
FIND_PACKAGE(geometry_msgs REQUIRED)
FIND_PACKAGE(rclcpp REQUIRED)
FIND_PACKAGE(rclcpp_components REQUIRED)
FIND_PACKAGE(px4_msgs REQUIRED)
FIND_PACKAGE(message_filters REQUIRED)
FIND_PACKAGE(cv_bridge REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

# GAZEBO DEPENDENCIES
FIND_PACKAGE(gz-plugin2 QUIET REQUIRED COMPONENTS all)
FIND_PACKAGE(gz-common5 QUIET REQUIRED)
FIND_PACKAGE(gz-math7 QUIET REQUIRED)
FIND_PACKAGE(gz-sim7 QUIET REQUIRED)
SET(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
SET(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
SET(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})
SET(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

# INCLUDES
INCLUDE_DIRECTORIES(include ${OpenCV_INCLUDE_DIRS})

# EXECUTABLES
ADD_EXECUTABLE(offboard src/offboard.cpp)
AMENT_TARGET_DEPENDENCIES(offboard rclcpp px4_msgs)
INSTALL(TARGETS offboard DESTINATION lib/${PROJECT_NAME})

# ADD_EXECUTABLE(gz_gimbal src/gz_gimbal.cpp)
# AMENT_TARGET_DEPENDENCIES(
#   gz_gimbal
#   rclcpp
#   std_msgs
#   sensor_msgs
#   message_filters
#   cv_bridge
# )
# TARGET_LINK_LIBRARIES(gz_gimbal ${OpenCV_LIBS} apriltag)
# INSTALL(TARGETS gz_gimbal DESTINATION lib/${PROJECT_NAME})

# EXPORT DEPENDENCIES
AMENT_EXPORT_DEPENDENCIES(
	ament_cmake
	rclcpp
	rclcpp_components
	rosidl_default_runtime
	eigen3_cmake_module
	Eigen3
  std_msgs
	sensor_msgs
	geometry_msgs
	px4_msgs
  message_filters
  cv_bridge
)

# INSTALL LAUNCH FILES
INSTALL(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# INSTALL GAZEBO FILES
INSTALL(DIRECTORY gazebo DESTINATION share/${PROJECT_NAME}/)


# Build Plugins
SET(DEPS
    PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    PRIVATE gz-sim7::gz-sim7)

ADD_LIBRARY(HelloWorldPlugin SHARED gazebo/plugins/HelloWorldPlugin.cc)
TARGET_LINK_LIBRARIES(HelloWorldPlugin ${DEPS})
INSTALL(TARGETS HelloWorldPlugin DESTINATION share/${PROJECT_NAME}/gazebo/plugins)

ADD_LIBRARY(ModelPlugin SHARED gazebo/plugins/ModelPlugin.cc)
TARGET_LINK_LIBRARIES(ModelPlugin ${DEPS})
INSTALL(TARGETS ModelPlugin DESTINATION share/${PROJECT_NAME}/gazebo/plugins)


# FORM ROS2 Package
AMENT_PACKAGE()
