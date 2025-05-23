cmake_minimum_required(VERSION 3.5)
project(picamera_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
# find_package(px4_msgs REQUIRED)
ament_auto_find_build_dependencies()

# libcamera
pkg_check_modules(LIBCAMERA REQUIRED libcamera)
include_directories(include ${LIBCAMERA_INCLUDE_DIRS})

# picamera_pub ======================================
set(TARGET picamera_pub)
ament_auto_add_library(${TARGET} SHARED src/${TARGET}.cpp)
rclcpp_components_register_nodes(${TARGET} "picamera_ros::PiCameraROS")
# ===================================================================

# picamera_pub_exec ======================================
ament_auto_add_executable(${TARGET}_exec src/${TARGET}.cpp)
ament_target_dependencies(${TARGET}_exec 
    cv_bridge
    vision_msgs
)
# ===================================================================

# # recorder ======================================
# set(RECORD_TARGET recorder)
# ament_auto_add_library(${RECORD_TARGET} SHARED src/${RECORD_TARGET}.cpp)
# rclcpp_components_register_nodes(${RECORD_TARGET} "Recorder")
# # ===================================================================

# # picamera_pub_exec ======================================
# ament_auto_add_executable(${RECORD_TARGET}_exec src/${RECORD_TARGET}.cpp)
# ament_target_dependencies(${RECORD_TARGET}_exec
#     std_msgs 
#     rosbag2_cpp
#     px4_msgs
# )
# ===================================================================

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
