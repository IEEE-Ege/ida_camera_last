cmake_minimum_required(VERSION 3.8)
project(ida_camera)

# Strict warnings in CI, ignored on MSVC
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── ROS2 dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# ── Third-party ──────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# ── Component library ────────────────────────────────────────────────────────
# A "component" is a shared library that the ROS2 component container can
# load at runtime without restarting. It also generates a standalone node
# executable via rclcpp_components_register_node below.
add_library(camera_processor_component SHARED
  src/camera_processor_node.cpp
)

target_include_directories(camera_processor_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(camera_processor_component
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  diagnostic_msgs
  cv_bridge
)

target_link_libraries(camera_processor_component
  ${OpenCV_LIBS}
  yaml-cpp
)

# Registers the class as a loadable component AND generates the
# camera_processor_node executable in one call.
rclcpp_components_register_node(camera_processor_component
  PLUGIN "ida_camera::CameraProcessorNode"
  EXECUTABLE camera_processor_node
)

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS camera_processor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
