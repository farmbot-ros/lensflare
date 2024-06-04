cmake_minimum_required(VERSION 3.15)
project(genicam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wall -Wextra -Wpedantic -Wunknown-pragmas)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")


include_directories(
  include
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(arena_sdk REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(fmt REQUIRED)
find_package(harvester_interfaces REQUIRED)

set(ros_dependencies
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  cv_bridge
  image_transport
  harvester_interfaces
  arena_sdk
  OpenCV
  fmt
)


add_executable(camera_node
  src/CameraNode.cpp
)

add_executable(camera_info
  src/CameraInfo.cpp
)

ament_target_dependencies(camera_node ${ros_dependencies})
ament_target_dependencies(camera_info ${ros_dependencies})


install(TARGETS
  camera_node
  camera_info
  DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)


ament_package()
