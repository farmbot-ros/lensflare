cmake_minimum_required(VERSION 3.15)
project(genicam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wall -Wextra -Wpedantic -Wunknown-pragmas)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

include(FetchContent)
FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

include_directories(
  include
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
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
  lifecycle_msgs
  rclcpp_lifecycle
  sensor_msgs
  cv_bridge
  image_transport
  harvester_interfaces
  arena_sdk
  OpenCV
  fmt
)

add_executable(camera_info
  src/CameraInfo.cpp
)
ament_target_dependencies(camera_info ${ros_dependencies})


if (FALSE)
  add_executable(camera_manager
    src/CameraManager.cpp
    src/CameraNode.cpp
  )
  ament_target_dependencies(camera_manager ${ros_dependencies})
  install(TARGETS
    camera_info
    camera_manager
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  add_executable(camera_node
    src/CameraNode.cpp
  )
  ament_target_dependencies(camera_node ${ros_dependencies})
  # target_link_options(camera_node PRIVATE -Wl,-e,main_cn)
  install(TARGETS
    camera_info
    camera_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    DIRECTORY configs
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)


ament_package()
