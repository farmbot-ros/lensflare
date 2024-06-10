cmake_minimum_required(VERSION 3.8)
project(harvester_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraDevice.msg"
  "msg/CameraDeviceArray.msg"
  "srv/CreateCamera.srv"
  "srv/TriggerCamera.srv"
  "srv/TriggerFlash.srv"
)

ament_package()
