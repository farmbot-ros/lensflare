cmake_minimum_required(VERSION 3.15)
project(farmbot_lensflare)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()
cmake_policy(SET CMP0074 NEW)
# set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/genicam" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(FetchContent)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

set(executables
    main
)

# set(GENICAM_INSTALL_LIBDIRS lib/genicam)
# set(GENICAM_INSTALL_INCLUDEDIRS include/genicam)
#
add_subdirectory(genicam)
# find_package(Genicam REQUIRED NO_CMAKE_PATH PATHS ${CMAKE_CURRENT_SOURCE_DIR}/genicam CONFIG NO_DEFAULT_PATH)
# add_library(${PROJECT_NAMESPACE}::genicam ALIAS genicam)
# include_directories(${Genicam_INCLUDES})
# install(DIRECTORY genicam/library/CPP/include/ DESTINATION include/genicam)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(farmbot_interfaces REQUIRED)



set(dependencies_ros
    rclcpp
    std_msgs
    std_srvs
    nav_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    tf2
    tf2_ros
    message_filters
    pcl_ros
    pcl_conversions
    diagnostic_msgs
    diagnostic_updater
    farmbot_interfaces
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ARAVIS REQUIRED aravis-0.8)


find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
link_directories(${Eigen_LIBRARY_DIRS})
add_definitions(${Eigen_DEFINITIONS})
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)
#find curl


set(dependencies_sys
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Eigen_LIBRARIES}
    ${ARAVIS_LIBRARIES}
    ${OpenCV_LIBRARIES}
    genicam
    nlohmann_json::nlohmann_json
)

foreach(executable ${executables})
    add_executable(${executable} src/${executable}.cpp)
    ament_target_dependencies(${executable} ${dependencies_ros})
    target_link_libraries(${executable} ${dependencies_sys})
endforeach()

install(TARGETS
    ${executables}
    DESTINATION lib/${PROJECT_NAME}
)

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${ARAVIS_INCLUDE_DIRS}
    ${nlohmann_json_INCLUDE_DIRS}
)


ament_package()
