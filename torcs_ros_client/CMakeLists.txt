cmake_minimum_required(VERSION 3.8)
project(torcs_ros_client)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(torcs_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
    include
    common/include
)

set(SRC_FILES
    common/src/SimpleParser.cpp
    src/torcs_ros_client_node.cpp
)

#add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

add_executable(${PROJECT_NAME} ${SRC_FILES})

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    tf2
    tf2_ros
    tf2_sensor_msgs
    geometry_msgs
    torcs_interfaces
    std_msgs
)
target_include_directories(${PROJECT_NAME} PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_sensor_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${torcs_interfaces_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${tf2_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${tf2_sensor_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${torcs_interfaces_LIBRARIES}
)

install(DIRECTORY
    config
    launch
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
