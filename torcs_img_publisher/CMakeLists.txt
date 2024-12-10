cmake_minimum_required(VERSION 3.8)
project(torcs_img_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)

if(OpenCV_VERSION VERSION_LESS "3.0")
  # use 2.4 modules
else()
  # use 3.x modules
endif()

include_directories(
    include
)

add_executable(${PROJECT_NAME} src/torcs_img_publisher.cpp)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_geometry
    image_transport
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${image_geometry_INCLUDE_DIRS}
    ${image_transport_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} 
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${image_geometry_INCLUDE_DIRS}
    ${image_transport_INCLUDE_DIRS}
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
