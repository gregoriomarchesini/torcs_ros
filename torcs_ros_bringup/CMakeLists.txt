cmake_minimum_required(VERSION 3.5)
project(torcs_ros_bringup)

find_package(ament_cmake REQUIRED)
find_package(torcs_ros_client REQUIRED)
find_package(torcs_ros_drive_ctrl REQUIRED)
find_package(torcs_img_publisher REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
#install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
