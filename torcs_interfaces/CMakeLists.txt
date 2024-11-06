cmake_minimum_required(VERSION 3.8)
project(torcs_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TORCSCtrl.msg
  msg/TORCSSensors.msg
  msg/BoolStamped.msg
  msg/Int8Stamped.msg
  msg/UInt32Stamped.msg
  msg/Float32Stamped.msg
  DEPENDENCIES
  builtin_interfaces
  std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
