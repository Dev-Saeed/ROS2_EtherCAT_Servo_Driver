cmake_minimum_required(VERSION 3.8)
project(servo_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

# Generate the service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetAcceleration.srv"
  "srv/SetKvp.srv"
)

# Ensure dependencies are set for generated interfaces
ament_export_dependencies(rosidl_default_runtime)

ament_package()
