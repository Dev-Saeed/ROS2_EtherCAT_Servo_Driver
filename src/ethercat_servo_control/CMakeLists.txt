cmake_minimum_required(VERSION 3.8)
project(ethercat_servo_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(soem REQUIRED)


add_executable(motor_control_node src/motor_control_node.cpp)
ament_target_dependencies(motor_control_node rclcpp geometry_msgs soem)

include_directories(include)
target_link_libraries(motor_control_node soem)

# Install the executable
install(TARGETS
  motor_control_node
  DESTINATION lib/${PROJECT_NAME}
  )

<<<<<<< HEAD
# Add setcap command as a post-install step //Didnt work
# install(CODE "execute_process(COMMAND sudo setcap cap_net_raw+ep ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/motor_control_node)")
# install(CODE "execute_process(COMMAND sudo setcap cap_net_admin,cap_net_raw=eip ~/ros2_ws/Servant_ws/install/ethercat_servo_control/lib/ethercat_servo_control/motor_control_node)")


=======
>>>>>>> 3a50146f5e52bca35aa463bec3988bebebd4e223
install(
  DIRECTORY include
  DESTINATION include
  )

ament_package()
