cmake_minimum_required(VERSION 3.5)
project(cartpole_01)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(controller src/cartpole_controller.cpp)
ament_target_dependencies(controller rclcpp std_msgs sensor_msgs)

install(
  TARGETS controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch urdf sdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
