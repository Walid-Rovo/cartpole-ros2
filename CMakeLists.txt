cmake_minimum_required(VERSION 3.5)
project(cartpole_01)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

# install(
#   PROGRAMS script/spawn_cartpole.py
#   DESTINATION lib/${PROJECT_NAME}
# )
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
