cmake_minimum_required(VERSION 3.8)
project(rosbot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ros2_control REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
# Include Cpp “include” directory
#include_directories(include)

# Create Cpp executable and link with dependencies
#add_executable(cpp_executable src/cpp_node.cpp)
#ament_target_dependencies(cpp_executable rclcpp)

# Install Cpp executables in the ros2_ws/install/cpp_py_pkg/lib/cpp_py_pkg/ folder
#install(TARGETS
#cpp_executable
#DESTINATION lib/${PROJECT_NAME}
#)
# ===========================================#
#   for cpp file use above dependencies      #
#============================================#

# for python below dependencies#

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables in the ros2_ws/install/cpp_py_pkg/lib/cpp_py_pkg/ folder
install(PROGRAMS
  ${PROJECT_NAME}/twist_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY rosbot_controller launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
