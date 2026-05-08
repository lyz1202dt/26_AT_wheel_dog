cmake_minimum_required(VERSION 3.8)
project(remote_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(serial REQUIRED)
find_package(Threads REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Create executable
add_executable(remote_node
  src/main.cpp
  src/remote_node.cpp
  src/remote_comm.cpp
)

add_executable(keys_cpp
  src/keys_control.cpp
)

target_include_directories(keys_cpp
PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

# Include directories
target_include_directories(remote_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(remote_node
  rclcpp
  robot_interfaces
  serial
  geometry_msgs
  sensor_msgs
  tf2
)

ament_target_dependencies(keys_cpp
    rclcpp
    robot_interfaces
)
# Link system libraries
target_link_libraries(remote_node
  Threads::Threads
)

# Install targets
install(TARGETS
  remote_node
  keys_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/keys_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keys_py   # 重命名为 keys_py
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
