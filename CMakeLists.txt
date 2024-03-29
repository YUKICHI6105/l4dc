cmake_minimum_required(VERSION 3.8)
project(l4dc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_plugins2 REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(l4dc3_node src/Leg_for_DCmoter3.cpp)
add_executable(l4dc4_node src/Leg_for_DCmoter4.cpp)
add_executable(robomaster_test_node src/robomaster_test.cpp)
ament_target_dependencies(l4dc3_node rclcpp std_msgs can_plugins2 sensor_msgs)
ament_target_dependencies(l4dc4_node rclcpp std_msgs can_plugins2 sensor_msgs)
ament_target_dependencies(robomaster_test_node rclcpp std_msgs can_plugins2 sensor_msgs)

target_include_directories(l4dc3_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(l4dc4_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(robomaster_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_export_include_directories(include)

target_compile_features(l4dc3_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(l4dc4_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(robomaster_test_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(
  TARGETS l4dc3_node l4dc4_node robomaster_test_node
  DESTINATION lib/${PROJECT_NAME} 
)  

# Install launch files.
install(DIRECTORY
  l4dc_launch config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
