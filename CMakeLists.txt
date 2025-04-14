cmake_minimum_required(VERSION 3.8)
project(arista_camera_middleman)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(can_comm src/test_handshake.cpp src/CanDevice.cpp)
target_include_directories(can_comm PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  include
)
ament_target_dependencies(can_comm rclcpp std_msgs)

# Add the test_handshake executable
add_executable(test_handshake src/test_handshake.cpp src/CanDevice.cpp)
target_include_directories(test_handshake PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(test_handshake
  "rclcpp"
)

# Install the executable
install(TARGETS can_comm test_handshake
    DESTINATION lib/${PROJECT_NAME})

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
