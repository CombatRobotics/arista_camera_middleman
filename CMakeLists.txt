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
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

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

add_executable(focuser_test src/test_focus.cpp)
target_include_directories(focuser_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(focuser_test PRIVATE i2c)

add_executable(rpicam_test src/test_rpicamera.cpp)
target_include_directories(rpicam_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rpicam_test
  rclcpp sensor_msgs cv_bridge 
)
target_link_libraries(rpicam_test ${OpenCV_LIBS} ) 

add_executable(auto_focus_node src/test_autofocus.cpp)
target_include_directories(auto_focus_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(auto_focus_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(auto_focus_node ${OpenCV_LIBS})


# Install the executable
install(TARGETS can_comm test_handshake rpicam_test focuser_test 
  auto_focus_node
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
