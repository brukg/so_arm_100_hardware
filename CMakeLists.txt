cmake_minimum_required(VERSION 3.10)
project(so_arm_100_hardware)

# Add SCServo subdirectory
add_subdirectory(include/SCServo_Linux)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/so_arm_100_interface.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

target_link_libraries(${PROJECT_NAME}
  SCServo
  yaml-cpp::yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  std_srvs
)

pluginlib_export_plugin_description_file(hardware_interface so_arm_100_hardware.xml)




ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp hardware_interface pluginlib rclcpp_lifecycle std_srvs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add test executable
add_executable(test_servo
  test/test_servo.cpp
)

target_include_directories(test_servo 
  PRIVATE
  include/SCServo_Linux  # Direct path to SCServo headers
)

target_link_libraries(test_servo
  PRIVATE
  SCServo  # Link directly to SCServo library
)


add_executable(ping_test
  test/ping_test.cpp
)

target_include_directories(ping_test PRIVATE
  include
)

target_link_libraries(ping_test
  SCServo
)

install(
  DIRECTORY
    config
    include
    config/posses
  DESTINATION 
    share/${PROJECT_NAME}
)

install(
  TARGETS 
    ${PROJECT_NAME}
    test_servo
    ping_test
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS
    scripts/calibrate_arm.py
    scripts/zero_pose.py
    scripts/jog_joints.py
  DESTINATION 
    lib/${PROJECT_NAME}
)
ament_package()
