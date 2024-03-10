cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/WaypointAction.action"
  DEPENDENCIES std_msgs geometry_msgs # CRITICAL
)

# build
add_executable(tortoisebot_action_server src/tortoisebot_action_server.cpp)
ament_target_dependencies(tortoisebot_action_server rclcpp rclcpp_action nav_msgs geometry_msgs)
rosidl_target_interfaces(tortoisebot_action_server ${PROJECT_NAME} "rosidl_typesupport_cpp") # FIX

# install
install(TARGETS tortoisebot_action_server DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
    test/main_test.cpp
    test/tortoisebot_action_server_test.cpp
  )

  # build test
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp rclcpp_action nav_msgs geometry_msgs)
  rosidl_target_interfaces(${PROJECT_NAME}_test ${PROJECT_NAME} "rosidl_typesupport_cpp")
  
  # install test
  install(TARGETS ${PROJECT_NAME}_test DESTINATION lib/${PROJECT_NAME})

  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
