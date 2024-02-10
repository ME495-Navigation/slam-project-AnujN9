cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)        # If no build type is set, set one
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Type of build." FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # find_package(Catch2 3 REQUIRED)
  #   add_executable(test_turtlelib tests/test_geometry2d.cpp tests/test_se2d.cpp tests/test_svg.cpp tests/test_diff_drive.cpp)
  #   target_link_libraries(test_turtlelib Catch2::Catch2WithMain turtlelib) # AnyOtherLibrariesAsNeeded)
  #   add_test(NAME tests_for_turtlelib COMMAND test_turtlelib)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}_srv
  "srv/InitialPose.srv"
  "srv/Control.srv"
  LIBRARY_NAME ${PROJECT_NAME}
  DEPENDENCIES
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_srv "rosidl_typesupport_cpp")

add_executable(turtle_control src/turtle_control.cpp)
target_link_libraries(turtle_control turtlelib::turtlelib)
ament_target_dependencies(turtle_control rclcpp nuturtlebot_msgs geometry_msgs sensor_msgs)

add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry turtlelib::turtlelib "${cpp_typesupport_target}")
ament_target_dependencies(odometry rclcpp sensor_msgs nav_msgs tf2 tf2_ros geometry_msgs)

add_executable(circle src/circle.cpp)
target_link_libraries(circle turtlelib::turtlelib "${cpp_typesupport_target}")
ament_target_dependencies(circle rclcpp std_srvs geometry_msgs)

install(TARGETS
  turtle_control
  odometry
  circle
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  tests
  src
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
