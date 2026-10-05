cmake_minimum_required(VERSION 3.8)
project(cakebot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(amcl_path_publisher
  src/amcl_path_publisher.cpp
)

target_compile_features(amcl_path_publisher PRIVATE cxx_std_17)

ament_target_dependencies(amcl_path_publisher
  geometry_msgs
  nav_msgs
  rclcpp
)

# Launch files, parameter/configuration resources, and the AMCL path node are
# installed here. Runtime dependencies are declared in package.xml.
install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS amcl_path_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/localization_smoke_test.py
  DESTINATION lib/${PROJECT_NAME}
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
