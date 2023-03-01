cmake_minimum_required(VERSION 3.8)
project(itav_agv_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(OpenCV REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(
  DIRECTORY 
    launch
    params
  DESTINATION 
    share/${PROJECT_NAME})
install(PROGRAMS
  scripts/itav_agv_controller.py
  DESTINATION lib/${PROJECT_NAME})



# add_executable(dwa src/dwa.cpp)
# target_link_libraries(dwa ${OpenCV_LIBS} )

# install(TARGETS
#   dwa
#   DESTINATION lib/${PROJECT_NAME}
# )
ament_package()

add_executable(teleop src/itav_agv_teleop.cpp)
ament_target_dependencies(teleop rclcpp geometry_msgs)
install(TARGETS
teleop
DESTINATION lib/${PROJECT_NAME})