cmake_minimum_required(VERSION 3.8)
project(cpdwc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# add_executable(dwa src/dwa.cpp)

add_library(cpdwc 
  src/cpdwc.cpp
  src/dwa.cpp
  src/state_lattice_planner.cpp
)




add_executable(controller 
  src/main.cpp
)

add_executable(teleop src/itav_agv_teleop.cpp)
ament_target_dependencies(teleop 
  rclcpp 
  geometry_msgs
  )

ament_target_dependencies(cpdwc
  rclcpp 
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  cartographer_ros_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  )


install(TARGETS
teleop
DESTINATION lib/${PROJECT_NAME})

target_link_libraries(cpdwc 
  ${Boost_SYSTEM_LIBRARY}
  ${EIGEN3_LIBRARIES}
)


install(TARGETS
  cpdwc
  controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
    launch
    params
  DESTINATION 
    share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/transform.py
  DESTINATION 
  lib/${PROJECT_NAME}
)



# Export include directories
ament_export_include_directories(include)

ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  cartographer_ros_msgs
)

target_link_libraries(controller
  cpdwc
)


ament_package()