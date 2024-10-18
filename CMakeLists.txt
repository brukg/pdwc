cmake_minimum_required(VERSION 3.8)
project(pdwc)

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



# add_executable(dwa src/dwa.cpp)

add_executable(controller 
  src/pdwc.cpp
  src/dwa.cpp
  src/state_lattice_planner.cpp
  )


add_executable(teleop src/itav_agv_teleop.cpp)
ament_target_dependencies(teleop 
  rclcpp 
  geometry_msgs
  )

ament_target_dependencies(controller
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

target_link_libraries(controller 
  ${Boost_SYSTEM_LIBRARY}
  ${EIGEN3_LIBRARIES}
  )

install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()