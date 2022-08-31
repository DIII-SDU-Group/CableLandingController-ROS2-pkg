cmake_minimum_required(VERSION 3.5)
project(cable_landing_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(src/matlab_MPC)
link_directories(src/matlab_MPC)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(iii_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(cable_landing_controller_sw 
  src/cable_landing_controller_sw.cpp
  src/matlab_MPC/minOrMax.cpp
  src/matlab_MPC/mpcmoveCodeGeneration.cpp
  src/matlab_MPC/mpcmoveCodeGeneration_data.cpp
  src/matlab_MPC/mpcmoveCodeGeneration_initialize.cpp
  src/matlab_MPC/mpcmoveCodeGeneration_rtwutil.cpp
  src/matlab_MPC/mpcmoveCodeGeneration_terminate.cpp
  src/matlab_MPC/norm.cpp
  src/matlab_MPC/qpkwik.cpp
  src/matlab_MPC/rtGetInf.cpp
  src/matlab_MPC/rtGetNaN.cpp
  src/matlab_MPC/rt_nonfinite.cpp
  src/matlab_MPC/trisolve.cpp
  src/matlab_MPC/xgerc.cpp
  src/matlab_MPC/xnrm2.cpp
  src/matlab_MPC/xpotrf.cpp
)
ament_target_dependencies(cable_landing_controller_sw
  rclcpp
  Eigen3
  iii_interfaces
  sensor_msgs
  px4_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
install(TARGETS cable_landing_controller_sw DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# install the config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
