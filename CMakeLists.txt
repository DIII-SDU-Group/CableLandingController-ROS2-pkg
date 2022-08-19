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

include_directories(src/pos_MPC)
link_directories(src/pos_MPC)

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

add_executable(cable_landing_controller_sw 
  src/cable_landing_controller_sw.cpp
  src/pos_MPC/addBoundToActiveSetMatrix_.cpp
  src/pos_MPC/checkStoppingAndUpdateFval.cpp
  src/pos_MPC/compute_deltax.cpp
  src/pos_MPC/computeFirstOrderOpt.cpp
  src/pos_MPC/computeFval.cpp
  src/pos_MPC/computeFval_ReuseHx.cpp
  src/pos_MPC/computeGrad_StoreHx.cpp
  src/pos_MPC/computeQ_.cpp
  src/pos_MPC/countsort.cpp
  src/pos_MPC/deleteColMoveEnd.cpp
  src/pos_MPC/driver.cpp
  src/pos_MPC/eye.cpp
  src/pos_MPC/factorQR.cpp
  src/pos_MPC/feasibleratiotest.cpp
  src/pos_MPC/feasibleX0ForWorkingSet.cpp
  src/pos_MPC/fullColLDL2_.cpp
  src/pos_MPC/iterate.cpp
  src/pos_MPC/linearForm_.cpp
  src/pos_MPC/loadProblem.cpp
  src/pos_MPC/maxConstraintViolation.cpp
  src/pos_MPC/modifyOverheadPhaseOne_.cpp
  src/pos_MPC/partialColLDL3_.cpp
  src/pos_MPC/phaseone.cpp
  src/pos_MPC/PositionMPCStepFunction.cpp
  src/pos_MPC/PositionMPCStepFunction_data.cpp
  src/pos_MPC/PositionMPCStepFunction_initialize.cpp
  src/pos_MPC/PositionMPCStepFunction_rtwutil.cpp
  src/pos_MPC/PositionMPCStepFunction_terminate.cpp
  src/pos_MPC/PresolveWorkingSet.cpp
  src/pos_MPC/quadprog.cpp
  src/pos_MPC/ratiotest.cpp
  src/pos_MPC/RemoveDependentEq_.cpp
  src/pos_MPC/RemoveDependentIneq_.cpp
  src/pos_MPC/rtGetInf.cpp
  src/pos_MPC/rtGetNaN.cpp
  src/pos_MPC/rt_nonfinite.cpp
  src/pos_MPC/setProblemType.cpp
  src/pos_MPC/xgeqp3.cpp
  src/pos_MPC/xnrm2.cpp
  src/pos_MPC/xrotg.cpp
  src/pos_MPC/xzgeqp3.cpp
  src/pos_MPC/xzlarf.cpp
  src/pos_MPC/xzlarfg.cpp

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
