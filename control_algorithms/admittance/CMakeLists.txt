cmake_minimum_required(VERSION 3.0.2)
project(admittance)

## Compile as C++11, supported in ROS Kinetic and newer
## add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  cartesian_state_msgs
  roscpp
  sensor_msgs
  tf
  tf_conversions
  trajectory_msgs
  eigen_conversions
  joint_effort_msg
  joint_state_msg
  nav_msgs
  geometry_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED NO_MODULE)
list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots/")
find_package(pinocchio REQUIRED)



set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -Wall")

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES Admittance
 CATKIN_DEPENDS cartesian_state_msgs roscpp sensor_msgs tf tf_conversions trajectory_msgs eigen_conversions joint_effort_msg joint_state_msg nav_msgs geometry_msgs std_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${EIGEN3_INCLUDE_DIR})


add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/Admittance.h
  src/Admittance/Admittance.cpp
)
target_link_libraries(  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_library(BiQuad
  include/hybridadmittance/BiQuad.h
  src/HybridAdmittance/BiQuad.cpp
)
target_link_libraries(BiQuad
  ${catkin_LIBRARIES}
)

add_library(hybridadmittance
  include/hybridadmittance/HybridAdmittance.h
  src/HybridAdmittance/HybridAdmittance.cpp
)
target_link_libraries(hybridadmittance
  BiQuad ${catkin_LIBRARIES}
)
add_library(torque_hybridadmittance
  include/hybridadmittance/torque_HybridAdmittance.h
  src/HybridAdmittance/torque_HybridAdmittance.cpp
)
target_link_libraries(torque_hybridadmittance
  BiQuad Eigen3::Eigen pinocchio::pinocchio ${catkin_LIBRARIES}
)



add_executable(${PROJECT_NAME}_node src/Admittance/Admittance_node.cpp)
add_executable(hybridadmittance_node src/HybridAdmittance/HybridAdmittance_node.cpp)
add_executable(torque_hybridadmittance_node src/HybridAdmittance/torque_HybridAdmittance_node.cpp)
add_executable(reference_node src/HybridAdmittance/reference_node.cpp)
add_executable(wrench_signal_generate src/Admittance/WrenchSignalGenerate.cpp)

##target_link_libraries(torque_hybridadmittance pinocchio::pinocchio)



## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(hybridadmittance_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(torque_hybridadmittance_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(reference_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(BiQuad ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
target_link_libraries(hybridadmittance_node
  ${catkin_LIBRARIES}
  hybridadmittance
)
target_link_libraries(torque_hybridadmittance_node
  ${catkin_LIBRARIES}
  torque_hybridadmittance
  Eigen3::Eigen 
  pinocchio::pinocchio
)
target_link_libraries(reference_node
  ${catkin_LIBRARIES}
)
target_link_libraries(wrench_signal_generate
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

catkin_install_python(PROGRAMS 
  scripts/nokov_state_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

