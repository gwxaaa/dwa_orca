cmake_minimum_required(VERSION 3.0.2)
project(dwa_orca)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
test.msg
  #Message2.msg
 )
## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dwa_orca
  CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp rospy std_msgs  message_runtime
#  DEPENDS system_lib
)


include_directories(
include
  ${catkin_INCLUDE_DIRS} 
  ${gazebo_INCLUDE_DIRS}
 
)

#  add_executable(DWA_main_1  src/DWA_main_1.cpp  src/DWA.cpp src/KinematicModel.cpp )
#  target_link_libraries( DWA_main_1 ${catkin_LIBRARIES})
# install(TARGETS DWA_main_1  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

add_executable(pub_sub src/pub_sub.cpp src/ModelSubPub.cpp src/Agent.cpp  src/Vector2.cc   src/Line.cc src/Obstacle.cc src/Neighbor.cc
 src/KinematicModel.cpp src/DWA.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(pub_sub  ${catkin_LIBRARIES} )

# add_executable(DWA_main  src/DWA_main.cpp  src/DWA.cpp src/KinematicModel.cpp )
#add_executable(main_1  src/main_1.cpp  src/KinematicModel.cpp)
## Specify libraries to link a library or executable target against

# target_link_libraries( main_1 ${catkin_LIBRARIES})
# install(TARGETS main_1  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
# target_link_libraries( DWA_main ${catkin_LIBRARIES})
# install(TARGETS DWA_main  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


