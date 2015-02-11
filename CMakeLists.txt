cmake_minimum_required(VERSION 2.8.3)
project(trs_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  rosbag
  roscpp
  roslib
  std_msgs
  tf
  trs_msgs
  visualization_msgs
  leap_motion
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#find_package(Eigen REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  getMotorCmdSet.srv
  getAllCurrentCmd.srv
  getSlaveCurrentCmd.srv
  getSlaveLockCmd.srv
  getHeadPositionCmd.srv
  switchNode.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  trs_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES trs_control
 CATKIN_DEPENDS rosbag roscpp roslib std_msgs tf trs_msgs visualization_msgs leap_motion
 DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(/usr/include/eigen3)

## Declare a cpp library
# add_library(trs_control
#   src/${PROJECT_NAME}/trs_control.cpp
# )

#if(0)
## Declare a cpp executable
add_executable(trs_bagRecorderArmsAndAngles src/bagRecorderArmsAndAngles.cpp) 
add_executable(trs_bagRecorderArmsAndObject src/bagRecorderArmsAndObject.cpp)
add_executable(trs_bagRecorderHeadAndArms src/bagRecorderHeadAndArms.cpp)
add_executable(trs_bagRecorderLegs src/bagRecorderLegs.cpp)
add_executable(trs_bagRecorderPostures src/bagRecorderPostures.cpp)
#add_executable(trs_bagRecorderMotion src/bagRecorderMotion.cpp)

add_executable(trs_mergeBags src/mergeBags.cpp)

add_executable(trs_commandBuilder src/commandBuilder.cpp)
add_executable(trs_basicControl src/basicControl.cpp)
add_executable(trs_faceTracking src/faceTracking.cpp)
add_executable(trs_objectTracking src/objectTracking.cpp)
add_executable(trs_tabletCommand src/tabletCommand.cpp)
add_executable(trs_handshake src/handshake.cpp)
add_executable(trs_balanceLegs src/balanceLegs.cpp)

add_executable(trs_kdtreeAnglesToArms src/kdtreeAnglesToArms.cpp)  
add_executable(trs_kdtreeObjectToArms src/kdtreeObjectToArms.cpp) 
add_executable(trs_kdtreeHeadToArms src/kdtreeObjectToArms.cpp) 

add_executable(trs_randomPosturePlayer src/randomPosturePlayer.cpp)
add_executable(trs_randomLegsPlayer src/randomLegsPlayer.cpp)

add_executable(trs_conductor src/conductor.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(trs_control_node trs_control_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(trs_bagRecorderArmsAndAngles ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(trs_bagRecorderArmsAndObject ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(trs_bagRecorderHeadAndArms ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(trs_bagRecorderLegs ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(trs_bagRecorderPostures ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
#target_link_libraries(trs_bagRecorderMotion ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})

target_link_libraries(trs_mergeBags ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})

target_link_libraries(trs_commandBuilder ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(trs_basicControl ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS}) 
target_link_libraries(trs_faceTracking ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(trs_objectTracking ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(trs_tabletCommand ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(trs_handshake ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(trs_balanceLegs ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})

target_link_libraries(trs_kdtreeAnglesToArms ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS}) 
target_link_libraries(trs_kdtreeObjectToArms ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS}) 
target_link_libraries(trs_kdtreeHeadToArms ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS}) 

target_link_libraries(trs_randomPosturePlayer ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(trs_randomLegsPlayer ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})

target_link_libraries(trs_conductor ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
#endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS trs_control trs_control_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_trs_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
