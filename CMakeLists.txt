cmake_minimum_required(VERSION 2.8.3)
project(ma_slam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# SET(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
#set(INSTALL_LIB_DIR lib CACHE PATH "/usr/local/include/opengv")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  pcl_conversions 
  pcl_ros
  genmsg
  image_geometry
  visualization_msgs
  #message_generation
  #opengv
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#FIND_LIBRARY(opengv_DIR opengv /usr/local/include/opengv)
#TARGET_LINK_LIBRARIES(opengv ${opengv_DIR})
# catkin_python_setup()
FIND_PACKAGE(OpenCV 2 REQUIRED)
LINK_DIRECTORIES( ${LINK_DIRECTORIES} /home/leo/opengv/lib)
find_package(PCL 1.2 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#find_package( opengv REQUIRED )
## Generate messages in the 'msg' folder
add_message_files( 
	FILES 
	keyframeMsg.msg
	keyframeGraphMsg.msg
	keyframeMsgStamped.msg
	keyframeGraphMsgStamped.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
   geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ma_slam
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs geometry_msgs pcl_conversions pcl_ros image_geometry visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  /home/leo/opengv/include
   ${CERES_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(ma_slam
#   src/${PROJECT_NAME}/ma_slam.cpp
# )

## Declare a cpp executable
#add_executable(ma_slam_node src/ma_slam_node.cpp src/CentralStorage.cpp src/RobotListener_Keyframes.cpp src/HelperFcts.cpp src/PtClListener.cpp src/PoseListener.cpp src/SyncListener.cpp src/random_generators.cpp src/experiment_helpers.cpp src/time_measurement.cpp)
add_executable(ma_slam src/ma_slam.cpp src/CentralStorage.cpp src/KeyFrameListenerTraining.cpp src/HelperFcts.cpp src/SyncListener.cpp src/KalmanFilterScale.cpp src/PoseGraph.cpp src/3dcov/cbshot.cpp)



## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(ma_slam_node ma_slam_generate_messages_cpp CentralStorage_cpp RobotListener_Keyframes_cpp HelperFcts_cpp PtClListener_cpp PoseListener_cpp SyncListener_cpp random_generators_cpp experiment_helpers_cpp time_measurement_cpp)
add_dependencies(ma_slam ma_slam_generate_messages_cpp CentralStorage_cpp KeyFrameListenerTraining_cpp HelperFcts_cpp SyncListener_cpp KalmanFilterScale_cpp cbshot_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(ma_slam ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} opengv ${CERES_LIBRARIES})
#ExtrMatchDescrCl RobotListener_Keyframes 

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
# install(TARGETS ma_slam ma_slam_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ma_slam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
