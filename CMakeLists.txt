# Make the Klamp't examples 
CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
PROJECT(TSDF2Mesh)


find_package(catkin REQUIRED 
COMPONENTS
  nodelet
  roscpp
  rospy
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  shape_msgs
  rviz_visual_tools
)


SET(KLAMPT_ROOT ../../../Klampt CACHE PATH "Klampt root directory")
SET (CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")
MESSAGE(STATUS "Looking for FindKlampt.cmake in ${CMAKE_MODULE_PATH}")
# COMPILER SETTINGS (default: Release) and flags
INCLUDE(CompilerSettings)
FIND_PACKAGE(Klampt REQUIRED)
ADD_DEFINITIONS(${KLAMPT_DEFINITIONS})
INCLUDE_DIRECTORIES(${KLAMPT_INCLUDE_DIRS})

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)


catkin_package(
#  INCLUDE_DIRS include 
#  LIBRARIES DisinfectNodelets
 CATKIN_DEPENDS  nodelet roscpp rospy std_msgs tf2_geometry_msgs tf2_ros shape_msgs rviz_visual_tools
)

include_directories(
# include
${catkin_INCLUDE_DIRS}
)


#IKDemo 
ADD_EXECUTABLE(${PROJECT_NAME}_node src/main.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_node ${KLAMPT_LIBRARIES} ${catkin_LIBRARIES})
ADD_DEPENDENCIES(${PROJECT_NAME}_node   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(${PROJECT_NAME}_node PRIVATE ${KLAMPT_INCLUDE_DIRS})
