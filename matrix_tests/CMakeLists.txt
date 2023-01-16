cmake_minimum_required(VERSION 2.8.3)
project(serp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
  rospy
  message_generation 
  std_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_message_files(DIRECTORY msg 
  FILES
  Matrix.msg
)

generate_messages(DEPENDENCIES 
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include 
    CATKIN_DEPENDS roscpp sensor_msgs rospy message_runtime std_msgs
#    DEPENDS 
)

catkin_install_python(PROGRAMS src/vision_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########
## Build ##
###########

## Declare a C++ executable
add_executable(main_node src/main_node.cpp)
## Rename C++ executable without prefix
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(main_node PROPERTIES OUTPUT_NAME main_node PREFIX "")
## Add cmake target dependencies of the executable
add_dependencies(main_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(main_node ${catkin_LIBRARIES})

#############
## Install ##
#############

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(main_node ${OpenCV_LIBRARIES})
