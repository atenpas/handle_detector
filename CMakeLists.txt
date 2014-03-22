cmake_minimum_required(VERSION 2.8.3)
project(handle_detector)

## find catkin and required packages
find_package(catkin REQUIRED COMPONENTS 
	eigen_conversions 
  geometry_msgs 
	message_generation 
  roscpp 
	pcl_ros
  pcl_conversions
  std_msgs 
	tf
	tf_conversions
  visualization_msgs)

## find Lapack
find_package(LAPACK REQUIRED)

## find Eigen package
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

## compiler optimization flags
set(CMAKE_CXX_FLAGS "-DNDEBUG -O3 -fopenmp")

## add messages
add_message_files(
  FILES
  # localization related messages
  CylinderMsg.msg
  CylinderArrayMsg.msg
  HandleListMsg.msg
)

generate_messages(DEPENDENCIES geometry_msgs)

## Declare a catkin package
catkin_package(
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    eigen_conversions 
    geometry_msgs 
    message_generation 
    roscpp 
    pcl_ros
    pcl_conversions
    std_msgs 
    tf
    tf_conversions
    visualization_msgs
 DEPENDS 
  Eigen
  LAPACK)

include_directories(${catkin_INCLUDE_DIRS})

## create executables
add_executable(localization src/localization.cpp)

## create libraries
add_library(affordances_lib src/affordances.cpp)
add_library(visualizer_lib src/visualizer.cpp)
add_library(messages_lib src/messages.cpp)
add_library(cylindrical_shell_lib src/cylindrical_shell.cpp)

## add dependencies
add_dependencies(messages_lib ${PROJECT_NAME}_gencpp)

## link libraries to localization executable
target_link_libraries(localization ${catkin_LIBRARIES})
target_link_libraries(localization affordances_lib)
target_link_libraries(localization visualizer_lib)
target_link_libraries(localization messages_lib)

## link libraries to affordances library
target_link_libraries(affordances_lib ${catkin_LIBRARIES})
target_link_libraries(affordances_lib cylindrical_shell_lib)
target_link_libraries(affordances_lib lapack)

## link libraries to messages library
target_link_libraries(messages_lib ${catkin_LIBRARIES})

## install targets
install(TARGETS localization
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
