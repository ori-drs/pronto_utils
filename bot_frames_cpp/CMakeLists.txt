cmake_minimum_required(VERSION 2.8.3)
# needed for string arrays
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

project(bot_frames_cpp)

find_package(catkin REQUIRED COMPONENTS
  bot_core bot_core_lcmtypes bot_frames)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS 
    bot_frames
  LIBRARIES bot_frames_cpp
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

######################################################
add_library(${PROJECT_NAME} src/bot_frames_cpp.cpp)


#############
## Install ##
#############

install(TARGETS ${TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
