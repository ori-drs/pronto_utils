cmake_minimum_required(VERSION 2.8.3)
# needed for string arrays
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

if (NOT COMMAND pods_use_pkg_config_packages)
  include(cmake/pods.cmake)
endif()

find_package(catkin REQUIRED COMPONENTS
  bot_core_lcmtypes)

project(bot2_frames_cpp)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES bot2_frames_cpp
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

######################################################

add_library(${PROJECT_NAME} src/bot_frames_cpp.cpp)

#add_dependencies(${PROJECT_NAME}
#  ${catkin_EXPORTED_TARGETS}
#)
pods_use_pkg_config_packages(${PROJECT_NAME} bot2-frames eigen3)


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
