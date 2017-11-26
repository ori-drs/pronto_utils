cmake_minimum_required(VERSION 2.8.3)
# needed for string arrays
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

if (NOT COMMAND pods_use_pkg_config_packages)
  include(cmake/pods.cmake)
endif()

project(pronto_filters)

find_package(catkin REQUIRED COMPONENTS
  pronto_vis
  pronto_math
  bot_core_lcmtypes
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS pronto_vis pronto_math
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

######################################################

add_library(${PROJECT_NAME} src/filter_cloudtolaser.cpp
  src/filter_light.cpp src/filter_planes.cpp src/grow_cloud.cpp)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
pods_use_pkg_config_packages(${PROJECT_NAME} bot2-core)


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
