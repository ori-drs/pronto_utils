cmake_minimum_required(VERSION 2.8.3)
# needed for string arrays
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

if (NOT COMMAND pods_use_pkg_config_packages)
  include(cmake/pods.cmake)
endif()

project(pronto_lidar_filters)

find_package(catkin REQUIRED COMPONENTS
  pronto_math
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS pronto_math
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
add_library(${PROJECT_NAME} src/lidar_filters.cpp)
#configure_file(pronto_vis_config.h.in pronto_vis_config.h)
#add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} pronto_math)
#target_link_libraries(${PROJECT_NAME} pcl_common)
pods_use_pkg_config_packages(${PROJECT_NAME} lcmtypes_bot2-core)



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
