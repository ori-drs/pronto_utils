cmake_minimum_required(VERSION 2.8.3)
# needed for string arrays
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

if (NOT COMMAND pods_use_pkg_config_packages)
  include(cmake/pods.cmake)
endif()

project(pronto_vis)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pronto_math
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS pcl_ros pronto_math
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

######################################################
add_library(${PROJECT_NAME} src/pronto_vis.cpp)
#configure_file(pronto_vis_config.h.in pronto_vis_config.h)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} pronto_math)
target_link_libraries(${PROJECT_NAME} pcl_common)
pods_use_pkg_config_packages(${PROJECT_NAME} lcmtypes_visualization)


######################################################
add_executable(test-point-cloud src/test_point_cloud.cpp)
target_link_libraries(test-point-cloud pronto_vis ${catkin_LIBRARIES})

add_executable(test-point-cloud-pcl src/test_point_cloud_pcl.cpp)
target_link_libraries(test-point-cloud-pcl pronto_vis ${catkin_LIBRARIES} )

add_executable(test-point-cloud-pcl-2 src/test_point_cloud_pcl_2.cpp)
target_link_libraries(test-point-cloud-pcl-2 pronto_vis ${catkin_LIBRARIES} )

add_executable(test-point-cloud-normal src/test_point_cloud_normal.cpp)
target_link_libraries(test-point-cloud-normal pronto_vis ${catkin_LIBRARIES} )


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
