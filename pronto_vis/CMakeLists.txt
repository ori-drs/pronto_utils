cmake_minimum_required(VERSION 2.8.3)
project(pronto_vis)

if (NOT COMMAND pods_use_pkg_config_packages)
  include(cmake/pods.cmake)
endif()

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pronto_math
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES pronto_vis
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