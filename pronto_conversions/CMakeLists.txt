cmake_minimum_required(VERSION 2.8.3)
# needed for string arrays
add_compile_options(-std=c++11)

project(pronto_conversions)

find_package(catkin REQUIRED COMPONENTS pronto_math
                                        pronto_core
                                        bot_core_lcmtypes
                                        pronto_lcmtypes
                                        eigen_utils_lcmtypes)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS pronto_math
                              pronto_core
                              bot_core_lcmtypes
                              pronto_lcmtypes
                              eigen_utils_lcmtypes)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

######################################################

add_library(${PROJECT_NAME} src/pronto_conversions_lcm.cpp
                            src/pronto_meas_lcm.cpp)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

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
