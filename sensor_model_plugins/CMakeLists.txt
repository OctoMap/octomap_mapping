cmake_minimum_required(VERSION 2.8.3)
project(sensor_model_plugins)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  utbot_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sensor_model_plugins
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare Library
add_library(sensor_model_plugins
  plugins/gemini_model_plugin.cpp
  plugins/isa500_model_plugin.cpp
  plugins/me1512_model_plugin.cpp
)

add_dependencies(sensor_model_plugins
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(sensor_model_plugins
  ${catkin_LIBRARIES}
)

## Install
install(TARGETS sensor_model_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  sensor_model_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)