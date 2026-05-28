cmake_minimum_required(VERSION 3.0.2)
project(ur_robot_driver)

add_definitions( -DROS_BUILD )

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    geometry_msgs
    roscpp
    sensor_msgs
    std_srvs
    tf
    tf2_geometry_msgs
    tf2_msgs
    kdl_parser
    rpwc_msgs
    rpwc
)
find_package(Boost REQUIRED)
find_package(ur_client_library REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    urcl_log_handler
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    kdl_parser
    roscpp
    sensor_msgs
    std_srvs
    tf
    tf2_geometry_msgs
    tf2_msgs
    rpwc_msgs
    rpwc
  DEPENDS
    Boost
    ur_client_library
)

add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(urcl_log_handler
  src/urcl_log_handler.cpp
)
target_link_libraries(urcl_log_handler ${catkin_LIBRARIES} ur_client_library::urcl)

add_executable(rpwc_ur_bridge_native src/rpwc_ur_bridge_native.cpp src/io_manager.cpp src/robot_state_manager.cpp)
add_dependencies(rpwc_ur_bridge_native ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rpwc_ur_bridge_native ${catkin_LIBRARIES} ur_client_library::urcl urcl_log_handler)

install(TARGETS urcl_log_handler 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
