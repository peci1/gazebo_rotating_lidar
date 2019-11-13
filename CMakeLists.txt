cmake_minimum_required(VERSION 2.8.3)
project(gazebo_rotating_lidar)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_plugins
  roscpp
  sensor_msgs
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS gazebo_custom_sensor_preloader gazebo_dev gazebo_plugins roscpp sensor_msgs
)

include_directories(
  include
)
include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
  src/RotatingLidarSensor.cpp
  src/ScanIterator.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall)

add_library(gazebo_ros_rotating_lidar src/RotatingLidarRosPlugin.cpp)
add_dependencies(gazebo_ros_rotating_lidar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_rotating_lidar ${PROJECT_NAME} RayPlugin ${catkin_LIBRARIES})

if(${CATKIN_ENABLE_TESTING})
  catkin_add_gtest(test_scan_iterator test/test_scan_iterator.cpp)
  target_link_libraries(test_scan_iterator ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES sensors.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})