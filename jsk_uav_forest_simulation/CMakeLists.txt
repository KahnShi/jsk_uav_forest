cmake_minimum_required(VERSION 2.8.3)
project(jsk_uav_forest_simulation)

set(PACKAGE_DEPENDENCIES
  roscpp
  std_msgs
  geometry_msgs
)
SET(CMAKE_CXX_FLAGS "-std=c++11")
find_package(cmake_modules REQUIRED)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread random)
find_package(catkin REQUIRED ${PACKAGE_DEPENDENCIES})

# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)


include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

add_library(gazebo_moving_object_plugin src/gazebo_moving_object_plugin.cpp)
target_link_libraries(gazebo_moving_object_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(uav_teleop_keyboard src/uav_teleop_keyboard.cpp)
target_link_libraries(uav_teleop_keyboard ${catkin_LIBRARIES})
