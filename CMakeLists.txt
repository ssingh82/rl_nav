cmake_minimum_required(VERSION 2.8.3)
project(rl_nav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  joy
  gazebo_ros
  tf

)

find_package(PCL REQUIRED)
find_package(gazebo REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()



###########
## Build ##
###########

add_service_files(DIRECTORY srv
  FILES
  ExpectedPath.srv
  QPrediction.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
# LIBRARIES rl_nav
#  CATKIN_DEPENDS message_runtime joy roscpp rospy std_msgs 
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS} )

set(CMAKE_CXX_FLAGS "-std=c++11")

add_executable(rl_nav src/main_controller.cpp src/JoystickNode.cpp src/PTAMLearner.cpp src/SarsaLearner.cpp src/Helper.cpp)
add_dependencies(rl_nav DEPS ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(rl_nav ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GAZEBO_LIBRARIES})