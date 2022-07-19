cmake_minimum_required(VERSION 3.0.2)
project(labjack_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  labjack_msgs
)

set(LabJackM_INCLUDE_DIRS /usr/local/lib/include)
set(LabJackM_LIBRARIES
LabJackM
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs labjack_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LabJackM_INCLUDE_DIRS}
)
 
add_executable(${PROJECT_NAME}_node
  src/labjack_ros_node.cpp

  src/labjack_ros_driver.cpp
  include/labjack_ros/labjack_ros_driver.h

  src/labjack_ros.cpp
  include/labjack_ros/labjack_ros.h
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${LabJackM_LIBRARIES}
)

add_executable(${PROJECT_NAME}_kill
  src/labjack_ros_close_device.cpp
)
target_link_libraries(${PROJECT_NAME}_kill
  ${LabJackM_LIBRARIES}
)

add_executable(${PROJECT_NAME}_test
  src/labjack_ros_test.cpp
  
  src/labjack_ros_driver.cpp
  include/labjack_ros/labjack_ros_driver.h

  src/labjack_ros.cpp
  include/labjack_ros/labjack_ros.h
)
target_link_libraries(${PROJECT_NAME}_test
  ${catkin_LIBRARIES}
  ${LabJackM_LIBRARIES}
)