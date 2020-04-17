cmake_minimum_required(VERSION 2.8.3)
project(labjackt7test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

set(LabJackM_INCLUDE_DIRS /usr/local/lib/include)
set(LabJackM_LIBRARIES
LabJackM
)

add_message_files(
  FILES
  labjackt7test_channel.msg
  labjackt7test_streaming.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_generation message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LabJackM_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_connection_test
  src/labjack_t7_connection_test.cpp

  src/labjack_serial_test.cpp
  include/labjackt7test/labjack_serial_test.h
)
target_link_libraries(${PROJECT_NAME}_connection_test
  #${catkin_LIBRARIES}
  ${LabJackM_LIBRARIES}
)

add_executable(${PROJECT_NAME}_driver_test
  src/labjack_t7_driver_test.cpp

  src/labjack_t7_driver.cpp
  include/labjackt7test/labjack_t7_driver.h
)
target_link_libraries(${PROJECT_NAME}_driver_test
  ${catkin_LIBRARIES}
  ${LabJackM_LIBRARIES}
)
 
add_executable(${PROJECT_NAME}_node
  src/labjack_t7_node.cpp

  src/labjack_t7_driver.cpp
  include/labjackt7test/labjack_t7_driver.h

  src/labjack_t7_ros.cpp
  include/labjackt7test/labjack_t7_ros.h
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${LabJackM_LIBRARIES}
)

add_executable(${PROJECT_NAME}_kill
  src/labjack_close_device.cpp
)
target_link_libraries(${PROJECT_NAME}_kill
  ${LabJackM_LIBRARIES}
)
