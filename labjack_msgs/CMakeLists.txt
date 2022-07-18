cmake_minimum_required(VERSION 3.0.2)
project(labjack_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation
)

add_message_files(
  FILES
  labjack_channel.msg
  labjack_stream.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES(labjack_msgs)
   CATKIN_DEPENDS std_msgs message_generation message_runtime
#  DEPENDS system_lib
)