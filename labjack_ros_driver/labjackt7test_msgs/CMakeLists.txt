cmake_minimum_required(VERSION 2.8.3)
project(labjackt7test_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation
)

add_message_files(
  FILES
  labjackt7test_channel.msg
  labjackt7test_streaming.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES labjackt7test_msgs
   CATKIN_DEPENDS std_msgs message_generation message_runtime
#  DEPENDS system_lib
)