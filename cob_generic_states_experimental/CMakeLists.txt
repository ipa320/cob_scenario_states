cmake_minimum_required(VERSION 2.8.3)
project(cob_generic_states_experimental)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs)

catkin_python_setup()

add_service_files(
  FILES
    Door.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs
)

catkin_package()
