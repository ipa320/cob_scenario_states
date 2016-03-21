cmake_minimum_required(VERSION 2.8.3)
project(cob_generic_states)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

catkin_python_setup()

add_service_files(
  FILES
    GetOrder.srv
    TriggerTask.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package()
