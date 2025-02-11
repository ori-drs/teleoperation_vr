cmake_minimum_required(VERSION 3.0.2)
project(teleoperation_vr)
find_package(catkin REQUIRED)
# catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS 
roscpp 
rospy
std_msgs
geometry_msgs
message_generation)


add_message_files(
  FILES
  controller.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
)

catkin_package(
CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
)