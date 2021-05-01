cmake_minimum_required(VERSION 2.8.3)
project(hydrophones)

find_package(catkin REQUIRED COMPONENTS
  rospy
  genmsg
  std_msgs
  geometry_msgs
)

catkin_python_setup()

add_message_files(FILES
    Signal.msg
    SignalArrayStamped.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
  rospy
)

# catkin_add_nosetests(test)
