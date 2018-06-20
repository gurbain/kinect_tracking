cmake_minimum_required(VERSION 2.8.3)
project(tigrillo_tracking)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  rospy
  std_msgs
)

# add_service_files(DIRECTORY srv
#                   FILES
# 		  		  	blabla.srv)
# 
# add_message_files(DIRECTORY msg
# 				  FILES
# 				  blabla.msg
# )

catkin_python_setup()

# generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS bin/tracker
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



