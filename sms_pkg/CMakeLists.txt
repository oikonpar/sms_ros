cmake_minimum_required(VERSION 2.8.3)
project(sms_pkg)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation genmsg)

## Declare ROS messages and services
add_message_files(FILES control_all.msg control_with_id.msg)
add_service_files(FILES encoder_abspos.srv encoder_cur.srv encoder_pos.srv encoder_vel.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build follower
include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(sms_node src/sms_node.cpp)
#target_link_libraries(sms_node ${catkin_LIBRARIES})
#add_dependencies(sms_node sms_pkg_generate_messages_cpp)
#add_dependencies(sms_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
