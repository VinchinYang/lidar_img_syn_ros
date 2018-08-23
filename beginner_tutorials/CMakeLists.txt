cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin 
    REQUIRED COMPONENTS 
    roscpp 
    rospy 
    std_msgs 
    genmsg
    pcl_ros
    cv_bridge
    )

## Declare ROS messages and services
add_message_files(FILES Num.msg marker_6dof.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS}) 

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
target_link_libraries(listener ${PCL_LIBRARIES})
target_link_libraries(listener /opt/ros/kinetic/lib/libmessage_filters.so)
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
