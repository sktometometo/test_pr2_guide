cmake_minimum_required(VERSION 2.8.3)
project(pr2_guide)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation
)

add_service_files(
    DIRECTORY srv/SpotServer
    FILES
    SpotServer_AddSpot.srv
    SpotServer_DeleteSpot.srv
    SpotServer_GetSpots.srv
    SpotServer_SaveFile.srv
)

add_service_files(
    DIRECTORY srv/StatusBasedActionServer
    FILES
    StatusBasedActionServer_GetStatus.srv
    StatusBasedActionServer_SetStatus.srv
)

add_service_files(
    DIRECTORY srv/StatusBasedMotionServer
    FILES
    StatusBasedMotionServer_GetStatus.srv
    StatusBasedMotionServer_SetStatus.srv
)

add_message_files(
    DIRECTORY msg/Spot
    FILES 
    Spot.msg
    SpotArray.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs pr2_guide)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})


