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
    FILES
    SpotServer_AddSpot.srv
    SpotServer_DeleteSpot.srv
    SpotServer_GetSpots.srv
)

add_message_files(
    FILES
    Spot.msg
    SpotArray.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
)

include_directories(
)

