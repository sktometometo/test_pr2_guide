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
    DIRECTORY srv
    FILES
    GoToSpotCancel.srv
    GoToSpotStart.srv
    SpotManagerAdd.srv
    SpotManagerDelete.srv
    SpotManagerGet.srv
    SpotManagerSave.srv
    StatusManagerSetStatus.srv
)


add_message_files(
    DIRECTORY msg
    FILES 
    Spot.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs pr2_guide)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})


