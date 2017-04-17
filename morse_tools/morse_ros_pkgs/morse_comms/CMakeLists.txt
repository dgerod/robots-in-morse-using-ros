cmake_minimum_required(VERSION 2.8.3)
project(morse_msgs)

find_package(catkin REQUIRED COMPONENTS
             roscpp rospy std_msgs geometry_msgs sensor_msgs message_generation)

add_service_files(FILES PlaceIkTarget.srv PlaceJoints.srv
    MoveToPose.srv FindObject.srv GetObjects.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)
