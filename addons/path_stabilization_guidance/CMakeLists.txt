cmake_minimum_required(VERSION 3.1)
project(pf_guidance_system)

find_package(catkin
        # ROS components
        REQUIRED COMPONENTS
        rospy
        auv_msgs
        gazebo_msgs
        geometry_msgs
        medusa_msgs
        std_msgs)