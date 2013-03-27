cmake_minimum_required(VERSION 2.8.3)
project(camera_info_manager_py)

find_package(catkin REQUIRED COMPONENTS rospy rostest sensor_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_python_setup()
catkin_package(CATKIN_DEPENDS sensor_msgs)

# Unit test uses nose, but needs rostest to create a ROS environment.
add_rostest(tests/unit_test.test)
