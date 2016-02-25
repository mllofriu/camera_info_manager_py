cmake_minimum_required(VERSION 2.8.3)
project(camera_info_manager_py)

find_package(catkin REQUIRED COMPONENTS camera_info_manager roscpp rospy sensor_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_python_setup()
catkin_package(CATKIN_DEPENDS sensor_msgs)

add_executable(generate_camera_info
  tests/generate_camera_info.cpp
  )
target_link_libraries(generate_camera_info ${catkin_LIBRARIES})

# Unit test uses nose, but needs rostest to create a ROS environment.
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/unit_test.test)
endif(CATKIN_ENABLE_TESTING)
