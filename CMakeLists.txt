cmake_minimum_required(VERSION 2.8.3)
project(camera_info_manager_py)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_python_setup()
catkin_package(CATKIN_DEPENDS sensor_msgs)

# Unit test uses nose, but needs rostest to create a ROS environment.
if (CATKIN_ENABLE_TESTING)
  find_package(roscpp REQUIRED)
  find_package(camera_info_manager REQUIRED)
  add_executable(generate_camera_info
    tests/generate_camera_info.cpp
    )
  # TODO(lucasw) Can the roscpp and camera_info_manager dependencies
  # get added to catkin_LIBRARIES somehow?
  target_link_libraries(generate_camera_info ${catkin_LIBRARIES}
      ${roscpp_LIBRARIES} ${camera_info_manager_LIBRARIES})

  find_package(rostest REQUIRED)

  add_rostest(tests/load_cpp_camera_info.test DEPENDENCIES generate_camera_info)
  add_rostest(tests/unit_test.test)
endif(CATKIN_ENABLE_TESTING)
