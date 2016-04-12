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

  # Run generate_camera_info with env ROS_HOME set to pwd
  # or if not pwd somewhere else where the load_cpp_camera_info.py
  # will be able to find it (TODO(lucasw)).
  add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/camera_info/camera.yaml
                    COMMAND ROS_HOME=${CMAKE_CURRENT_BINARY_DIR} rosrun camera_info_manager_py generate_camera_info
                    DEPENDS generate_camera_info)

  find_package(rostest REQUIRED)

  add_custom_target(camera_yaml DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/camera_info/camera.yaml)
  # TODO(lucasw) I want to have the rostest generate camera.yaml,
  # this does not.
  add_rostest(tests/load_cpp_camera_info.test DEPENDENCIES camera_yaml)
  add_rostest(tests/unit_test.test)
endif(CATKIN_ENABLE_TESTING)
