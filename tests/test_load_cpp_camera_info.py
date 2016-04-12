#!/usr/bin/env python

import unittest

from camera_info_manager import *


class TestLoadCppCameraInfo(unittest.TestCase):

    def test_load_cpp_camera_info(self):
        # TODO(lucasw) load the camera.yaml that the C++ camera_info_manager
        # generated, then check values against expected.
        # The values ought to be passed as args into both this test and the C++
        # camera info generation.
        ci = loadCalibrationFile("camera_info/camera.yaml", "camera")
        print ci
        self.assertTrue(True)

if __name__ == '__main__':
    rospy.init_node('test_load_cpp_camera_info')
    import rostest
    rostest.rosrun('camera_info_manager_py', 'test_load_cpp_camera_info', TestLoadCppCameraInfo)
