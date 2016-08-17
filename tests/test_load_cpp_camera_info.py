#!/usr/bin/env python

import unittest

from camera_info_manager import *
from sensor_msgs.srv import SetCameraInfo

class TestLoadCppCameraInfo(unittest.TestCase):

    def test_load_cpp_camera_info(self):
        rospy.wait_for_service('set_camera_info')
        set_camera_info = rospy.ServiceProxy('set_camera_info', SetCameraInfo)
        camera_info = CameraInfo()
        camera_info.width = 123
        camera_info.height = 456
        try:
            resp1 = set_camera_info(camera_info)
        except rospy.ServiceException as exc:
            rospy.logerr("service did not process request" + str(exc))
            self.assertTrue(False)
            return

        self.assertTrue(resp1.success)

        # TODO(lucasw) load the camera.yaml that the C++ camera_info_manager
        # generated, then check values against expected.
        # The values ought to be passed as args into both this test and the C++
        # camera info generation.
        loaded_camera_info = loadCalibrationFile("camera_info/camera.yaml", "camera")
        self.assertEquals(camera_info, loaded_camera_info)

if __name__ == '__main__':
    rospy.init_node('test_load_cpp_camera_info')
    import rostest
    rostest.rosrun('camera_info_manager_py', 'test_load_cpp_camera_info', TestLoadCppCameraInfo)
