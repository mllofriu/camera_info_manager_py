#!/usr/bin/env python

PKG='camera_info_manager_py'
import roslib; roslib.load_manifest(PKG)

import sys
import unittest

import yaml

from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo
from camera_info_manager import *

class TestCameraInfoManager(unittest.TestCase):
    """Unit tests for Python camera_info_manager.
    """

    # camera name tests
    def test_valid_camera_names(self):
        """Test that valid camera names are accepted"""
        cinfo = CameraInfoManager()

        self.assertTrue(cinfo.setCameraName("a"))
        self.assertTrue(cinfo.setCameraName("1"))
        self.assertTrue(cinfo.setCameraName("_"))
        self.assertTrue(cinfo.setCameraName("abcdefghijklmnopqrstuvwxyz"))
        self.assertTrue(cinfo.setCameraName("ABCDEFGHIJKLMNOPQRSTUVWXYZ"))
        self.assertTrue(cinfo.setCameraName("0123456789"))
        self.assertTrue(cinfo.setCameraName("0123456789abcdef"))
        self.assertTrue(cinfo.setCameraName("A1"))
        self.assertTrue(cinfo.setCameraName("9z"))
        self.assertTrue(cinfo.setCameraName("08144361026320a0_640x480_mono8"))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_camera_info_manager', TestCameraInfoManager) 
