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
        """Test that valid camera names are accepted."""
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

    def test_invalid_camera_names(self):
        """Test that invalid camera names are rejected."""
        cinfo = CameraInfoManager()

        self.assertFalse(cinfo.setCameraName(""))
        self.assertFalse(cinfo.setCameraName("-21"))
        self.assertFalse(cinfo.setCameraName("C++"))
        self.assertFalse(cinfo.setCameraName("file:///tmp/url.yaml"))
        self.assertFalse(cinfo.setCameraName("file://${INVALID}/xxx.yaml"))

    def test_gen_camera_name(self):
        """Test camera name generation."""
        cinfo = CameraInfoManager()
        self.assertEqual(cinfo.genCameraName("a"), "a")
        self.assertEqual(cinfo.genCameraName("1"), "1")
        self.assertEqual(cinfo.genCameraName("_"), "_")
        self.assertEqual(cinfo.genCameraName(""), "_")
        self.assertEqual(cinfo.genCameraName("-21"), "_21")
        self.assertEqual(cinfo.genCameraName("C++"), "C__")
        self.assertEqual(cinfo.genCameraName("file:///tmp/url.yaml"),
                         "file____tmp_url_yaml")
        self.assertEqual(cinfo.genCameraName("file://${INVALID}/xxx.yaml"),
                         "file_____INVALID__xxx_yaml")
        self.assertEqual(cinfo.genCameraName("axis-00408c8ae301.local"),
                         "axis_00408c8ae301_local")

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_camera_info_manager', TestCameraInfoManager) 