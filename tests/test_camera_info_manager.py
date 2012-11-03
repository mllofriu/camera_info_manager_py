#!/usr/bin/env python

PKG='camera_info_manager_py'
import roslib; roslib.load_manifest(PKG)

import sys
import unittest

#import yaml

#from sensor_msgs.msg import CameraInfo
#from sensor_msgs.srv import SetCameraInfo

from camera_info_manager import *

g_package_name = PKG
g_test_name = "test_calibration"
g_package_filename = "/tests/" + g_test_name +".yaml"
g_package_url = "package://" + g_package_name + g_package_filename
g_package_name_url = "package://" + g_package_name + "/tests/${NAME}.yaml"
g_default_url = "file://${ROS_HOME}/camera_info/${NAME}.yaml"
g_camera_name = "08144361026320a0"

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

        # valid strings pass through unchanged
        self.assertEqual(genCameraName("a"), "a")
        self.assertEqual(genCameraName("1"), "1")
        self.assertEqual(genCameraName("_"), "_")
        self.assertEqual(genCameraName("0123456789abcdef"), "0123456789abcdef")
        self.assertEqual(genCameraName("08144361026320a0_640x480_mono8"),
                         "08144361026320a0_640x480_mono8")

        # invalid strings get '_' substitution
        self.assertEqual(genCameraName(""), "_")
        self.assertEqual(genCameraName("-21"), "_21")
        self.assertEqual(genCameraName("C++"), "C__")
        self.assertEqual(genCameraName("file:///tmp/url.yaml"),
                         "file____tmp_url_yaml")
        self.assertEqual(genCameraName("file://${INVALID}/xxx.yaml"),
                         "file_____INVALID__xxx_yaml")
        self.assertEqual(genCameraName("axis-00408c8ae301.local"),
                         "axis_00408c8ae301_local")

    def test_url_resolution(self):
        """Test URL variable resolution."""
        cn = "axis_camera"

        # strings with no variables pass through unchanged
        self.assertEqual(resolveURL("file:///tmp/url.yaml", cn),
                         "file:///tmp/url.yaml")
        self.assertEqual(resolveURL(g_package_url, cn), g_package_url)
        self.assertEqual(resolveURL("", cn), "")

        # :todo: test variable substitution (when implemented)

    def test_valid_url_parsing(self):
        """Test valid URL parsing."""

        self.assertEqual(parseURL(""), URL_empty)

        self.assertEqual(parseURL("file:///"), URL_file)
        self.assertEqual(parseURL("file:///tmp/url.yaml"), URL_file)
        self.assertEqual(parseURL("FILE:///tmp/url.yaml"), URL_file)

        self.assertEqual(parseURL(g_package_url), URL_package)
        self.assertEqual(parseURL("packAge://camera_info_manager/x"),
                         URL_package)
        self.assertEqual(parseURL("package://no_such_package/calibr.yaml"),
                         URL_package)

    def test_invalid_url_parsing(self):
        """Test invalid URL parsing."""

        self.assertEqual(parseURL("file://"), URL_invalid)
        self.assertEqual(parseURL("flash:///"), URL_invalid)
        self.assertEqual(parseURL("html://ros.org/wiki/camera_info_manager"),
                         URL_invalid)
        self.assertEqual(parseURL("package://"), URL_invalid)
        self.assertEqual(parseURL("package:///"), URL_invalid)
        self.assertEqual(parseURL("package://calibration.yaml"), URL_invalid)
        self.assertEqual(parseURL("package://camera_info_manager_py/"),
                         URL_invalid)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_camera_info_manager', TestCameraInfoManager) 
