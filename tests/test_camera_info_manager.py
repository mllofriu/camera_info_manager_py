#!/usr/bin/env python

PKG='camera_info_manager_py'
import roslib; roslib.load_manifest(PKG)

import sys
import os
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
g_default_camera_name = "axis_camera"

# initialized ${ROS_HOME} for testing
g_ros_home = "/tmp"
os.environ["ROS_HOME"] = g_ros_home
g_default_yaml = g_ros_home + "/camera_info/" + g_default_camera_name + ".yaml"


def delete_file(filename):
    """ Delete a file, not complaining if it does not exist.

    :param filename: path to file.
    """
    try:
        os.remove(filename)
    except OSError:             # OK if file did not exist
        pass

def expected_calibration():
    """ These data must match the contents of test_calibration.yaml."""

    ci = CameraInfo()
    ci.width = 640
    ci.height = 480

    # set distortion coefficients
    ci.distortion_model = "plumb_bob"
    ci.D = [-1.04482, 1.59252, -0.0196308, 0.0287906, 0.0]

    # set camera matrix
    ci.K = [1168.68, 0., 295.015, 0., 1169.01, 252.247, 0., 0., 1.]

    # set rectification matrix
    ci.R = [1., 0., 0., 0., 1., 0., 0., 0., 1.]

    # set projection matrix
    ci.P = [1168.68, 0., 295.015, 0., 0., 1169.01, 252.247, 0., 0., 0., 1., 0.]

    return ci

class TestCameraInfoManager(unittest.TestCase):
    """Unit tests for Python camera_info_manager.
    """

    # camera name tests

    def test_valid_camera_names(self):
        """Test that valid camera names are accepted."""
        cinfo = CameraInfoManager()

        # a list of valid names to try:
        names = ["a", "1", "_",
                 "A1", "9z",
                 "abcdefghijklmnopqrstuvwxyz",
                 "ABCDEFGHIJKLMNOPQRSTUVWXYZ",
                 "0123456789",
                 "0123456789abcdef",
                 "axis_00408c8ae301_local"
                 "08144361026320a0_640x480_mono8"]
        for cn in names:
            self.assertTrue(cinfo.setCameraName(cn))
            self.assertEqual(cinfo.getCameraName(), cn)

    def test_invalid_camera_names(self):
        """Test that invalid camera names are rejected."""
        cinfo = CameraInfoManager()

        # a list of invalid names to try:
        names = ["", "-21", "C++",
                 "axis-00408c8ae301.local",
                 "file:///tmp/url.yaml",
                 "file://${INVALID}/xxx.yaml"]
        for cn in names:
            self.assertFalse(cinfo.setCameraName(cn))
            self.assertEqual(cinfo.getCameraName(), g_default_camera_name)

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

    # URL parsing and validation

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
        """ Test invalid URL parsing."""

        self.assertEqual(parseURL("file://"), URL_invalid)
        self.assertEqual(parseURL("flash:///"), URL_invalid)
        self.assertEqual(parseURL("html://ros.org/wiki/camera_info_manager"),
                         URL_invalid)
        self.assertEqual(parseURL("package://"), URL_invalid)
        self.assertEqual(parseURL("package:///"), URL_invalid)
        self.assertEqual(parseURL("package://calibration.yaml"), URL_invalid)
        self.assertEqual(parseURL("package://camera_info_manager_py/"),
                         URL_invalid)

    def test_get_package_filename(self):
        """ Test getPackageFileName() function."""

        # resolve known file in this package
        filename = getPackageFileName(g_package_url)
        pkgPath = roslib.packages.get_pkg_dir(g_package_name)
        expected_filename = pkgPath + g_package_filename
        self.assertEqual(filename, expected_filename)

        # resolve non-existent package
        filename = getPackageFileName("package://no_such_package/"
                                      + g_package_filename)
        self.assertEqual(filename, "")

    # calibration data handling

    def test_get_missing_info(self):
        """ Test ability to detect missing CameraInfo."""
        cinfo = CameraInfoManager()
        self.assertRaises(CameraInfoMissingError, cinfo.isCalibrated)
        self.assertRaises(CameraInfoMissingError, cinfo.getCameraInfo)

    def test_load_calibration_file(self):
        """ Test loadCalibrationFile() function. """

        # try with an actual file in this directory
        pkgPath = roslib.packages.get_pkg_dir(g_package_name)
        filename = pkgPath + g_package_filename
        ci = loadCalibrationFile(filename, g_default_camera_name)
        self.assertEqual(ci, expected_calibration())

        # an empty file should return a null calibration
        filename = pkgPath + "/tests/empty.yaml"
        ci = loadCalibrationFile(filename, g_default_camera_name)
        self.assertEqual(ci, CameraInfo())

        # a non-existent file should return a null calibration
        delete_file(g_default_yaml)
        ci = loadCalibrationFile(g_default_yaml, g_default_camera_name)
        self.assertEqual(ci, CameraInfo())

    def test_get_uncalibrated_info(self):
        """ Test ability to provide uncalibrated CameraInfo"""
        delete_file(g_default_yaml)
        cinfo = CameraInfoManager()
        cinfo.loadCameraInfo()
        self.assertFalse(cinfo.isCalibrated())
        self.assertEqual(cinfo.getCameraInfo(), CameraInfo())

    def test_get_calibrated_info(self):
        """ Test ability to provide calibrated CameraInfo"""
        cinfo = CameraInfoManager(url=g_package_url)
        cinfo.loadCameraInfo()
        self.assertTrue(cinfo.isCalibrated())
        self.assertEqual(cinfo.getCameraInfo(), expected_calibration())

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_camera_info_manager', TestCameraInfoManager) 
