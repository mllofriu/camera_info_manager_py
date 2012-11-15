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

rospy.init_node("service_test_node")

cinfo = CameraInfoManager()

# spin in the main thread: required for service callbacks
rospy.spin()
