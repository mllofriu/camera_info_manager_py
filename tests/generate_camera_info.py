#!/usr/bin/env python

import rospy
import camera_info_manager
import yaml

from sensor_msgs.msg import CameraInfo
# from sensor_msgs.srv import SetCameraInfo()

rospy.init_node("generate_camera_info_py")
ci = CameraInfo()
cname = "camera"
camera_info_manager.saveCalibrationFile(ci, "camera_info_manager_py.yaml", cname)

with open("camera_info_manager_py_yaml_dump.yaml", 'w') as f:
    calib = {'image_width': ci.width,
             'image_height': ci.height,
             'camera_name': cname,
             'distortion_model': ci.distortion_model,
             'distortion_coefficients': {'data': ci.D, 'rows': 1, 'cols': len(ci.D)},
             'camera_matrix': {'data': ci.K, 'rows': 3, 'cols': 3},
             'rectification_matrix': {'data': ci.R, 'rows': 3, 'cols': 3},
             'projection_matrix': {'data': ci.P, 'rows': 3, 'cols': 4}}

    rc = yaml.dump(calib, f)
    f.close()
