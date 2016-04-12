/**
  Example of saving and loading a camera_info yaml using the C++ camera_info_manager
*/

#include <camera_info_manager/camera_info_manager.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

// The saving of calibration files is private, this is how
// that is bypassed.
// (from https://github.com/ros-industrial/human_tracker/blob/master/packages/camera_info_manager/tests/unit_test.cpp )
// issue SetCameraInfo service request
void set_calibration(ros::NodeHandle node,
                     const sensor_msgs::CameraInfo &calib)
{
  ros::ServiceClient client =
      node.serviceClient<sensor_msgs::SetCameraInfo>("set_camera_info");
  sensor_msgs::SetCameraInfo set_camera_info;
  set_camera_info.request.camera_info = calib;
  client.call(set_camera_info);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "generate_camera_info");
  ros::NodeHandle nh;
  camera_info_manager::CameraInfoManager cim(nh);

  ros::spin();
  // Can't call this private function
  // cim.saveCalibrationFile(camera_info, "cpp_cim_test.yaml", "camera");
  // this works but this node needs to just provide the service
  if (false)
  {
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sensor_msgs::CameraInfo camera_info;
    set_calibration(nh, camera_info);

    std::string url;
    ros::param::get("~url", url);
    cim.validateURL(url);
    cim.loadCameraInfo(url);
    // cim.saveCalibrationFile("cpp_cim_test2.yaml");
  }
}
