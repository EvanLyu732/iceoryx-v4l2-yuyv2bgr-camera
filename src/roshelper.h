#ifndef ROSHELPER_H_
#define ROSHELPER_H_

#include <ros/ros.h>
#include <memory>
#include <string>
#include "common.h"

namespace RosHelper {

class RosHelper {
 public:
  RosHelper() = default;
  ~RosHelper() = default;

  /**
   * @brief Parse camera configuration from ROS parameters
   * @return CamConfig object containing camera settings
   */
  Common::CamConfig parseConfig() {
    ros::NodeHandle nh("~");
    
    std::string cam_name, dev_name;
    int width, height, io_method;

    // Get parameters from ROS parameter server
    nh.param<std::string>("cam_name", cam_name, "camera_0");
    nh.param<std::string>("dev_name", dev_name, "/dev/video0");
    nh.param<int>("width", width, 640);
    nh.param<int>("height", height, 480);
    nh.param<int>("io_method", io_method, 0);  // default to MMAP

    return Common::CamConfig{
      .cam_name = cam_name,
      .dev_name = dev_name,
      .width = width,
      .height = height,
      .io_type = static_cast<Common::IOType>(io_method)
    };
  }

 private:
  ros::NodeHandle nh_;
};

}  // namespace RosHelper

#endif  // ROSHELPER_H_ 