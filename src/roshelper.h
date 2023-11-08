#pragma once

#include <cstddef>
#include <iterator>

#include "ros/init.h"
#include "ros/node_handle.h"
#ifndef ROS_FOUND
#include <ros/ros.h>
#endif

#include <iostream>
#include <string>

#include "common.h"
#include "cv_bridge/cv_bridge.h"

namespace RosHelper {

using Common::CamConfig;
using Common::IOType;
using std::cerr;

struct RosHelper {
  void initNode(int argc, char** argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    node_name_ = node_name;
    return;
  }

  CamConfig parseConfig() {
    nh_ = ros::NodeHandle();
    ros::NodeHandle param("~");

    int width, height, io_type;
    std::string dev_name, cam_name;
    param.param<std::string>("dev", dev_name, "/dev/video0");
    param.param<std::string>("name", cam_name, "/ud_camera1");
    param.param<std::int32_t>("io_type", io_type, 0);
    param.param<std::int32_t>("width", width, 1920);
    param.param<std::int32_t>("height", height, 1080);

    if ((io_type < 0) || (io_type > 2)) {
      cerr << "ros launch config io_type is " << io_type
           << " not in range (0, 1, 2)"
           << "\n";
    }

    return CamConfig{.cam_name = cam_name,
                     .dev_name = dev_name,
                     .width = width,
                     .height = height,
                     .io_type = static_cast<IOType>(io_type)};
  };

  [[noreturn]] void doSpin() {
    while (ros::ok()) {
      ros::spin();
    }
  }

  void pubCVImg(unsigned char* data, size_t data_size, timeval captured_ts) {
    static auto pub_color =
        this->getNodeHandle().advertise<cv_bridge::CvImage>("ud_camera0", 1);
    static CamConfig config = this->parseConfig();

    //将 RGB 图像按照格式进行存取
    sensor_msgs::Image msg;
    msg.header.seq = 1;
    msg.header.frame_id = "ud_camera";
    msg.header.stamp.sec = captured_ts.tv_sec;
    msg.header.stamp.nsec = captured_ts.tv_usec * 1000;

    msg.width = config.width;
    msg.height = config.height;
    msg.encoding = "bgra8";
    msg.is_bigendian = 0;
    msg.step = config.width * 2;
    msg.data = std::vector<unsigned char>(data, data + data_size);
    pub_color.publish(msg);
    return;
  }

  inline ros::NodeHandle getNodeHandle() { return nh_; }

 private:
  ros::NodeHandle nh_;
  std::string node_name_;
};

}  // namespace RosHelper