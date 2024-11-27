#include <unistd.h>

#include <boost/program_options.hpp>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include "common.h"
#include "roshelper.h"
#include "v4l2cam.h"

/*
 * pass config to startup cam node
 * cam_node --type ros // then startup as rosnode, read config from launch file
 * cam_node --type native --dev <dev_name> --width xxx --height yyy // then
 * startup as rosnode
 */
namespace po = boost::program_options;

using Common::CamConfig;
using Common::IOType;
using std::cerr;
using std::cout;

struct CamInitializer {
  static std::shared_ptr<V4L2::CV4l2Cam> initRosMode(int argc, char** argv,
                                                     std::string node_name) {
    std::shared_ptr<RosHelper::RosHelper> ros_helper =
        std::make_shared<RosHelper::RosHelper>();
    CamConfig cam_config = ros_helper->parseConfig();
    return V4L2::produceV4l2Cam(cam_config);
  }
  static std::shared_ptr<V4L2::CV4l2Cam> initNativeMode(CamConfig cam_config) {
    return V4L2::produceV4l2Cam(cam_config);
  }
};

template <typename T>
T getCmdArg(po::variables_map vm, std::string name) {
  bool res = vm.count(name);
  if (res) {
    return vm[name].as<T>();
  } else {
    // throw exception and exit program
    cerr << " --" << name << " arg"
         << " is not given. process aborted." << '\n';
    std::abort();
    return {};
  }
}

std::shared_ptr<V4L2::CV4l2Cam> getInitCamForNativeMode(po::variables_map vm) {
  std::string dev_name = getCmdArg<std::string>(vm, "dev_name");
  std::string cam_name = getCmdArg<std::string>(vm, "cam_name");
  int height = getCmdArg<int>(vm, "height");
  int width = getCmdArg<int>(vm, "width");
  int io_method = getCmdArg<int>(vm, "io_method");

  if ((io_method < 0) || (io_method > 2)) {
    cerr << "wrong io_method type number " << io_method
         << " is given, required (0:mmap, 1:usrptr, 2:dma)"
         << "\n";
    std::abort();
    return nullptr;
  }

  CamConfig config = CamConfig{.cam_name = cam_name,
                               .dev_name = dev_name,
                               .width = width,
                               .height = height,
                               .io_type = static_cast<IOType>(io_method)};

  auto cam_ptr = CamInitializer::initNativeMode(config);
  return cam_ptr;
}

int main(int argc, char** argv) {
  po::options_description desc;
  desc.add_options()("help", "show help message")(
      "type", po::value<std::string>(),
      "config camera node start up type. arg is native or ros.")(
      "cam_name", po::value<std::string>(),
      "The name of start up camera  e.g (camera_0)")("dev_name",
                                                     po::value<std::string>(),
                                                     "device to be opened. "
                                                     "e.g "
                                                     "(/dev/video0)")(
      "height", po::value<int>(), "camera shown image height.")(
      "width", po::value<int>(), "camera config width.")(
      "io_method", po::value<int>(),
      "v4l2 method type (0: mmap, 1:userptr, 2: dma)");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (argc <= 1) {
    cout
        << "v4l2 cam driver use below config to startup, all args are required:"
        << "\n";
    cout << desc << "\n";
    return (EXIT_FAILURE);
  }

  std::string cmd_start_type = getCmdArg<std::string>(vm, "type");
  if ((cmd_start_type != "native") && (cmd_start_type != "ros")) {
    cerr << "wrong cam driver start type: " << vm["type"].as<std::string>()
         << "\n";
    cerr << "use: --type native or --type ros "
         << "\n";
    return (EXIT_FAILURE);
  }

  INFO("cam driver start as %s", cmd_start_type.c_str());

  std::shared_ptr<V4L2::CV4l2Cam> cam_ptr;
  if (cmd_start_type == "ros") {
    cam_ptr = CamInitializer::initRosMode(argc, argv, "image_publisher");
    ros::init(argc, argv, "dma_image_node");
  } else {
    cam_ptr = getInitCamForNativeMode(vm);
  }

  cam_ptr->GrabImg();
  return 0;
}