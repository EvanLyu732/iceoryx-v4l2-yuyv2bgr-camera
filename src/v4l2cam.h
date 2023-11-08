#pragma once

#include <linux/videodev2.h>
#include <sys/types.h>

#include <iostream>
#include <memory>

#include "common.h"

namespace V4L2 {

using Common::CamConfig;
using Common::Context_t;
using Common::ImgBuf;
using Common::IOType;

struct CV4l2Cam {
  virtual ~CV4l2Cam() = default;
  CV4l2Cam(CamConfig config) {
    INFO("update base class cam config");
    updateConfig(config);
    initByConfig(config);
  }

  virtual bool GrabImg() = 0;

  void clearUpConext() noexcept {
    if (context.cam_fd != 0) {
      int ret = close(context.cam_fd);
    }
  }

  inline void updateConfig(CamConfig config) noexcept {
    cam_config_ = config;
    return;
  }

  bool initByConfig(CamConfig config) noexcept;

 public:
  CamConfig cam_config_;
  Context_t context;
};

struct MMapCV4l2Cam : public CV4l2Cam {
  using CV4l2Cam::CV4l2Cam;
};

struct UsrPtrCV4l2Cam : public CV4l2Cam {
  using CV4l2Cam::CV4l2Cam;
};

struct DMACV4l2Cam : public CV4l2Cam {
  using CV4l2Cam::CV4l2Cam;

  ~DMACV4l2Cam() { clearUpResources(); }

  bool initMemBuffers() noexcept;
  void clearUpResources() noexcept;

  bool GrabImg() noexcept override;
};

static std::shared_ptr<CV4l2Cam> produceV4l2Cam(CamConfig cam_config) {
  switch (cam_config.io_type) {
      // case IOType::IO_METHOD_MMAP:
      //   std::cout << "construct IO_METHOD_MMAP v4l2 cam" << '\n';
      //   return std::make_shared<MMapCV4l2Cam>(cam_config);

    case IOType::IO_METHOD_DMA:
      INFO("construct IO_METHOD_DMA v4l2 cam");
      auto cam_ptr = std::make_shared<DMACV4l2Cam>(cam_config);
      cam_ptr->initMemBuffers();
      return cam_ptr;

      // case IOType::IO_METHOD_USERPTR:
      //   std::cout << "construct IO_METHOD_USERPTR v4l2 cam" << '\n';
      //   return std::make_shared<UsrPtrCV4l2Cam>(cam_config);
  }
}

}  // namespace V4L2