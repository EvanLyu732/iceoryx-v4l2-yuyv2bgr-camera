#include "v4l2cam.h"

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/types.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iceoryx_hoofs/cxx/optional.hpp>
#include <iceoryx_posh/capro/service_description.hpp>
#include <iceoryx_posh/iceoryx_posh_types.hpp>
#include <iceoryx_posh/popo/port_queue_policies.hpp>
#include <iceoryx_posh/popo/publisher_options.hpp>
#include <iostream>
#include <memory>

#include "shmImgBuf.h"
#include "shmUtility.h"
#include "cuda_runtime_api.h"
#include "fcntl.h"

#ifdef PUB_WITH_ICEORYX
#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#endif

#include "nvbufsurface.h"
#include "poll.h"
#include "roshelper.h"
#include "sensor_msgs/image_encodings.h"
#include "yuv2rgb.cuh"

#ifdef DEBUG
#include <opencv2/opencv.hpp>
#endif

using namespace cv;
using namespace cv::cuda;

namespace V4L2 {

using Common::convertMonotonicToRealtime;
using Common::NvBuffer;

bool CV4l2Cam::initByConfig(CamConfig config) noexcept {
  int fd = open(config.dev_name.c_str(), O_RDWR, 0);

  if (fd < 0) {
    int ret = close(fd);
    if (ret < 0) {
      ERROR_RETURN("Cannot close device:(%d): fd (%d)",
                   cam_config_.dev_name.c_str(), fd);
      exit(EXIT_FAILURE);
    }
  }

  INFO("open device: %s, fd: %d", cam_config_.dev_name.c_str(), fd);

  // check device capability
  struct v4l2_capability cam_cap;

  if (ioctl(fd, VIDIOC_QUERYCAP, &cam_cap) < 0) {
    ERROR_RETURN("error in query capability");
  }

  if (!(cam_cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    ERROR_RETURN("capabilities not support v4l2_cap_video_capture");
  }

  // setup device format
  struct v4l2_format fmt;
  std::memset(&fmt, 0, sizeof(fmt));

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = config.width;
  fmt.fmt.pix.height = config.height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

  if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
    ERROR_RETURN(
        "ioctl set video fmt failed, (fd: %d) (dev_name: %s), (err: %s, "
        "errno: %d)",
        fd, config.dev_name.c_str(), strerror(errno), errno);
  }

  context.cam_fd = fd;
  return true;
}

bool DMACV4l2Cam::initMemBuffers() noexcept {
  NvBufSurf::NvCommonAllocateParams camparams = {0};
  int yuyv_buffers_fd[V4L2_BUFFERS_NUM] = {0};
  int bgr_buffers_fd[V4L2_BUFFERS_NUM] = {0};

  /* Setup buffer for storing image */
  size_t mem_size = V4L2_BUFFERS_NUM * sizeof(NvBuffer);
  context.buffers = (NvBuffer *)malloc(mem_size);
  INFO("DMA Cam allocated persistent memory size %d", mem_size);
  if (!context.buffers) {
    ERROR_RETURN("Failed to allocate global buffer context");
  }

  /* Setup allocated nv params */
  camparams.memType = NVBUF_MEM_SURFACE_ARRAY;
  camparams.width = cam_config_.width;
  camparams.height = cam_config_.height;
  camparams.layout = NVBUF_LAYOUT_PITCH;
  camparams.colorFormat = NVBUF_COLOR_FORMAT_BGR;
  camparams.memtag = NvBufSurfaceTag_NONE;

  /*  Allocate YUYV NvBuffer */
  if (NvBufSurf::NvAllocate(&camparams, V4L2_BUFFERS_NUM, yuyv_buffers_fd)) {
    ERROR_RETURN("Failed to create YUYV NvBuffer");
  }

  /* Create YUYV buffer and provide it with camera */
  for (unsigned int index = 0; index < V4L2_BUFFERS_NUM; index++) {
    NvBufSurface *pSurf = NULL;

    /* Bind Stored DMA fd to Allocated Buffers fd */
    context.buffers[index].dmabuff_fd = yuyv_buffers_fd[index];

    if (-1 == NvBufSurfaceFromFd(yuyv_buffers_fd[index], (void **)(&pSurf))) {
      ERROR_RETURN("Failed to get NvBuffer parameters");
    }

    if (-1 == NvBufSurfaceMap(pSurf, 0, 0, NVBUF_MAP_READ_WRITE)) {
      ERROR_RETURN("Failed to map buffer");
    }
    context.buffers[index].start =
        (unsigned char *)pSurf->surfaceList[0].mappedAddr.addr[0];
    context.buffers[index].size = pSurf->surfaceList[0].dataSize;
    /* Size 6422528 */
  }

  INFO("DMA Cam Succeed in preparing stream buffers");

  /* Setup v4l2 request buffers */
  struct v4l2_requestbuffers rb;
  memset(&rb, 0, sizeof(rb));
  rb.count = V4L2_BUFFERS_NUM;
  rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rb.memory = V4L2_MEMORY_DMABUF;

  INFO("Setup requset buffers for cam_fd: %d", context.cam_fd);

  if (ioctl(context.cam_fd, VIDIOC_REQBUFS, &rb) < 0) {
    ERROR_RETURN(
        "Failed to request v4l2 buffers: (fd: %d) (err: %s) (errno: %d)",
        context.cam_fd, strerror(errno), errno);
  }

  if (rb.count != V4L2_BUFFERS_NUM) {
    ERROR_RETURN("V4l2 buffer number is not as desired");
  }

  /* initialize V4l2 Buffers (Binding with DMA fd) */
  context.v4l2_buffer_list =
      (v4l2_buffer *)malloc(V4L2_BUFFERS_NUM * sizeof(v4l2_buffer));

  for (unsigned int index = 0; index < V4L2_BUFFERS_NUM; index++) {
    struct v4l2_buffer buf;

    /* Query camera v4l2 buf length */
    memset(&buf, 0, sizeof buf);
    buf.index = index;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_DMABUF;
    buf.flags |= V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

    if (ioctl(context.cam_fd, VIDIOC_QUERYBUF, &buf) < 0) {
      ERROR_RETURN("Failed to query buff: (fd: %d) (err: %s) (errno: %d)",
                   context.cam_fd, strerror(errno), errno);
    }

    buf.m.fd = (unsigned long)context.buffers[index].dmabuff_fd;

    if (ioctl(context.cam_fd, VIDIOC_QBUF, &buf) < 0) {
      ERROR_RETURN("Failed to enqueue buffers: %s (%d)", strerror(errno),
                   errno);
    }
    context.v4l2_buffer_list[index] = buf;
  }

  return true;
};

bool DMACV4l2Cam::GrabImg() noexcept {
  enum v4l2_buf_type type;
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  /* Start v4l2 streaming */
  if (ioctl(context.cam_fd, VIDIOC_STREAMON, &type) < 0)
    ERROR_RETURN("Failed to start streaming on fd:(%d): err: %s (%d)",
                 context.cam_fd, strerror(errno), errno);

  usleep(200);
  INFO("Camera video start streaming ...");

  /* Get Camera Config */
  int height = cam_config_.height;
  int width = cam_config_.width;

  static const size_t yuyv_img_size = height * width * 2;
  static const size_t bgr_img_size = height * width * 3;

  std::string cam_name = cam_config_.cam_name;

  /* Wait for a frame each time time */
  struct pollfd fds[1];

  fds[0].fd = context.cam_fd;
  fds[0].events = POLLIN;

  /* Sequence Frame Number */
  static int seq_num{0};

  /* Zero-Copy GPU Memory  */
  unsigned char *cpu_yuyv_buf;
  {
    auto cudaError = cudaHostAlloc((void **)&cpu_yuyv_buf, yuyv_img_size,
                                   cudaHostAllocMapped);
    if (cudaError != cudaSuccess) {
      WARN("Allocate YUYV Img BUffer on CUDA Failed");
      return false;
    }
  }

  unsigned char *cpu_bgr_buf;
  {
    auto cudaError =
        cudaHostAlloc((void **)&cpu_bgr_buf, bgr_img_size, cudaHostAllocMapped);
    if (cudaError != cudaSuccess) {
      WARN("Allocate BGR Img BUffer on CUDA Failed");
      return false;
    }
  }

  unsigned char *gpu_yuyv_buf;
  unsigned char *gpu_bgr_buf;

  cudaHostGetDevicePointer((void **)&gpu_yuyv_buf, (void *)cpu_yuyv_buf, 0);
  cudaHostGetDevicePointer((void **)&gpu_bgr_buf, (void *)cpu_bgr_buf, 0);

  /* Wait for camera event with timeout = 1000 ms */
  INFO("polling event begin....");

  /* Initial Iceoryx */
#ifdef PUB_WITH_ICEORYX
  /* App Name Tag: "camera-publisher-" + @CAM_NAME@ */
  char APP_NAME[50] = "camera-publisher-";
  std::strcat(APP_NAME, cam_config_.cam_name.c_str());

  INFO("Create iceoryx app: %s", APP_NAME);
  iox::runtime::PoshRuntime::initRuntime(APP_NAME);

  /* Channel Name Tag: "channel" + @CAM_NAME@ + "channel" */
  INFO("Create iceoryx ipc channel %s", cam_config_.cam_name.c_str());
  iox::cxx::string<50> channel_name;
  channel_name.unsafe_assign(cam_config_.cam_name.c_str());
  // std::memcpy((void *)channel_name.c_str(), cam_config_.cam_name.c_str(),
  //             sizeof(cam_config_.cam_name.c_str()));

  iox::popo::PublisherOptions publishoptions;
  publishoptions.historyCapacity = 0;
  publishoptions.nodeName = "cam_driver";
  publishoptions.subscriberTooSlowPolicy =
      iox::popo::ConsumerTooSlowPolicy::DISCARD_OLDEST_DATA;

  iox::popo::Publisher<BgrImgBuf> img_publisher(
      {"camera", channel_name, "channel"}, publishoptions);
#endif

#ifdef JETPACK_SUPPORT
  /* Prepare Hardware Buffer */
  NvBufSurface *pSurf = NULL;
#endif

  while (poll(fds, 1, 1000) > 0) {
    if (fds[0].revents & POLLIN) {
      /* Only Tracking for 30 Frames */
      if (seq_num >= V4L2_BUFFERS_NUM) {
        std::memset(&context.v4l2_buffer_list[seq_num].memory, 0,
                    sizeof(&context.v4l2_buffer_list[seq_num].memory));
        seq_num = 0;
      }

      /* Dequeue a camera buffiox::capro::string */
      auto v4l2_buf = context.v4l2_buffer_list[seq_num];
      if (ioctl(context.cam_fd, VIDIOC_DQBUF, &v4l2_buf) < 0) {
        ERROR_RETURN("Failed to dequeue camera buff: %s (%d)", strerror(errno),
                     errno);
      }

#ifdef JETPACK_SUPPORT

      if (-1 == NvBufSurfaceFromFd(context.buffers[seq_num].dmabuff_fd,
                                   (void **)(&pSurf))) {
        ERROR_RETURN("Cannot get NvBufSurface from fd");
      }

      if (NvBufSurfaceMap(pSurf, 0, 0, NVBUF_MAP_READ_WRITE) != 0) {
        ERROR_RETURN("Err in Map");
      }

      // TODO: Must Unmap NvBufSurface, If not, Mem Leak.
#endif

      /* Convert Caputre TimeStamp To  Wall Time */
      convertMonotonicToRealtime(v4l2_buf.timestamp);

      /* Copy Captured Image Into Gpu Buffer */
      std::memcpy(gpu_yuyv_buf, context.buffers[seq_num].start, yuyv_img_size);

      /* Using GPU To Convert YUYV Image into BGR Format */
      gpuConvertYUYVtoBGR(gpu_yuyv_buf, gpu_bgr_buf, width, height);

#ifdef JETPACK_SUPPORT
      /* Copy Captured Image Into DMA Buffer */
      std::memcpy(pSurf->surfaceList[0].mappedAddr.addr[0], gpu_bgr_buf,
                  width * height * 3);
      /* Store Converted TimeStamp Into Reserved Field */
      pSurf->surfaceList[0]._reserved[0] = &v4l2_buf.timestamp.tv_sec;
      pSurf->surfaceList[0]._reserved[1] = &v4l2_buf.timestamp.tv_usec;

      /* Cache sync for VIC operation since the data is from CPU */
      if (-1 == NvBufSurfaceSyncForDevice(pSurf, 0, 0)) {
        ERROR_RETURN("Cannot sync output buffer");
      }
#endif

#ifdef PUB_WITH_ICEORYX
      /* Use Iceoryx For Sending Image Buffer */
      auto send_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now().time_since_epoch())
                           .count();

      img_publisher.loan()
          .and_then([&](auto &sample) {
            std::memcpy((void *)sample->buf.c_str(), gpu_bgr_buf, bgr_img_size);
            sample->send_ts = send_time;
            sample.publish();
          })
          .or_else([](auto &error) {
            // Do something with error
            std::cerr << "Unable to loan sample, error: " << error << std::endl;
          });
#endif PUB_WITH_ICEORYX

#ifdef DEBUG
      // cv::Mat bgr_img(height, width, CV_8UC3, (void *)str_buf.c_str());
      cv::Mat bgr_img(height, width, CV_8UC3, gpu_bgr_buf);
      cv::imshow("output_img", bgr_img);
      cv::waitKey(1);
#endif

      /* Enqueue camera buffer back to driver */
      if (ioctl(context.cam_fd, VIDIOC_QBUF, &v4l2_buf)) {
        ERROR_RETURN("Failed to queue camera buffers: %s (%d)", strerror(errno),
                     errno);
      }

      seq_num++;
    }
  }

  cudaFree(cpu_bgr_buf);
  cudaFree(cpu_yuyv_buf);

#ifdef PUB_WITH_ICEORYX
  iox::runtime::PoshRuntime::getInstance().shutdown();
#endif
  return true;
};

void DMACV4l2Cam::clearUpResources() noexcept {
  /* clean up allocated buffer*/
  free(context.buffers);

  /* clean up v4l2_buffer list */
  free(context.v4l2_buffer_list);

  // boost::interprocess::shared_memory_object::remove(shm_name.c_str());

  // Unmap NvBuffer

  /* close device fd */
  INFO("on deconstruct cloose fd: %d", context.cam_fd);
  if (context.cam_fd != 0) {
    close(context.cam_fd);
  }
  return;
}

}  // namespace V4L2
