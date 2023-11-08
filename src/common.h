#pragma once

#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdint>
#include <string>

#include "NvBufSurface.h"

#define V4L2_BUFFERS_NUM 30

#define __FILENAME__ \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define INFO(fmt, ...)                                               \
  printf("INFO: (file: [%s] ): (func: [%s] ): (line: %d) " fmt "\n", \
         __FILENAME__, __FUNCTION__, __LINE__, ##__VA_ARGS__);

#define WARN(fmt, ...)                                               \
  printf("WARN: (file: [%s] ): (func: [%s] ): (line: %d) " fmt "\n", \
         __FILENAME__, __FUNCTION__, __LINE__, ##__VA_ARGS__);

#define CHECK_ERROR(cond, label, fmt, ...)                              \
  if (!cond) {                                                          \
    error = 1;                                                          \
    printf("ERROR: (file: [%s] ): (func: [%s] ): (line: %d) " fmt "\n", \
           __FILENAME__, __FUNCTION__, __LINE__, ##__VA_ARGS__);        \
    goto label;                                                         \
  }

#define ERROR_RETURN(fmt, ...)                                          \
  do {                                                                  \
    printf("ERROR: (file: [%s] ): (func: [%s] ): (line: %d) " fmt "\n", \
           __FILENAME__, __FUNCTION__, __LINE__, ##__VA_ARGS__);        \
    return false;                                                       \
  } while (0)

namespace Common {

typedef u_char* ImgBuf;

enum class IOType : uint8_t {
  IO_METHOD_MMAP = 0,
  IO_METHOD_USERPTR = 1,
  IO_METHOD_DMA = 2,
};

struct CamConfig {
  std::string cam_name;
  std::string dev_name;
  int width;
  int height;
  IOType io_type;
};

struct NvBuffer {
  // pointer to the start image
  ImgBuf start;
  // image size
  ssize_t size;
  // dma fd
  int dmabuff_fd;
};

struct Context_t {
  /* NvBuffers storing captured image (YUYV) */
  NvBuffer* buffers;

  /* v4l2_buffer list for captured image */
  v4l2_buffer* v4l2_buffer_list;

  /* Camera Fd for storing opened device */
  int cam_fd;
};

/* Correlate v4l2 pixel format and NvBuffer color format */
struct NvColorFmt {
  unsigned int v4l2_pixfmt;
  NvBufSurfaceColorFormat nvbuff_color;
};

static NvColorFmt nvcolor_fmt[] = {
    /* TODO: add more pixel format mapping */
    {V4L2_PIX_FMT_UYVY, NVBUF_COLOR_FORMAT_UYVY},
    {V4L2_PIX_FMT_VYUY, NVBUF_COLOR_FORMAT_VYUY},
    {V4L2_PIX_FMT_YUYV, NVBUF_COLOR_FORMAT_YUYV},
    {V4L2_PIX_FMT_YVYU, NVBUF_COLOR_FORMAT_YVYU},
    {V4L2_PIX_FMT_GREY, NVBUF_COLOR_FORMAT_GRAY8},
    {V4L2_PIX_FMT_YUV420M, NVBUF_COLOR_FORMAT_YUV420},
};

static NvBufSurfaceColorFormat get_nvbuff_color_fmt(unsigned int v4l2_pixfmt) {
  unsigned i;

  for (i = 0; i < sizeof(nvcolor_fmt) / sizeof(nvcolor_fmt[0]); i++) {
    if (v4l2_pixfmt == nvcolor_fmt[i].v4l2_pixfmt)
      return nvcolor_fmt[i].nvbuff_color;
  }

  return NVBUF_COLOR_FORMAT_INVALID;
}

// Function to convert a timeval from CLOCK_MONOTONIC to CLOCK_REALTIME
static void convertMonotonicToRealtime(struct timeval& monotonicTime) {
  struct timespec monotonicTs, realtimeTs;

  // Get the current CLOCK_REALTIME timestamp
  clock_gettime(CLOCK_REALTIME, &realtimeTs);

  // Get the CLOCK_MONOTONIC timestamp corresponding to the same time
  clock_gettime(CLOCK_MONOTONIC, &monotonicTs);

  // Calculate the difference between CLOCK_REALTIME and CLOCK_MONOTONIC
  // timestamps
  long long monotonicToRealtimeOffsetUs =
      (realtimeTs.tv_sec - monotonicTs.tv_sec) * 1000000LL +
      (realtimeTs.tv_nsec - monotonicTs.tv_nsec) / 1000LL;

  // Apply the offset to the input timeval
  monotonicTime.tv_sec += monotonicToRealtimeOffsetUs / 1000000LL;
  monotonicTime.tv_usec += monotonicToRealtimeOffsetUs % 1000000LL;

  // Normalize timeval if needed
  if (monotonicTime.tv_usec >= 1000000) {
    monotonicTime.tv_sec += 1;
    monotonicTime.tv_usec -= 1000000;
  }
}

}  // namespace Common