#pragma once

#include <cstddef>
#include <cstdint>

#include "iceoryx_hoofs/cxx/string.hpp"

// For Convince
constexpr static size_t reserved_bgr_size = 1920 * 1080 * 3;

template <const size_t ImgSize>
struct ShmImgBuf {
  /* TimeStamp For Sending Buffer*/
  uint64_t send_ts;
  /* TImeStamp In microseconds*/
  uint64_t captured_ts_us;
  /* imgBuffer in here */
  iox::cxx::string<ImgSize> buf;
};

using BgrImgBuf = ShmImgBuf<reserved_bgr_size>;