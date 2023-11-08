#pragma once

/* This file is reserved for using boost interprocess ipc */

#include <boost/interprocess/containers/string.hpp>  // boost/containers/string.hpp
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/containers/vector.hpp>  // boost/containers/vector.hpp
#include <boost/interprocess/managed_shared_memory.hpp>
#include <cstdint>
#include <iostream>

namespace bip = boost::interprocess;

namespace Shm {
template <typename Alloc = std::allocator<char>>
struct ImgBuffer {
  using string =
      bip::basic_string<char, std::char_traits<char>,
                        typename Alloc::template rebind<char>::other>;

  ImgBuffer(Alloc alloc = {}) : buf(alloc) {}

  template <typename T>
  ImgBuffer(uint64_t capture_ts, T&& buf, Alloc alloc = {})
      : capture_ts(capture_ts), buf(std::forward<T>(buf), alloc) {}

  uint64_t capture_ts;
  string buf;
};
}  // namespace Shm

using InData = Shm::ImgBuffer<>;  // just heap allocated

namespace Shared {
using segment = bip::managed_shared_memory;  // or managed_shared_memory
using segment_manager = segment::segment_manager;

template <typename T>
using alloc = bip::allocator<T, segment_manager>;
template <typename T>
using vector = bip::vector<T, alloc<T>>;

using InData = Shm::ImgBuffer<alloc<char>>;  // shared memory version

vector<InData>& locate(segment& smt) {
  auto* v = smt.find_or_construct<vector<InData>>("ImgBuf")(
      smt.get_segment_manager());
  assert(v);
  return *v;
}
}  // namespace Shared
