#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

#include <opencv2/core.hpp>

namespace rover_cameras
{

// Encoding the viewer should decode to for a given input encoding ("" if unsupported).
// Alpha is dropped, so rgba8 -> rgb8 and bgra8 -> bgr8.
std::string decodedEncoding(const std::string & input_encoding);

// Output size for a requested width/height. 0 means "keep": both 0 keeps the input size,
// one 0 keeps the aspect ratio. Always rounded down to even (I420 and x264 need it).
std::pair<int, int> outputSize(int in_width, int in_height, int req_width, int req_height);

// Resizes (if needed) then converts to a contiguous I420 image of out_height*3/2 rows.
// scratch holds the resized image between calls. Throws std::invalid_argument on an
// unsupported encoding.
void toI420(
  const uint8_t * data, int width, int height, size_t step, const std::string & encoding,
  int out_width, int out_height, cv::Mat & scratch, cv::Mat & i420);

}  // namespace rover_cameras
