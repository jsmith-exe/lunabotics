#include "rover_cameras/convert.hpp"

#include <algorithm>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace rover_cameras
{

std::string decodedEncoding(const std::string & input_encoding)
{
  if (input_encoding == "rgb8" || input_encoding == "rgba8") {return "rgb8";}
  if (input_encoding == "bgr8" || input_encoding == "bgra8") {return "bgr8";}
  if (input_encoding == "mono8") {return "mono8";}
  return "";
}

std::pair<int, int> outputSize(int in_width, int in_height, int req_width, int req_height)
{
  int w = req_width;
  int h = req_height;
  if (w <= 0 && h <= 0) {
    w = in_width;
    h = in_height;
  } else if (w <= 0) {
    w = static_cast<int>(static_cast<int64_t>(in_width) * h / in_height);
  } else if (h <= 0) {
    h = static_cast<int>(static_cast<int64_t>(in_height) * w / in_width);
  }
  w = std::max(2, w & ~1);
  h = std::max(2, h & ~1);
  return {w, h};
}

void toI420(
  const uint8_t * data, int width, int height, size_t step, const std::string & encoding,
  int out_width, int out_height, cv::Mat & scratch, cv::Mat & i420)
{
  int type;
  int code = -1;
  if (encoding == "rgb8") {
    type = CV_8UC3; code = cv::COLOR_RGB2YUV_I420;
  } else if (encoding == "bgr8") {
    type = CV_8UC3; code = cv::COLOR_BGR2YUV_I420;
  } else if (encoding == "rgba8") {
    type = CV_8UC4; code = cv::COLOR_RGBA2YUV_I420;
  } else if (encoding == "bgra8") {
    type = CV_8UC4; code = cv::COLOR_BGRA2YUV_I420;
  } else if (encoding == "mono8") {
    type = CV_8UC1;
  } else {
    throw std::invalid_argument("unsupported image encoding: " + encoding);
  }

  const cv::Mat src(height, width, type, const_cast<uint8_t *>(data), step);
  const cv::Mat * scaled = &src;
  if (out_width != width || out_height != height) {
    // INTER_AREA gives the best downscale but is only fast for whole-number ratios;
    // for anything else (e.g. 1080p -> 720p) it costs ~10x more than INTER_LINEAR.
    const bool integer_downscale = out_width < width && width % out_width == 0 &&
      height % out_height == 0 && width / out_width == height / out_height;
    cv::resize(src, scratch, cv::Size(out_width, out_height), 0, 0,
      integer_downscale ? cv::INTER_AREA : cv::INTER_LINEAR);
    scaled = &scratch;
  }

  if (code >= 0) {
    cv::cvtColor(*scaled, i420, code);
  } else {
    // Grey: luma is the image itself, chroma is neutral.
    i420.create(out_height * 3 / 2, out_width, CV_8UC1);
    scaled->copyTo(i420.rowRange(0, out_height));
    i420.rowRange(out_height, i420.rows).setTo(128);
  }
}

}  // namespace rover_cameras
