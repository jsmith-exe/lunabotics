#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

struct AVCodecContext;
struct AVFrame;
struct AVPacket;

namespace rover_cameras
{

struct EncoderConfig
{
  std::string codec = "libx264";
  int width = 0;
  int height = 0;
  int64_t bit_rate = 1300000;
  // VBV buffer, in milliseconds of bit_rate. Caps how far a single frame (mostly keyframes)
  // can overshoot, which bounds the latency spike it causes on a rate-limited link. 0 = no VBV.
  int vbv_buffer_ms = 200;
  int threads = 4;
  std::string preset = "superfast";
  std::string tune = "zerolatency";
  // Expected frame rate. The encoder measures the real rate from the pts and rescales the
  // rate control to match, so this only matters for the first second or so.
  double fps_hint = 30.0;
  // Extra options handed to avcodec_open2 (codec-context and private options alike).
  std::vector<std::pair<std::string, std::string>> av_options;
};

// Parses "key=value,key=value". Values may contain '=' and ':' (e.g. x264-params), not ','.
std::vector<std::pair<std::string, std::string>> parseAvOptions(const std::string & text);

// Thin libavcodec wrapper for low-latency streaming of I420 frames. pts are in milliseconds.
//
// libx264 rate control budgets bit_rate / fps per frame, and ffmpeg's wrapper gives it a
// constant frame rate (x264's variable-frame-rate mode would fix that, but delays every
// packet by a frame). So the real frame rate is measured from the pts and the rate handed
// to x264 is rescaled to match: bit_rate holds at whatever rate frames actually arrive.
// Not thread-safe: use from one thread.
class H264Encoder
{
public:
  using PacketCallback =
    std::function<void(const uint8_t * data, size_t size, int64_t pts, bool keyframe)>;

  H264Encoder();
  ~H264Encoder();
  H264Encoder(const H264Encoder &) = delete;
  H264Encoder & operator=(const H264Encoder &) = delete;

  // Throws std::runtime_error on failure. Returns options the codec did not recognise.
  std::vector<std::string> open(const EncoderConfig & config);
  void close();
  bool isOpen() const {return ctx_ != nullptr;}
  const EncoderConfig & config() const {return config_;}
  // Codec family as ffmpeg_image_transport names it, e.g. "h264".
  std::string codecFamily() const;

  // Takes effect from the next frame without reopening (libx264 reconfigures in place).
  void setBitRate(int64_t bit_rate, int vbv_buffer_ms);
  // Frame rate measured from the pts (smoothed).
  double measuredFps() const;

  // i420 must be a contiguous width*height*3/2 buffer. pts must strictly increase.
  // force_keyframe requests an IDR. The callback runs once per packet produced.
  void encode(const uint8_t * i420, int64_t pts_ms, bool force_keyframe, const PacketCallback & cb);

private:
  void applyRate();
  void trackFrameRate(int64_t pts_ms);
  void drain(const PacketCallback & cb);

  EncoderConfig config_;
  AVCodecContext * ctx_ = nullptr;
  AVFrame * frame_ = nullptr;
  AVPacket * packet_ = nullptr;
  int64_t last_pts_ = -1;
  bool measured_ = false;
  double interval_ms_ = 33.3;
  double rate_scale_ = 1.0;
};

}  // namespace rover_cameras
