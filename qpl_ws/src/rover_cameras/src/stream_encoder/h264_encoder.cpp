#include "h264_encoder.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/dict.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
}

namespace rover_cameras
{

namespace
{
std::string avError(int err)
{
  char buf[AV_ERROR_MAX_STRING_SIZE] = {0};
  av_strerror(err, buf, sizeof(buf));
  return buf;
}

std::string trim(const std::string & s)
{
  const auto b = s.find_first_not_of(" \t");
  const auto e = s.find_last_not_of(" \t");
  return b == std::string::npos ? "" : s.substr(b, e - b + 1);
}
}  // namespace

bool encoderExists(const std::string & name)
{
  return avcodec_find_encoder_by_name(name.c_str()) != nullptr;
}

std::vector<std::pair<std::string, std::string>> parseAvOptions(const std::string & text)
{
  std::vector<std::pair<std::string, std::string>> out;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item = trim(item);
    if (item.empty()) {
      continue;
    }
    const auto eq = item.find('=');
    if (eq == std::string::npos || eq == 0) {
      throw std::invalid_argument("AV option '" + item + "' is not key=value");
    }
    out.emplace_back(trim(item.substr(0, eq)), trim(item.substr(eq + 1)));
  }
  return out;
}

H264Encoder::H264Encoder() = default;

H264Encoder::~H264Encoder() {close();}

std::vector<std::string> H264Encoder::open(const EncoderConfig & config)
{
  close();
  if (config.width <= 0 || config.height <= 0 || config.width % 2 || config.height % 2) {
    throw std::runtime_error("encoder size must be positive and even");
  }
  const AVCodec * codec = avcodec_find_encoder_by_name(config.codec.c_str());
  if (!codec) {
    throw std::runtime_error("unknown encoder: " + config.codec);
  }
  ctx_ = avcodec_alloc_context3(codec);
  if (!ctx_) {
    throw std::runtime_error("cannot allocate codec context");
  }
  config_ = config;
  ctx_->width = config.width;
  ctx_->height = config.height;
  ctx_->pix_fmt = AV_PIX_FMT_YUV420P;
  ctx_->time_base = AVRational{1, 1000};
  ctx_->framerate = AVRational{static_cast<int>(config.fps_hint + 0.5), 1};
  // Keyframes are forced by the caller on a timer and when viewers join, so the GOP only
  // needs to be longer than that interval.
  ctx_->gop_size = 600;
  ctx_->max_b_frames = 0;
  ctx_->thread_count = config.threads;
  // Slice threads add no frames of delay; frame threads add one per thread.
  ctx_->thread_type = FF_THREAD_SLICE;
  last_pts_ = -1;
  measured_ = false;
  interval_ms_ = 1000.0 / config.fps_hint;
  rate_scale_ = 1.0;
  applyRate();

  AVDictionary * opts = nullptr;
  if (!config.preset.empty()) {av_dict_set(&opts, "preset", config.preset.c_str(), 0);}
  if (!config.tune.empty()) {av_dict_set(&opts, "tune", config.tune.c_str(), 0);}
  // Makes a forced I frame an IDR, which is what a joining decoder (e.g. h264_cuvid) needs.
  av_dict_set(&opts, "forced-idr", "1", 0);
  for (const auto & kv : config.av_options) {
    av_dict_set(&opts, kv.first.c_str(), kv.second.c_str(), 0);
  }
  const int err = avcodec_open2(ctx_, codec, &opts);
  std::vector<std::string> unused;
  const AVDictionaryEntry * e = nullptr;
  while ((e = av_dict_get(opts, "", e, AV_DICT_IGNORE_SUFFIX))) {
    unused.emplace_back(std::string(e->key) + "=" + e->value);
  }
  av_dict_free(&opts);
  if (err < 0) {
    close();
    throw std::runtime_error("cannot open " + config.codec + ": " + avError(err));
  }

  frame_ = av_frame_alloc();
  packet_ = av_packet_alloc();
  if (!frame_ || !packet_) {
    close();
    throw std::runtime_error("cannot allocate frame/packet");
  }
  frame_->format = AV_PIX_FMT_YUV420P;
  frame_->width = config.width;
  frame_->height = config.height;
  return unused;
}

void H264Encoder::close()
{
  if (packet_) {av_packet_free(&packet_);}
  if (frame_) {av_frame_free(&frame_);}
  if (ctx_) {avcodec_free_context(&ctx_);}
}

std::string H264Encoder::codecFamily() const
{
  return ctx_ ? avcodec_get_name(ctx_->codec_id) : "";
}

void H264Encoder::setBitRate(int64_t bit_rate, int vbv_buffer_ms)
{
  config_.bit_rate = bit_rate;
  config_.vbv_buffer_ms = vbv_buffer_ms;
  if (ctx_) {
    applyRate();
  }
}

void H264Encoder::applyRate()
{
  // x264 budgets bit_rate / fps_hint per frame, so scale by fps_hint / real fps to hold the
  // real bit rate. The VBV size is already in bits, so it needs no scaling.
  const auto rate = static_cast<int64_t>(config_.bit_rate * rate_scale_);
  ctx_->bit_rate = rate;
  if (config_.vbv_buffer_ms > 0) {
    ctx_->rc_max_rate = rate;
    ctx_->rc_buffer_size = static_cast<int>(config_.bit_rate * config_.vbv_buffer_ms / 1000);
  } else {
    ctx_->rc_max_rate = 0;
    ctx_->rc_buffer_size = 0;
  }
}

void H264Encoder::encode(
  const uint8_t * i420, int64_t pts_ms, bool force_keyframe, const PacketCallback & cb)
{
  if (!ctx_) {
    throw std::runtime_error("encoder not open");
  }
  const int w = config_.width;
  const int h = config_.height;
  // Not ref-counted, so avcodec_send_frame takes its own copy and i420 can be reused at once.
  frame_->data[0] = const_cast<uint8_t *>(i420);
  frame_->data[1] = frame_->data[0] + w * h;
  frame_->data[2] = frame_->data[1] + (w / 2) * (h / 2);
  frame_->linesize[0] = w;
  frame_->linesize[1] = w / 2;
  frame_->linesize[2] = w / 2;
  trackFrameRate(pts_ms);
  frame_->pts = pts_ms;
  frame_->pict_type = force_keyframe ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_NONE;

  int err = avcodec_send_frame(ctx_, frame_);
  if (err == AVERROR(EAGAIN)) {
    drain(cb);
    err = avcodec_send_frame(ctx_, frame_);
  }
  if (err < 0) {
    throw std::runtime_error("send_frame failed: " + avError(err));
  }
  drain(cb);
}

void H264Encoder::trackFrameRate(int64_t pts_ms)
{
  const int64_t dt = last_pts_ < 0 ? 0 : pts_ms - last_pts_;
  const bool first_interval = last_pts_ >= 0 && !measured_;
  last_pts_ = pts_ms;
  if (dt <= 0 || dt > 1000) {
    return;  // first frame, or a pause (e.g. nobody watching): not a frame interval
  }
  // Seed from the first real interval rather than converging from fps_hint.
  interval_ms_ = first_interval ? static_cast<double>(dt) :
    interval_ms_ + 0.1 * (static_cast<double>(dt) - interval_ms_);
  measured_ = true;
  const double scale = config_.fps_hint * interval_ms_ / 1000.0;
  if (std::abs(scale / rate_scale_ - 1.0) > 0.1) {
    rate_scale_ = scale;
    applyRate();
  }
}

double H264Encoder::measuredFps() const {return 1000.0 / interval_ms_;}

void H264Encoder::drain(const PacketCallback & cb)
{
  while (true) {
    const int err = avcodec_receive_packet(ctx_, packet_);
    if (err == AVERROR(EAGAIN) || err == AVERROR_EOF) {
      return;
    }
    if (err < 0) {
      throw std::runtime_error("receive_packet failed: " + avError(err));
    }
    cb(packet_->data, static_cast<size_t>(packet_->size), packet_->pts,
      (packet_->flags & AV_PKT_FLAG_KEY) != 0);
    av_packet_unref(packet_);
  }
}

}  // namespace rover_cameras
