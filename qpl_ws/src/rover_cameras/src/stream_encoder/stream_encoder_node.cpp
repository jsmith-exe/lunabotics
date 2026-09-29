// Encodes a camera's raw colour stream to H.264 for viewing over the rate-limited link.
//
// Replaces ffmpeg_image_transport on the driver side. Loaded into the camera's own
// container with intra-process comms, so raw frames are handed over without copying, and
// the encode runs on a worker thread that always takes the newest frame: a slow encode
// drops frames here instead of stalling the driver. Output is ffmpeg_image_transport's
// FFMPEGPacket, so viewers (rviz) decode it with the stock ffmpeg subscriber plugin.

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "convert.hpp"
#include "h264_encoder.hpp"

extern "C" {
#include <libavutil/log.h>
}

namespace rover_cameras
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using ffmpeg_image_transport_msgs::msg::FFMPEGPacket;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using SteadyClock = std::chrono::steady_clock;

class StreamEncoder : public rclcpp::Node
{
public:
  explicit StreamEncoder(const rclcpp::NodeOptions & options)
  : Node("stream_encoder", options)
  {
    // libx264 logs its capabilities and per-session stats at info level on every (re)open.
    av_log_set_level(AV_LOG_WARNING);
    auto read_only = [](const std::string & d) {
        rcl_interfaces::msg::ParameterDescriptor p;
        p.description = d;
        p.read_only = true;
        return p;
      };
    auto desc = [](const std::string & d) {
        rcl_interfaces::msg::ParameterDescriptor p;
        p.description = d;
        return p;
      };
    input_topic_ = declare_parameter("input_topic", "image_raw", read_only("Raw image topic"));
    info_topic_ = declare_parameter(
      "camera_info_topic", "camera_info", read_only("Input camera_info; empty disables"));
    output_topic_ = declare_parameter(
      "output_topic", "teleop_stream",
      read_only("Output base topic: publishes <base>/ffmpeg and <base>/camera_info"));
    const auto reliability = declare_parameter(
      "output_reliability", "best_effort", read_only("best_effort or reliable"));
    input_reliable_ = declare_parameter(
      "input_reliability", "best_effort",
      read_only("Camera subscription: best_effort (driver in the same process) or reliable "
      "(frames over DDS from another process, e.g. Gazebo)")) == "reliable";

    declare_parameter("width", 0, desc("Output width; 0 keeps input (or aspect, with height)"));
    declare_parameter("height", 0, desc("Output height; 0 keeps input (or aspect, with width)"));
    declare_parameter("max_fps", 0.0, desc("Output frame-rate cap; 0 = input rate"));
    declare_parameter("codec", "libx264", desc("libavcodec encoder name"));
    declare_parameter("preset", "superfast", desc("Encoder preset"));
    declare_parameter("tune", "zerolatency", desc("Encoder tune"));
    declare_parameter("threads", 4, desc("Encoder (slice) threads"));
    declare_parameter("bit_rate", 1300000, desc("Target bit rate, bits/s. Live-tunable"));
    declare_parameter("vbv_buffer_ms", 200, desc("VBV buffer in ms of bit_rate; 0 disables"));
    declare_parameter("keyframe_interval", 1.0, desc("Seconds between forced IDR frames"));
    declare_parameter("av_options", "", desc("Extra AV options: key=value,key=value"));
    stats_period_ = declare_parameter("stats_period", 5.0, read_only("Stats interval, s"));

    settings_ = readSettings(get_parameters(kLiveParams), Settings{});
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & p) {return onSetParameters(p);});

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    if (reliability == "reliable") {
      qos.reliable();
    } else {
      qos.best_effort();
    }
    packet_pub_ = create_publisher<FFMPEGPacket>(output_topic_ + "/ffmpeg", qos);
    info_pub_ = create_publisher<CameraInfo>(output_topic_ + "/camera_info", qos);
    diag_pub_ = create_publisher<DiagnosticArray>("/diagnostics", 10);

    worker_ = std::thread([this] {workerLoop();});
    sub_timer_ = create_wall_timer(std::chrono::milliseconds(250), [this] {updateSubscriptions();});
    stats_timer_ = create_wall_timer(
      std::chrono::duration<double>(stats_period_), [this] {publishStats();});
    last_stats_ = SteadyClock::now();

    input_name_ = resolve(input_topic_);
    output_name_ = packet_pub_->get_topic_name();
    RCLCPP_INFO(
      get_logger(), "Teleop stream encoder ready: will encode %s to H.264 on %s (%s) while "
      "anyone is viewing it", input_name_.c_str(), output_name_.c_str(), reliability.c_str());
  }

  ~StreamEncoder() override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      running_ = false;
    }
    cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  struct Settings
  {
    int width = 0;
    int height = 0;
    double max_fps = 0.0;
    std::string codec, preset, tune;
    int threads = 4;
    int64_t bit_rate = 0;
    int vbv_buffer_ms = 0;
    double keyframe_interval = 1.0;
    std::string av_options;
  };

  struct Stats
  {
    uint64_t received = 0;
    uint64_t overwritten = 0;
    uint64_t throttled = 0;
    uint64_t encoded = 0;
    uint64_t keyframes = 0;
    uint64_t bytes = 0;
    size_t max_packet = 0;
    double convert_ms = 0.0;
    double encode_ms = 0.0;
    double latency_ms = 0.0;
  };

  // Parameters whose change needs the encoder reopened (and so costs a keyframe).
  inline static const std::set<std::string> kReopenParams = {
    "width", "height", "codec", "preset", "tune", "threads", "av_options"};

  inline static const std::vector<std::string> kLiveParams = {
    "width", "height", "max_fps", "codec", "preset", "tune", "threads", "bit_rate",
    "vbv_buffer_ms", "keyframe_interval", "av_options"};

  std::string resolve(const std::string & topic)
  {
    return get_node_topics_interface()->resolve_topic_name(topic);
  }

  // Validates and builds settings; throws std::invalid_argument on a bad value.
  Settings readSettings(const std::vector<rclcpp::Parameter> & params, Settings s) const
  {
    for (const auto & p : params) {
      const auto & n = p.get_name();
      if (n == "width") {s.width = static_cast<int>(p.as_int());}
      if (n == "height") {s.height = static_cast<int>(p.as_int());}
      if (n == "max_fps") {s.max_fps = p.as_double();}
      if (n == "codec") {s.codec = p.as_string();}
      if (n == "preset") {s.preset = p.as_string();}
      if (n == "tune") {s.tune = p.as_string();}
      if (n == "threads") {s.threads = static_cast<int>(p.as_int());}
      if (n == "bit_rate") {s.bit_rate = p.as_int();}
      if (n == "vbv_buffer_ms") {s.vbv_buffer_ms = static_cast<int>(p.as_int());}
      if (n == "keyframe_interval") {s.keyframe_interval = p.as_double();}
      if (n == "av_options") {s.av_options = p.as_string();}
    }
    if (s.width < 0 || s.height < 0) {throw std::invalid_argument("width/height must be >= 0");}
    if (s.max_fps < 0) {throw std::invalid_argument("max_fps must be >= 0");}
    if (s.bit_rate <= 0) {throw std::invalid_argument("bit_rate must be > 0");}
    if (s.vbv_buffer_ms < 0) {throw std::invalid_argument("vbv_buffer_ms must be >= 0");}
    if (s.threads < 0) {throw std::invalid_argument("threads must be >= 0");}
    if (!encoderExists(s.codec)) {throw std::invalid_argument("unknown encoder: " + s.codec);}
    parseAvOptions(s.av_options);
    return s;
  }

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    std::lock_guard<std::mutex> lock(mutex_);
    Settings next;
    try {
      next = readSettings(params, settings_);
    } catch (const std::exception & e) {
      result.successful = false;
      result.reason = e.what();
      return result;
    }
    for (const auto & p : params) {
      const auto & n = p.get_name();
      if (n == "bit_rate" || n == "vbv_buffer_ms") {
        rate_changed_ = true;
      } else if (kReopenParams.count(n)) {
        reopen_ = true;  // changes the codec setup
      }
    }
    settings_ = next;
    return result;
  }

  // Subscribes to the camera only while someone watches the output, and forces an IDR
  // whenever a viewer joins so it gets a picture straight away.
  void updateSubscriptions()
  {
    const size_t viewers = packet_pub_->get_subscription_count();
    if (viewers > last_viewers_) {
      std::lock_guard<std::mutex> lock(mutex_);
      force_keyframe_ = true;
    }
    last_viewers_ = viewers;

    if (viewers > 0 && !image_sub_) {
      // Keep-last 1: a frame we could not get to is replaced, never queued.
      image_sub_ = create_subscription<Image>(
        input_topic_, inputQos(),
        [this](Image::ConstSharedPtr msg) {onImage(std::move(msg));});
      if (!info_topic_.empty()) {
        info_sub_ = create_subscription<CameraInfo>(
          info_topic_, rclcpp::SensorDataQoS().keep_last(1),
          [this](CameraInfo::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            camera_info_ = std::move(msg);
          });
      }
      RCLCPP_INFO(get_logger(), "Teleop stream viewer connected to %s: subscribing to camera "
        "images on %s and encoding", output_name_.c_str(), input_name_.c_str());
    } else if (viewers == 0 && image_sub_) {
      image_sub_.reset();
      info_sub_.reset();
      std::lock_guard<std::mutex> lock(mutex_);
      pending_.reset();
      RCLCPP_INFO(get_logger(), "No viewers left on %s: unsubscribed from %s, encoder idle",
        output_name_.c_str(), input_name_.c_str());
    }
  }

  void onImage(Image::ConstSharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.received++;
      if (pending_) {
        stats_.overwritten++;
      }
      pending_ = std::move(msg);
    }
    cv_.notify_one();
  }

  void workerLoop()
  {
    while (true) {
      Image::ConstSharedPtr msg;
      Settings s;
      bool reopen = false;
      bool force_key = false;
      bool rate_changed = false;
      CameraInfo::ConstSharedPtr info;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] {return !running_ || pending_;});
        if (!running_) {
          return;
        }
        msg = std::move(pending_);
        pending_.reset();
        s = settings_;
        if (!admit(s.max_fps)) {
          stats_.throttled++;
          continue;
        }
        std::swap(reopen, reopen_);
        std::swap(force_key, force_keyframe_);
        std::swap(rate_changed, rate_changed_);
        info = camera_info_;
      }
      try {
        process(*msg, s, reopen, force_key, rate_changed, info);
      } catch (const std::exception & e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
          "Failed to encode a frame from %s: %s. Dropping it and reopening the encoder on the "
          "next frame", input_name_.c_str(), e.what());
        encoder_.close();  // reopened (with a keyframe) on the next frame
      }
    }
  }

  // Token bucket for max_fps: evenly thins e.g. 30 fps to 20 by keeping 2 frames in 3.
  // Admitting from 0.75 credit (borrowing the rest) tolerates timing jitter when the input
  // rate is at or just under max_fps; the long-run rate still cannot exceed max_fps.
  bool admit(double max_fps)
  {
    const auto now = SteadyClock::now();
    if (max_fps <= 0.0) {
      last_admit_ = now;
      return true;
    }
    const double dt = std::chrono::duration<double>(now - last_admit_).count();
    last_admit_ = now;
    credit_ = std::min(credit_ + dt * max_fps, 1.5);
    if (credit_ < 0.75) {
      return false;
    }
    credit_ -= 1.0;
    return true;
  }

  void process(
    const Image & msg, const Settings & s, bool reopen, bool force_key, bool rate_changed,
    const CameraInfo::ConstSharedPtr & info)
  {
    if (msg.width == 0 || msg.height == 0 || msg.step == 0 ||
      msg.data.size() < static_cast<size_t>(msg.step) * msg.height)
    {
      throw std::invalid_argument("malformed image: " + std::to_string(msg.width) + "x" +
              std::to_string(msg.height) + ", step " + std::to_string(msg.step) + ", " +
              std::to_string(msg.data.size()) + " bytes");
    }
    const int64_t stamp_ms = rclcpp::Time(msg.header.stamp).nanoseconds() / 1000000;
    // Stamps jumping back more than a second (sim reset, driver clock re-sync) would otherwise
    // be nudged forward 1 ms per frame, which rate control reads as ~1000 fps. Start afresh.
    const bool clock_jumped = last_pts_ >= 0 && stamp_ms < last_pts_ - 1000;
    if (clock_jumped) {
      RCLCPP_WARN(get_logger(), "Frame timestamps on %s jumped back %.1f s (sim reset or clock "
        "re-sync?); restarting the encoder", input_name_.c_str(), (last_pts_ - stamp_ms) / 1000.0);
    }

    const auto [out_w, out_h] = outputSize(
      static_cast<int>(msg.width), static_cast<int>(msg.height), s.width, s.height);
    const auto & cfg = encoder_.config();
    if (!encoder_.isOpen() || reopen || clock_jumped || msg.encoding != input_encoding_ ||
      out_w != cfg.width || out_h != cfg.height)
    {
      openEncoder(msg, s, out_w, out_h);
      force_key = true;
    } else if (rate_changed) {
      encoder_.setBitRate(s.bit_rate, s.vbv_buffer_ms);
      RCLCPP_INFO(get_logger(), "Teleop stream %s: bit rate now %.2f Mbit/s, VBV buffer %d ms",
        output_name_.c_str(), s.bit_rate / 1e6, s.vbv_buffer_ms);
    }

    const auto t0 = SteadyClock::now();
    toI420(msg.data.data(), static_cast<int>(msg.width), static_cast<int>(msg.height), msg.step,
      msg.encoding, out_w, out_h, scratch_, i420_);
    const auto t1 = SteadyClock::now();

    int64_t pts = stamp_ms;
    if (pts <= last_pts_) {
      pts = last_pts_ + 1;
    }
    last_pts_ = pts;
    if (s.keyframe_interval > 0.0 &&
      std::chrono::duration<double>(t0 - last_keyframe_).count() >= s.keyframe_interval)
    {
      force_key = true;
    }
    headers_[pts] = msg.header;

    encoder_.encode(i420_.data, pts, force_key,
      [&](const uint8_t * data, size_t size, int64_t packet_pts, bool key) {
        publishPacket(data, size, packet_pts, key, out_w, out_h);
      });
    const auto t2 = SteadyClock::now();
    // Packets normally come out in the same call, but keep a few frames' headers in case the
    // encoder is configured with delay (lookahead, B-frames).
    while (headers_.size() > 16) {
      headers_.erase(headers_.begin());
    }

    if (info && info_pub_->get_subscription_count() > 0) {
      publishCameraInfo(*info, msg.header, out_w, out_h);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    stats_.convert_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();
    stats_.encode_ms += std::chrono::duration<double, std::milli>(t2 - t1).count();
  }

  void openEncoder(const Image & msg, const Settings & s, int out_w, int out_h)
  {
    decoded_encoding_ = decodedEncoding(msg.encoding);
    if (decoded_encoding_.empty()) {
      throw std::invalid_argument("unsupported image encoding: " + msg.encoding);
    }
    EncoderConfig cfg;
    cfg.codec = s.codec;
    cfg.width = out_w;
    cfg.height = out_h;
    cfg.bit_rate = s.bit_rate;
    cfg.vbv_buffer_ms = s.vbv_buffer_ms;
    cfg.threads = s.threads;
    cfg.preset = s.preset;
    cfg.tune = s.tune;
    cfg.fps_hint = s.max_fps > 0.0 ? s.max_fps : 30.0;
    cfg.av_options = parseAvOptions(s.av_options);
    for (const auto & unused : encoder_.open(cfg)) {
      RCLCPP_WARN(get_logger(), "Encoder %s does not recognise av_options entry '%s'; ignoring it",
        s.codec.c_str(), unused.c_str());
    }
    input_encoding_ = msg.encoding;
    // Layout ffmpeg_image_transport 3.x expects: codec;av pixel format;cv_bridge format;
    // original encoding. The decoder outputs the last field.
    packet_encoding_ = encoder_.codecFamily() + ";yuv420p;bgr8;" + decoded_encoding_;
    last_pts_ = -1;
    headers_.clear();
    RCLCPP_INFO(
      get_logger(), "Encoding %s (%ux%u %s) to %s: %dx%d, %s preset %s, %d threads, target "
      "%.2f Mbit/s, VBV %d ms, keyframe every %.1f s", input_name_.c_str(), msg.width, msg.height,
      msg.encoding.c_str(), output_name_.c_str(), out_w, out_h, s.codec.c_str(), s.preset.c_str(),
      s.threads, s.bit_rate / 1e6, s.vbv_buffer_ms, s.keyframe_interval);
  }

  void publishPacket(
    const uint8_t * data, size_t size, int64_t pts, bool key, int width, int height)
  {
    auto it = headers_.find(pts);
    if (it == headers_.end()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Dropped an encoded packet (pts %ld) whose frame header is gone; happens only if the "
        "encoder is configured with delay (lookahead, B-frames)", pts);
      return;
    }
    auto out = std::make_unique<FFMPEGPacket>();
    out->header = it->second;
    headers_.erase(headers_.begin(), std::next(it));
    out->width = width;
    out->height = height;
    out->encoding = packet_encoding_;
    out->pts = static_cast<uint64_t>(pts);
    out->flags = key ? 1 : 0;
    out->is_bigendian = false;
    out->data.assign(data, data + size);
    // Node clock: wall time on the rover, /clock in the sim (use_sim_time), matching the stamps.
    const auto clock = get_clock();
    const double latency_ms =
      (clock->now() - rclcpp::Time(out->header.stamp, clock->get_clock_type())).seconds() * 1e3;
    packet_pub_->publish(std::move(out));

    if (key) {
      last_keyframe_ = SteadyClock::now();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.encoded++;
    stats_.keyframes += key;
    stats_.bytes += size;
    stats_.max_packet = std::max(stats_.max_packet, size);
    stats_.latency_ms += latency_ms;
  }

  void publishCameraInfo(
    const CameraInfo & in, const std_msgs::msg::Header & header, int width, int height)
  {
    auto out = std::make_unique<CameraInfo>(in);
    out->header = header;
    if (in.width > 0 && in.height > 0) {
      const double sx = static_cast<double>(width) / in.width;
      const double sy = static_cast<double>(height) / in.height;
      out->k[0] *= sx; out->k[2] *= sx; out->k[4] *= sy; out->k[5] *= sy;
      out->p[0] *= sx; out->p[2] *= sx; out->p[3] *= sx;
      out->p[5] *= sy; out->p[6] *= sy;
      out->roi.x_offset = static_cast<uint32_t>(in.roi.x_offset * sx);
      out->roi.y_offset = static_cast<uint32_t>(in.roi.y_offset * sy);
      out->roi.width = static_cast<uint32_t>(in.roi.width * sx);
      out->roi.height = static_cast<uint32_t>(in.roi.height * sy);
    }
    out->width = width;
    out->height = height;
    info_pub_->publish(std::move(out));
  }

  void publishStats()
  {
    Stats st;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      std::swap(st, stats_);
    }
    const auto now = SteadyClock::now();
    const double dt = std::chrono::duration<double>(now - last_stats_).count();
    last_stats_ = now;
    if (!image_sub_ && st.received == 0) {
      return;  // idle: nobody watching
    }
    const double n = std::max<double>(1.0, static_cast<double>(st.encoded));
    const double in_fps = st.received / dt;
    const double out_fps = st.encoded / dt;
    const double kbps = st.bytes * 8.0 / dt / 1000.0;
    RCLCPP_INFO(
      get_logger(),
      "Teleop stream %s: camera %.1f fps -> sent %.1f fps (%lu dropped as encoder busy, %lu "
      "skipped by max_fps), %.0f kbit/s, largest frame %.1f KB, %lu keyframes; per frame "
      "convert %.1f ms + encode %.1f ms; camera-to-publish latency %.0f ms",
      output_name_.c_str(), in_fps, out_fps, st.overwritten, st.throttled, kbps,
      st.max_packet / 1024.0, st.keyframes, st.convert_ms / n, st.encode_ms / n,
      st.latency_ms / n);

    DiagnosticArray arr;
    arr.header.stamp = now_ros();
    DiagnosticStatus status;
    status.name = std::string(get_fully_qualified_name());
    status.hardware_id = resolve(input_topic_);
    status.level = st.overwritten > st.encoded ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
    status.message = status.level == DiagnosticStatus::OK ? "streaming" : "dropping frames";
    auto kv = [&](const std::string & k, double v) {
        KeyValue x;
        x.key = k;
        x.value = std::to_string(v);
        status.values.push_back(x);
      };
    kv("in_fps", in_fps);
    kv("out_fps", out_fps);
    kv("overwritten", static_cast<double>(st.overwritten));
    kv("throttled", static_cast<double>(st.throttled));
    kv("kbit_per_s", kbps);
    kv("max_frame_kb", st.max_packet / 1024.0);
    kv("convert_ms", st.convert_ms / n);
    kv("encode_ms", st.encode_ms / n);
    kv("latency_ms", st.latency_ms / n);
    arr.status.push_back(status);
    diag_pub_->publish(arr);
  }

  builtin_interfaces::msg::Time now_ros() {return get_clock()->now();}

  // Configuration
  std::string input_topic_, info_topic_, output_topic_;
  std::string input_name_, output_name_;  // fully resolved, for logs
  bool input_reliable_ = false;

  // Keep-last 1 either way: a frame we could not get to is replaced, never queued. Across
  // processes a best-effort reader loses a whole multi-MB frame when one fragment is lost,
  // so reliable lets DDS resend the fragment instead (the publisher must be reliable too).
  rclcpp::QoS inputQos() const
  {
    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    if (input_reliable_) {
      qos.reliable();
    }
    return qos;
  }
  double stats_period_ = 5.0;

  // ROS interfaces
  rclcpp::Publisher<FFMPEGPacket>::SharedPtr packet_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<Image>::SharedPtr image_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  rclcpp::TimerBase::SharedPtr sub_timer_, stats_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
  size_t last_viewers_ = 0;
  SteadyClock::time_point last_stats_;

  // Shared with the worker, guarded by mutex_
  std::mutex mutex_;
  std::condition_variable cv_;
  bool running_ = true;
  Image::ConstSharedPtr pending_;
  CameraInfo::ConstSharedPtr camera_info_;
  Settings settings_;
  bool reopen_ = false;
  bool force_keyframe_ = false;
  bool rate_changed_ = false;
  Stats stats_;

  // Worker-only state
  std::thread worker_;
  H264Encoder encoder_;
  cv::Mat scratch_, i420_;
  std::string input_encoding_, decoded_encoding_, packet_encoding_;
  int64_t last_pts_ = -1;
  std::map<int64_t, std_msgs::msg::Header> headers_;
  SteadyClock::time_point last_keyframe_{};
  SteadyClock::time_point last_admit_{};
  double credit_ = 1.0;
};

}  // namespace rover_cameras

RCLCPP_COMPONENTS_REGISTER_NODE(rover_cameras::StreamEncoder)
