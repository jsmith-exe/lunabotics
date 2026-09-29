#include <gtest/gtest.h>

#include <cmath>
#include <cstdarg>
#include <cstring>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "convert.hpp"
#include "h264_encoder.hpp"

extern "C" {
#include <libavutil/log.h>
}

using rover_cameras::EncoderConfig;
using rover_cameras::H264Encoder;

TEST(Convert, OutputSizeKeepsAspectAndEven)
{
  EXPECT_EQ(rover_cameras::outputSize(1920, 1080, 0, 0), std::make_pair(1920, 1080));
  EXPECT_EQ(rover_cameras::outputSize(1920, 1080, 1280, 0), std::make_pair(1280, 720));
  EXPECT_EQ(rover_cameras::outputSize(1280, 800, 0, 541), std::make_pair(864, 540));
  EXPECT_EQ(rover_cameras::outputSize(641, 481, 0, 0), std::make_pair(640, 480));
}

TEST(Convert, RejectsMalformedInput)
{
  EXPECT_THROW(rover_cameras::outputSize(0, 480, 640, 0), std::invalid_argument);
  std::vector<uint8_t> buf(64 * 48 * 3);
  cv::Mat scratch, i420;
  // step shorter than a row of bgr8 pixels
  EXPECT_THROW(
    rover_cameras::toI420(buf.data(), 64, 48, 64, "bgr8", 64, 48, scratch, i420),
    std::invalid_argument);
}

TEST(Encoder, EncoderExists)
{
  EXPECT_TRUE(rover_cameras::encoderExists("libx264"));
  EXPECT_FALSE(rover_cameras::encoderExists("libx265x"));
}

TEST(Convert, DecodedEncoding)
{
  EXPECT_EQ(rover_cameras::decodedEncoding("rgba8"), "rgb8");
  EXPECT_EQ(rover_cameras::decodedEncoding("bgr8"), "bgr8");
  EXPECT_EQ(rover_cameras::decodedEncoding("16UC1"), "");
}

TEST(Convert, I420MatchesOpenCvAndHandlesStrideAndResize)
{
  // Padded rows (step > width*3), as drivers sometimes publish.
  const int w = 64, h = 48;
  const size_t step = w * 3 + 16;
  std::vector<uint8_t> buf(step * h);
  cv::Mat padded(h, w, CV_8UC3, buf.data(), step);
  cv::randu(padded, 0, 255);

  cv::Mat scratch, i420, expected;
  rover_cameras::toI420(buf.data(), w, h, step, "bgr8", w, h, scratch, i420);
  cv::cvtColor(padded, expected, cv::COLOR_BGR2YUV_I420);
  ASSERT_EQ(i420.rows, h * 3 / 2);
  EXPECT_EQ(cv::norm(i420, expected, cv::NORM_INF), 0.0);

  rover_cameras::toI420(buf.data(), w, h, step, "rgb8", 32, 24, scratch, i420);
  EXPECT_EQ(i420.cols, 32);
  EXPECT_EQ(i420.rows, 36);
  EXPECT_TRUE(i420.isContinuous());

  EXPECT_THROW(
    rover_cameras::toI420(buf.data(), w, h, step, "16UC1", w, h, scratch, i420),
    std::invalid_argument);
}

TEST(Encoder, ParseAvOptions)
{
  const auto o = rover_cameras::parseAvOptions(" profile=main , x264-params=a=1:b=2 ");
  ASSERT_EQ(o.size(), 2u);
  EXPECT_EQ(o[1].first, "x264-params");
  EXPECT_EQ(o[1].second, "a=1:b=2");
  EXPECT_THROW(rover_cameras::parseAvOptions("novalue"), std::invalid_argument);
}

namespace
{
// Encodes n moving-noise frames spaced interval_ms apart; returns {kbit/s, keyframes}.
std::pair<double, int> encodeClip(
  H264Encoder & enc, int n, int interval_ms, int force_every, int64_t start_pts = 0)
{
  const int w = enc.config().width, h = enc.config().height;
  cv::Mat base(h * 2, w * 2, CV_8UC3);
  cv::randu(base, 0, 255);
  cv::GaussianBlur(base, base, cv::Size(5, 5), 0);
  cv::Mat scratch, i420, noise(h, w, CV_8UC3);
  size_t bytes = 0;
  int keys = 0;
  for (int i = 0; i < n; ++i) {
    cv::Mat frame = base(cv::Rect(i % w, (i * 3) % h, w, h)).clone();
    cv::randu(noise, 0, 12);
    frame += noise;
    rover_cameras::toI420(frame.data, w, h, frame.step, "bgr8", w, h, scratch, i420);
    const bool force = force_every > 0 && i > 0 && i % force_every == 0;
    enc.encode(i420.data, start_pts + static_cast<int64_t>(i) * interval_ms, force,
      [&](const uint8_t *, size_t size, int64_t, bool key) {
        bytes += i >= n / 3 ? size : 0;  // steady state: skip rate control's warm-up
        keys += key;
      });
  }
  const double seconds = (n - n / 3) * interval_ms / 1000.0;
  return {bytes * 8.0 / seconds / 1000.0, keys};
}

EncoderConfig testConfig()
{
  EncoderConfig c;
  c.width = 320;
  c.height = 240;
  c.bit_rate = 400000;
  c.threads = 2;
  return c;
}
}  // namespace

// The point of the 1/1000 time base: the bit rate holds at whatever rate frames arrive.
// (ffmpeg_image_transport's fixed 100 fps assumption made it scale with the frame rate.)
TEST(Encoder, BitRateHoldsAcrossFrameRates)
{
  for (int interval_ms : {33, 100}) {
    H264Encoder enc;
    enc.open(testConfig());
    const auto [kbps, keys] = encodeClip(enc, 150, interval_ms, 0);
    EXPECT_NEAR(kbps, 400.0, 400.0 * 0.25) << "interval " << interval_ms << " ms";
    EXPECT_GE(keys, 1);
  }
}

TEST(Encoder, ForcedKeyframesAreFlagged)
{
  H264Encoder enc;
  enc.open(testConfig());
  const auto [kbps, keys] = encodeClip(enc, 60, 33, 15);
  (void)kbps;
  EXPECT_EQ(keys, 4);  // first frame + frames 15, 30, 45
}

TEST(Encoder, LiveBitRateChange)
{
  H264Encoder enc;
  enc.open(testConfig());
  encodeClip(enc, 60, 33, 0);
  enc.setBitRate(150000, 200);
  const auto [kbps, keys] = encodeClip(enc, 150, 33, 0, 60 * 33);
  (void)keys;
  EXPECT_LT(kbps, 250.0);
}

TEST(Encoder, RejectsBadConfig)
{
  H264Encoder enc;
  auto c = testConfig();
  c.width = 321;
  EXPECT_THROW(enc.open(c), std::runtime_error);
  c = testConfig();
  c.codec = "no_such_codec";
  EXPECT_THROW(enc.open(c), std::runtime_error);
  c = testConfig();
  c.av_options = {{"not_an_option", "1"}};
  const auto unused = enc.open(c);
  ASSERT_EQ(unused.size(), 1u);
  EXPECT_EQ(unused[0], "not_an_option=1");
}

// Packets must come back with the pts they went in with: the node maps pts back to the
// frame's ROS header. Header stamps are epoch milliseconds, so test at that scale too.
TEST(Encoder, PacketPtsMatchInput)
{
  for (int64_t start : {int64_t{0}, int64_t{1790542118349}}) {
    H264Encoder enc;
    enc.open(testConfig());
    cv::Mat frame(240, 320, CV_8UC3), scratch, i420;
    std::vector<int64_t> in, out;
    for (int i = 0; i < 10; ++i) {
      cv::randu(frame, 0, 255);
      rover_cameras::toI420(frame.data, 320, 240, frame.step, "bgr8", 320, 240, scratch, i420);
      in.push_back(start + i * 33);
      enc.encode(i420.data, in.back(), false,
        [&](const uint8_t *, size_t, int64_t pts, bool) {out.push_back(pts);});
    }
    EXPECT_EQ(in, out) << "start " << start;
  }
}

namespace
{
int g_vbv_warnings = 0;
void countVbvWarnings(void *, int, const char * fmt, va_list)
{
  if (fmt && std::strstr(fmt, "VBV buffer size cannot be smaller than one frame")) {
    ++g_vbv_warnings;
  }
}
}  // namespace

// A slow, jittery source (e.g. Gazebo at ~2 fps): the bit rate must still hold, and the VBV
// buffer must never be smaller than one frame (libx264 warns and overrides it if so).
TEST(Encoder, SlowJitteryFrameRate)
{
  av_log_set_callback(countVbvWarnings);
  g_vbv_warnings = 0;
  H264Encoder enc;
  enc.open(testConfig());
  const int w = 320, h = 240;
  cv::Mat base(h * 2, w * 2, CV_8UC3), frame, noise(h, w, CV_8UC3), scratch, i420;
  cv::randu(base, 0, 255);
  cv::GaussianBlur(base, base, cv::Size(5, 5), 0);
  size_t bytes = 0;
  int64_t pts = 0, measured_ms = 0;
  for (int i = 0; i < 90; ++i) {
    pts += (i % 2) ? 350 : 550;  // ~2.2 fps with jitter
    frame = base(cv::Rect(i % w, (i * 3) % h, w, h)).clone();
    cv::randu(noise, 0, 12);
    frame += noise;
    rover_cameras::toI420(frame.data, w, h, frame.step, "bgr8", w, h, scratch, i420);
    enc.encode(i420.data, pts, false, [&](const uint8_t *, size_t size, int64_t, bool) {
        if (i >= 30) {bytes += size;}
      });
    if (i == 29) {measured_ms = pts;}
  }
  av_log_set_callback(av_log_default_callback);
  const double kbps = bytes * 8.0 / ((pts - measured_ms) / 1000.0) / 1000.0;
  EXPECT_NEAR(kbps, 400.0, 400.0 * 0.3);
  EXPECT_EQ(g_vbv_warnings, 0);
}
