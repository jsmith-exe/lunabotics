// Tests for the CAN fixes. They run against the virtual interface vcan0 and never touch can0,
// so nothing reaches the real motors. Create vcan0 first (see test/run_can_tests.sh).

#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <hardware_interface/handle.hpp>
#include <rclcpp/logger.hpp>

#include "diffdrive_canbus/can_comms.hpp"
#include "diffdrive_canbus/diffdrive_interface.hpp"

using namespace diffdrive_canbus;
using namespace std::chrono_literals;

namespace
{
constexpr const char * TEST_IFACE = "vcan0";

constexpr uint32_t VELOCITY_SET_BASE = 0x02050480;  // api class 1, index 2
constexpr uint32_t DUTY_CYCLE_SET_BASE = 0x02050080; // api class 0, index 2
constexpr uint32_t STATUS_1_BASE = 0x02051840;       // api class 6, index 1
constexpr uint32_t POSITION_SET_BASE = 0x02050C80;   // api class 3, index 2

bool vcan_available()
{
  return if_nametoindex(TEST_IFACE) != 0;
}

// A plain raw socket on vcan0 that sees everything, standing in for the bus.
class BusTap
{
public:
  BusTap()
  {
    fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    sockaddr_can address {};
    address.can_family = AF_CAN;
    address.can_ifindex = static_cast<int>(if_nametoindex(TEST_IFACE));
    bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address));
  }
  ~BusTap() { ::close(fd_); }

  void send(uint32_t id, std::vector<uint8_t> data)
  {
    can_frame frame {};
    frame.can_id = id;
    frame.can_dlc = static_cast<uint8_t>(data.size());
    std::copy(data.begin(), data.end(), frame.data);
    ASSERT_EQ(write(fd_, &frame, sizeof(frame)), static_cast<ssize_t>(sizeof(frame)));
  }

  std::vector<can_frame> drain()
  {
    std::this_thread::sleep_for(5ms);
    std::vector<can_frame> frames;
    can_frame frame {};
    while (recv(fd_, &frame, sizeof(frame), MSG_DONTWAIT) == sizeof(frame)) {
      frames.push_back(frame);
    }
    return frames;
  }

private:
  int fd_ = -1;
};

int count_with_id(const std::vector<can_frame> & frames, uint32_t id)
{
  int count = 0;
  for (const auto & frame : frames) {
    if ((frame.can_id & CAN_EFF_MASK) == id) {
      ++count;
    }
  }
  return count;
}

class CanFixesTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!vcan_available()) {
      GTEST_SKIP() << "vcan0 not found; run test/run_can_tests.sh to create it";
    }
    can_.connect(TEST_IFACE);
  }

  SocketCanInterface can_;
  rclcpp::Logger logger_ = rclcpp::get_logger("test_can_fixes");
};
}  // namespace

// --- Socket lifecycle (fix 5) ---

TEST_F(CanFixesTest, CloseIsIdempotentAndMarksSocketClosed)
{
  EXPECT_TRUE(can_.is_open());
  can_.close();
  EXPECT_FALSE(can_.is_open());
  can_.close(); // Must not close an unrelated descriptor.
  EXPECT_FALSE(can_.is_open());
}

TEST_F(CanFixesTest, ReconnectReplacesSocket)
{
  can_.connect(TEST_IFACE);
  EXPECT_TRUE(can_.is_open());
  EXPECT_TRUE(can_.send_extended_frame(VELOCITY_SET_BASE + 1, std::vector<uint8_t>(8, 0)));
}

TEST_F(CanFixesTest, SendOnClosedSocketFailsAndIsCounted)
{
  can_.close();
  EXPECT_FALSE(can_.send_extended_frame(VELOCITY_SET_BASE + 1, std::vector<uint8_t>(8, 0)));
  EXPECT_EQ(can_.failed_sends(), 1u);
}

// --- Non-blocking reads and filtering (fix 4) ---

TEST_F(CanFixesTest, ReadReturnsImmediatelyWhenEmpty)
{
  can_frame frame {};
  while (can_.read_frame(frame)) {}
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(can_.read_frame(frame));
  EXPECT_LT(std::chrono::steady_clock::now() - start, 1ms);
}

TEST_F(CanFixesTest, FilterPassesOnlyRevMotorControllerFrames)
{
  BusTap tap;
  tap.send(0x123, {1, 2});                              // standard frame, not REV
  tap.send(CAN_EFF_FLAG | 0x01060000, {1, 2});          // extended, other manufacturer
  tap.send(CAN_EFF_FLAG | (STATUS_1_BASE + 3), {1, 2}); // REV status 1 from device 3
  std::this_thread::sleep_for(5ms);

  std::vector<can_frame> received;
  can_frame frame {};
  while (can_.read_frame(frame)) {
    received.push_back(frame);
  }
  ASSERT_EQ(received.size(), 1u);
  EXPECT_EQ(received[0].can_id & CAN_EFF_MASK, STATUS_1_BASE + 3);
}

// --- Commands resent every cycle (fix 1) ---

TEST_F(CanFixesTest, MotorResendsUnchangedSetpointEveryWrite)
{
  BusTap tap;
  DiffdriveMotor motor("wheel", 1, can_, 100.0f, logger_);
  std::vector<hardware_interface::CommandInterface> commands;
  motor.setup_ros_command_interfaces(commands);
  ASSERT_EQ(commands.size(), 1u);

  commands[0].set_value(0.5);
  motor.write();
  motor.write();
  motor.write();
  EXPECT_EQ(count_with_id(tap.drain(), VELOCITY_SET_BASE + 1), 3);

  commands[0].set_value(0.0); // A stop must also be resent every write.
  motor.write();
  motor.write();
  EXPECT_EQ(count_with_id(tap.drain(), VELOCITY_SET_BASE + 1), 2);
}

TEST_F(CanFixesTest, ActuatorResendsStopAfterResendPeriod)
{
  BusTap tap;
  Actuator::min_lift_mm = 25.0;
  Actuator::max_lift_mm = 225.0;
  Actuator actuator("lift", 5, can_, logger_);
  std::vector<hardware_interface::CommandInterface> commands;
  actuator.setup_ros_command_interfaces(commands);

  // Give the actuator a position reading, as status 3 would.
  can_frame status3 {};
  status3.can_id = CAN_EFF_FLAG | (SPARKMAX_PERIODIC_STATUS_3_BASE_ID + 5);
  status3.can_dlc = 8;
  status3.data[0] = 100;
  actuator.update_joint_state(status3);

  commands[0].set_value(0.0);
  actuator.write();
  actuator.write();
  EXPECT_EQ(count_with_id(tap.drain(), DUTY_CYCLE_SET_BASE + 5), 1);

  std::this_thread::sleep_for(ACTUATOR_RESEND_PERIOD + 50ms);
  actuator.write();
  EXPECT_EQ(count_with_id(tap.drain(), DUTY_CYCLE_SET_BASE + 5), 1);
}

// --- Feedback rates (fix 3) ---

TEST_F(CanFixesTest, MotorConfigureSetsFastVelocityStatus)
{
  BusTap tap;
  Motor motor("drum", 7, can_, 125.0f, logger_);
  motor.configure();

  bool found = false;
  for (const auto & frame : tap.drain()) {
    if ((frame.can_id & CAN_EFF_MASK) == STATUS_1_BASE + 7) {
      const uint16_t period = static_cast<uint16_t>(frame.data[0] | (frame.data[1] << 8));
      EXPECT_EQ(period, STATUS1_PERIOD_MS);
      EXPECT_LE(period, 50);
      found = true;
    }
  }
  EXPECT_TRUE(found);
}

// --- Dig guard: the lift only lowers while the drum is at its set speed ---

namespace
{
can_frame drum_status1(float rpm)
{
  can_frame frame {};
  frame.can_id = CAN_EFF_FLAG | (STATUS_1_BASE + 7);
  frame.can_dlc = 8;
  std::memcpy(frame.data, &rpm, sizeof(rpm)); // Little-endian float, as the SPARK MAX sends it
  return frame;
}

// raw 600 of 7..694 reads as about 194 mm, mid-travel.
can_frame actuator_status3(uint8_t can_id, uint16_t raw = 600)
{
  can_frame frame {};
  frame.can_id = CAN_EFF_FLAG | (SPARKMAX_PERIODIC_STATUS_3_BASE_ID + can_id);
  frame.can_dlc = 8;
  frame.data[0] = static_cast<uint8_t>(raw & 0xFF);
  frame.data[1] = static_cast<uint8_t>(raw >> 8);
  return frame;
}

class DigGuardTest : public CanFixesTest
{
protected:
  void SetUp() override
  {
    CanFixesTest::SetUp();
    if (IsSkipped()) return;
    DigGuard::enabled = true;
    DigGuard::pause_below_ratio = 0.8;
    DigGuard::resume_above_ratio = 0.9;
    Actuator::min_lift_mm = 25.0;
    Actuator::max_lift_mm = 225.0;
    drum_.setup_ros_command_interfaces(drum_commands_);
    actuator_.setup_ros_command_interfaces(actuator_commands_);
    actuator_.update_joint_state(actuator_status3(5));
  }

  void set_drum(double proportion, float measured_rpm)
  {
    drum_commands_[0].set_value(proportion);
    drum_.update_joint_state(drum_status1(measured_rpm));
  }

  Motor drum_{"drum", 7, can_, 125.0f, logger_};
  Actuator actuator_{"lift", 5, can_, logger_};
  DigGuard guard_{logger_};
  std::vector<hardware_interface::CommandInterface> drum_commands_;
  std::vector<hardware_interface::CommandInterface> actuator_commands_;
};
}  // namespace

TEST_F(DigGuardTest, BlocksLoweringWhenDrumNotSpinning)
{
  set_drum(0.0, 0.0f);
  EXPECT_FALSE(guard_.update(drum_));
}

TEST_F(DigGuardTest, AllowsLoweringAtSetSpeed)
{
  set_drum(0.5, 2800.0f); // 0.5 x 5600 RPM
  EXPECT_TRUE(guard_.update(drum_));
}

TEST_F(DigGuardTest, PausesWhenDrumSlowsAndResumesWithHysteresis)
{
  set_drum(0.5, 2800.0f);
  ASSERT_TRUE(guard_.update(drum_));

  set_drum(0.5, 2600.0f); // 93%: still digging freely
  EXPECT_TRUE(guard_.update(drum_));

  set_drum(0.5, 2000.0f); // 71%: hit something
  EXPECT_FALSE(guard_.update(drum_));

  set_drum(0.5, 2450.0f); // 88%: recovering, but not past the resume threshold
  EXPECT_FALSE(guard_.update(drum_));

  set_drum(0.5, 2600.0f); // 93%: dug through
  EXPECT_TRUE(guard_.update(drum_));
}

TEST_F(DigGuardTest, DrumSpinningBackwardsDoesNotCount)
{
  set_drum(0.5, -2800.0f);
  EXPECT_FALSE(guard_.update(drum_));
}

TEST_F(DigGuardTest, StaleDrumFeedbackBlocksLowering)
{
  set_drum(0.5, 2800.0f);
  ASSERT_TRUE(guard_.update(drum_));
  std::this_thread::sleep_for(DIG_GUARD_FEEDBACK_TIMEOUT + 50ms);
  EXPECT_FALSE(guard_.update(drum_));
}

TEST_F(DigGuardTest, DisabledAlwaysAllowsLowering)
{
  DigGuard::enabled = false;
  set_drum(0.0, 0.0f);
  EXPECT_TRUE(guard_.update(drum_));
}

TEST_F(DigGuardTest, PausedActuatorHoldsThenLowersWhenAllowed)
{
  BusTap tap;
  actuator_commands_[0].set_value(0.05); // Well below the ~194 mm reading

  actuator_.set_lowering_allowed(false);
  actuator_.write();
  auto frames = tap.drain();
  EXPECT_TRUE(actuator_.lowering_paused());
  EXPECT_EQ(count_with_id(frames, POSITION_SET_BASE + 5), 0);
  EXPECT_EQ(count_with_id(frames, DUTY_CYCLE_SET_BASE + 5), 1); // Stop, so it holds where it is

  actuator_.set_lowering_allowed(true);
  actuator_.write();
  frames = tap.drain();
  EXPECT_FALSE(actuator_.lowering_paused());
  EXPECT_EQ(count_with_id(frames, POSITION_SET_BASE + 5), 1); // Target sent straight away
}

TEST_F(DigGuardTest, RaisingIsNeverBlocked)
{
  BusTap tap;
  actuator_commands_[0].set_value(0.22); // Above the ~194 mm reading
  actuator_.set_lowering_allowed(false);
  actuator_.write();
  EXPECT_FALSE(actuator_.lowering_paused());
  EXPECT_EQ(count_with_id(tap.drain(), POSITION_SET_BASE + 5), 1);
}

TEST_F(DigGuardTest, SynchronisedCheckpointsDoNotAdvanceWhilePaused)
{
  SynchronisedActuator::max_distance_between_points_metres = 0.04;
  auto left = std::make_shared<SynchronisedActuator>("left", 5, can_, logger_, true);
  auto right = std::make_shared<SynchronisedActuator>("right", 6, can_, logger_, false);
  left->set_other_actuator(right);
  std::vector<hardware_interface::CommandInterface> commands;
  left->setup_ros_command_interfaces(commands);
  left->update_joint_state(actuator_status3(5));
  right->update_joint_state(actuator_status3(6));

  BusTap tap;
  commands[0].set_value(0.05); // About 144 mm down: several 40 mm checkpoints
  left->set_lowering_allowed(false);
  right->set_lowering_allowed(false);
  for (int i = 0; i < 5; ++i) left->write();
  tap.drain();

  left->set_lowering_allowed(true);
  right->set_lowering_allowed(true);
  left->write();
  bool found = false;
  for (const auto & frame : tap.drain()) {
    if ((frame.can_id & CAN_EFF_MASK) == POSITION_SET_BASE + 5) {
      float setpoint_mm;
      std::memcpy(&setpoint_mm, frame.data, sizeof(setpoint_mm));
      // First checkpoint is ~40 mm below the reading, sent with the -27.9 mm controller offset.
      EXPECT_GT(setpoint_mm, 194.0f - 40.0f - 27.9f - 5.0f);
      found = true;
    }
  }
  EXPECT_TRUE(found);
}
