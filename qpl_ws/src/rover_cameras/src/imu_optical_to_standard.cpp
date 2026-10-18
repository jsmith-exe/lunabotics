// Re-publishes the RealSense IMU in ROS axes (x forward, y left, z up) for the EKFs.
//
// Loaded into the RealSense container with intra-process comms, so the ~100 Hz IMU
// reaches it without serialisation. Replaces a Python node that cost ~0.4 of a core.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace rover_cameras
{

using sensor_msgs::msg::Imu;

class ImuOpticalToStandard : public rclcpp::Node
{
public:
  explicit ImuOpticalToStandard(const rclcpp::NodeOptions & options)
  : Node("imu_optical_to_standard", options)
  {
    // Best effort in to match the driver; reliable out for the EKFs and Madgwick.
    pub_ = create_publisher<Imu>("/camera/camera/imu_standard", rclcpp::QoS(10).reliable());
    sub_ = create_subscription<Imu>(
      "/camera/camera/imu", rclcpp::QoS(10).best_effort(),
      [this](Imu::ConstSharedPtr msg) {on_imu(*msg);});
    RCLCPP_INFO(
      get_logger(), "Converting RealSense IMU from optical to ROS axes: "
      "/camera/camera/imu -> /camera/camera/imu_standard (frame camera_imu_frame)");
  }

private:
  void on_imu(const Imu & in)
  {
    auto out = std::make_unique<Imu>();
    out->header = in.header;
    out->header.frame_id = "camera_imu_frame";  // standard-convention frame

    // Rotation: x_std = z_opt, y_std = -x_opt, z_std = -y_opt
    out->angular_velocity.x = in.angular_velocity.z;
    out->angular_velocity.y = -in.angular_velocity.x;
    out->angular_velocity.z = -in.angular_velocity.y;

    out->linear_acceleration.x = in.linear_acceleration.z;
    out->linear_acceleration.y = -in.linear_acceleration.x;
    out->linear_acceleration.z = -in.linear_acceleration.y;

    // Orientation unused (use_mag: False path doesn't trust it anyway)
    out->orientation_covariance[0] = -1.0;

    // Copied unrotated, as the Python node did.
    out->angular_velocity_covariance = in.angular_velocity_covariance;
    out->linear_acceleration_covariance = in.linear_acceleration_covariance;

    pub_->publish(std::move(out));
  }

  rclcpp::Publisher<Imu>::SharedPtr pub_;
  rclcpp::Subscription<Imu>::SharedPtr sub_;
};

}  // namespace rover_cameras

RCLCPP_COMPONENTS_REGISTER_NODE(rover_cameras::ImuOpticalToStandard)
