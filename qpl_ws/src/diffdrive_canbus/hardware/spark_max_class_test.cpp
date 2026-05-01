#include "diffdrive_canbus/spark_max.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

using diffdrive_canbus::CANComms;
using diffdrive_canbus::CANMode;
using diffdrive_canbus::SparkMax;

namespace
{

void run_velocity_for(
  SparkMax & spark,
  float rpm,
  std::chrono::milliseconds duration,
  std::chrono::milliseconds period = std::chrono::milliseconds(10))
{
  const auto start = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start < duration)
  {
    spark.send_heartbeats(true);
    spark.set_velocity_rpm(rpm, true);

    std::this_thread::sleep_for(period);
  }
}

void stop_for(
  SparkMax & spark,
  std::chrono::milliseconds duration,
  std::chrono::milliseconds period = std::chrono::milliseconds(10))
{
  const auto start = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start < duration)
  {
    spark.send_heartbeats(true);

    // Use duty stop rather than velocity 0 for now.
    // This makes sure the output is commanded off.
    spark.set_duty_cycle(0.0f, true);

    std::this_thread::sleep_for(period);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3)
  {
    std::cerr << "Usage:\n"
              << "  " << argv[0] << " /dev/ttyUSB0 <spark_can_id> [rpm]\n\n"
              << "Example:\n"
              << "  " << argv[0] << " /dev/ttyUSB0 1 500\n";
    return 1;
  }

  const std::string serial_device = argv[1];
  const uint8_t spark_id = static_cast<uint8_t>(std::stoi(argv[2]));

  float rpm = 500.0f;
  if (argc >= 4)
  {
    rpm = std::stof(argv[3]);
  }

  if (rpm < -3000.0f || rpm > 3000.0f)
  {
    std::cerr << "Refusing to run RPM outside +/-3000 for this test.\n";
    return 1;
  }

  CANComms can;

  try
  {
    can.connect(serial_device, 2000000, 20);

    if (!can.configure_adapter(
          1000000,
          false,
          0x00000000,
          0x00000000,
          CANMode::NORMAL,
          false,
          true))
    {
      std::cerr << "Failed to configure CAN adapter\n";
      return 1;
    }

    SparkMax spark(can, spark_id);

    std::cout << "\nConnected.\n";
    std::cout << "SPARK MAX ID: " << static_cast<int>(spark.device_id()) << "\n";
    std::cout << "Velocity target: " << rpm << " RPM\n\n";

    std::cout << "Printed heartbeat pair:\n";
    spark.send_heartbeats(true);

    std::cout << "\nClearing faults...\n";
    for (int i = 0; i < 5; ++i)
    {
      spark.clear_faults(true);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "\nArming for 2 seconds...\n";
    stop_for(spark, std::chrono::seconds(2));

    std::cout << "\nRunning velocity for 3 seconds...\n";
    run_velocity_for(spark, rpm, std::chrono::seconds(3));

    std::cout << "\nStopping for 2 seconds...\n";
    stop_for(spark, std::chrono::seconds(2));

    can.disconnect();
  }
  catch (const std::exception & e)
  {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}