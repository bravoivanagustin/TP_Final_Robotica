#include <rclcpp/rclcpp.hpp>
#include "mecanum_odometry.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robmovil::MecanumOdometry>());
  rclcpp::shutdown();
  return 0;
}
