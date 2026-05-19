#include <rclcpp/rclcpp.hpp>
#include "lazo_cerrado/KinematicHolonomicController.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robmovil::KinematicHolonomicController>();
  rclcpp::spin(node);
  node->stop_timer();
  rclcpp::shutdown();
  return 0;
}
