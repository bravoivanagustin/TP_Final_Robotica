#include "ekf_localizer/EkfLocalizer.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robmovil::EkfLocalizer>());
  rclcpp::shutdown();
  return 0;
}
