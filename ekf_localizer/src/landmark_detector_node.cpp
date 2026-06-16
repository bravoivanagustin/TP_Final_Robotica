#include "ekf_localizer/LandmarkDetector.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robmovil::LandmarkDetector>());
  rclcpp::shutdown();
  return 0;
}
