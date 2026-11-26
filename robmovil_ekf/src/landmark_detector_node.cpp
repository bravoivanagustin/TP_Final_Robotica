#include "landmark_detector.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robmovil_ekf::LandmarkDetector>());
  rclcpp::shutdown();
  return 0;
}
