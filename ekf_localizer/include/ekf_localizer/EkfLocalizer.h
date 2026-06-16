#pragma once

// EkfLocalizer: localización de robot Mecanum 4 ruedas mediante EKF
// con observaciones range-bearing a postes conocidos (frame map).
//
// Estado:    x = [px, py, theta]^T  (frame map)
// Control:   u = [dxb, dyb, dth]^T  (deltas en body frame, de /robot/odometry)
// Medición:  z = [range, bearing]^T  por landmark detectado (frame base_link)
//
// Etapa B: solo modelo matemático. La lógica ROS (callbacks, publicación)
// se implementa en Etapa C.

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <robmovil_msgs/msg/landmark_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <angles/angles.h>

namespace robmovil
{

class EkfLocalizer : public rclcpp::Node
{
public:

  EkfLocalizer();

  // Callbacks ROS 2
  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_landmarks(const robmovil_msgs::msg::LandmarkArray::SharedPtr msg);
  void on_posts(const geometry_msgs::msg::PoseArray::SharedPtr msg);

private:

  // -----------------------------------------------------------------------
  // Estado EKF
  // -----------------------------------------------------------------------

  Eigen::Vector3d x_;   // [px, py, theta] en frame map
  Eigen::Matrix3d P_;   // covarianza de estado 3×3
  Eigen::Matrix3d Q_;   // ruido de proceso diag(0.15², 0.15², 0.15²) — NO escala por dt
  Eigen::Matrix2d R_;   // ruido de medición diag(0.05², 0.03²)

  bool initialized_ = false;

  // -----------------------------------------------------------------------
  // Parámetros (valores default indicados en comentario)
  // -----------------------------------------------------------------------

  double sigma_dx_;           // default 0.15  — desvío ruido en dxb  (m/paso)
  double sigma_dy_;           // default 0.15  — desvío ruido en dyb  (m/paso)
  double sigma_dth_;          // default 0.15  — desvío ruido en dth  (rad/paso)
  double sigma_range_;        // default 0.05  — desvío medición range  (m)
  double sigma_bearing_;      // default 0.03  — desvío medición bearing (rad)
  double init_cov_;           // default 0.01  — diagonal inicial de P₀
  double association_max_dist_; // default 0.6 — distancia máxima para asociación de datos (m)
  double min_range_;          // default 0.2   — rango mínimo válido para evitar Jh singular (m)

  std::string map_frame_;     // default "map"
  std::string ekf_frame_;     // default "base_link_ekf"

  // -----------------------------------------------------------------------
  // Métodos del modelo matemático
  // -----------------------------------------------------------------------

  // f(x, u): integración Mecanum — mecanum_odometry.cpp:108-110
  //   x⁺ = x + dxb·cos(θ) - dyb·sin(θ)
  //   y⁺ = y + dxb·sin(θ) + dyb·cos(θ)
  //   θ⁺ = normalize(θ + dth)
  Eigen::Vector3d processModel(const Eigen::Vector3d& x,
                                const Eigen::Vector3d& u) const;

  // Jf = df/dx (3×3, evaluado en x̄, sin factor dt)
  //   ⎡ 1  0  -dxb·sin(θ) - dyb·cos(θ) ⎤
  //   ⎢ 0  1   dxb·cos(θ) - dyb·sin(θ) ⎥
  //   ⎣ 0  0             1              ⎦
  Eigen::Matrix3d computeJf(const Eigen::Vector3d& x,
                             const Eigen::Vector3d& u) const;

  // h(x, m): range-bearing esperado al landmark (mx, my)
  //   dx = mx - px,  dy = my - py,  q = dx²+dy²
  //   h = [sqrt(q),  normalize(atan2(dy, dx) - θ)]^T
  Eigen::Vector2d measurementModel(const Eigen::Vector3d& x,
                                    double mx, double my) const;

  // Jh = dh/dx (2×3, con r = sqrt(q))
  //   ⎡ -dx/r   -dy/r    0  ⎤
  //   ⎣  dy/q   -dx/q   -1  ⎦
  Eigen::Matrix<double, 2, 3> computeJh(const Eigen::Vector3d& x,
                                         double mx, double my) const;

  // -----------------------------------------------------------------------
  // Auxiliares privados
  // -----------------------------------------------------------------------

  bool tryInitFromTf();
  void publishEkfPose(const rclcpp::Time& stamp);

  // -----------------------------------------------------------------------
  // Infraestructura ROS 2
  // -----------------------------------------------------------------------

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr             odom_sub_;
  rclcpp::Subscription<robmovil_msgs::msg::LandmarkArray>::SharedPtr   landmarks_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr       posts_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                ekf_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster>  tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer>                tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>     tf_listener_;

  // Mapa de postes (mx, my en frame map)
  std::vector<Eigen::Vector2d>  map_landmarks_;

  // Stamp del último mensaje de odometría procesado (para calcular dt)
  rclcpp::Time  last_stamp_;
  bool          stamp_initialized_ = false;
};

}  // namespace robmovil
