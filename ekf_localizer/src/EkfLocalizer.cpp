// EkfLocalizer.cpp  —  Etapa C: EKF de localización con Eigen puro
//
// Estado:  x = [px, py, theta]^T  (frame map)
// Control: u = [dxb, dyb, dth]^T  (twist * dt, en body frame)
// Medición: z = [range, bearing]^T por landmark (frame base_link)
//
// Predicción:  en on_odometry  → processModel / computeJf
// Corrección:  en on_landmarks → measurementModel / computeJh  (secuencial por landmark)
// Mapa:        en on_posts (QoS transient_local) → map_landmarks_
// Init estado: tryInitFromTf → TF map→base_link (NUNCA desde /robot/odometry.pose)
// TF publicada: map→base_link_ekf  con stamp del mensaje de entrada (NUNCA now())

#include "ekf_localizer/EkfLocalizer.h"

#include <cmath>
#include <limits>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace robmovil
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

EkfLocalizer::EkfLocalizer()
  : Node("ekf_localizer")
{
  // -- Parámetros -----------------------------------------------------------
  map_frame_            = this->declare_parameter("map_frame",            std::string("map"));
  ekf_frame_            = this->declare_parameter("ekf_frame",            std::string("base_link_ekf"));
  sigma_dx_             = this->declare_parameter("sigma_dx",             0.15);
  sigma_dy_             = this->declare_parameter("sigma_dy",             0.15);
  sigma_dth_            = this->declare_parameter("sigma_dth",            0.15);
  sigma_range_          = this->declare_parameter("sigma_range",          0.05);
  sigma_bearing_        = this->declare_parameter("sigma_bearing",        0.03);
  init_cov_             = this->declare_parameter("init_cov",             0.01);
  association_max_dist_ = this->declare_parameter("association_max_dist", 0.6);
  min_range_            = this->declare_parameter("min_range",            0.2);

  // -- Matrices del filtro --------------------------------------------------
  // Q_ = diag(sigma_dx², sigma_dy², sigma_dth²)  — NO escala por dt
  Q_ = Eigen::Matrix3d::Zero();
  Q_(0, 0) = sigma_dx_  * sigma_dx_;
  Q_(1, 1) = sigma_dy_  * sigma_dy_;
  Q_(2, 2) = sigma_dth_ * sigma_dth_;

  // R_ = diag(sigma_range², sigma_bearing²)
  R_ = Eigen::Matrix2d::Zero();
  R_(0, 0) = sigma_range_   * sigma_range_;
  R_(1, 1) = sigma_bearing_ * sigma_bearing_;

  // Estado inicial y covarianza; se sobreescribirán en tryInitFromTf.
  x_.setZero();
  P_ = Eigen::Matrix3d::Identity() * init_cov_;

  // -- TF -------------------------------------------------------------------
  tf_buffer_      = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // -- Suscripciones --------------------------------------------------------
  // /posts: latched por CoppeliaSim → QoS transient_local + reliable obligatorio.
  auto posts_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  posts_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/posts", posts_qos,
      std::bind(&EkfLocalizer::on_posts, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/robot/odometry", rclcpp::QoS(10),
      std::bind(&EkfLocalizer::on_odometry, this, std::placeholders::_1));

  landmarks_sub_ = this->create_subscription<robmovil_msgs::msg::LandmarkArray>(
      "/landmarks", rclcpp::QoS(10),
      std::bind(&EkfLocalizer::on_landmarks, this, std::placeholders::_1));

  // -- Publicaciones --------------------------------------------------------
  ekf_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/robot/odometry_ekf", rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(),
      "EkfLocalizer listo. map=%s ekf=%s gate=%.2f "
      "sigma_dx=%.3f sigma_dy=%.3f sigma_dth=%.3f "
      "sigma_range=%.3f sigma_bearing=%.3f init_cov=%.4f",
      map_frame_.c_str(), ekf_frame_.c_str(), association_max_dist_,
      sigma_dx_, sigma_dy_, sigma_dth_,
      sigma_range_, sigma_bearing_, init_cov_);
}

// ---------------------------------------------------------------------------
// on_posts: carga el mapa de postes (una sola vez, transient_local)
// ---------------------------------------------------------------------------

void EkfLocalizer::on_posts(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  map_landmarks_.clear();
  map_landmarks_.reserve(msg->poses.size());
  for (const auto& p : msg->poses) {
    map_landmarks_.emplace_back(p.position.x, p.position.y);
  }
  RCLCPP_INFO(this->get_logger(), "Mapa recibido: %zu postes en frame '%s'.",
              map_landmarks_.size(), msg->header.frame_id.c_str());
}

// ---------------------------------------------------------------------------
// tryInitFromTf: inicializa x_ y P_ desde TF map→base_link
// ---------------------------------------------------------------------------

bool EkfLocalizer::tryInitFromTf()
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    // TimePointZero: tomar la última TF disponible (no exigir stamp exacto).
    tf = tf_buffer_->lookupTransform(map_frame_, "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF %s -> base_link no disponible para inicializar EKF: %s",
        map_frame_.c_str(), ex.what());
    return false;
  }

  x_(0) = tf.transform.translation.x;
  x_(1) = tf.transform.translation.y;
  x_(2) = tf2::getYaw(tf.transform.rotation);
  P_    = Eigen::Matrix3d::Identity() * init_cov_;
  initialized_ = true;

  RCLCPP_INFO(this->get_logger(),
      "Estado EKF inicializado desde TF: x=%.3f y=%.3f theta=%.3f",
      x_(0), x_(1), x_(2));
  return true;
}

// ---------------------------------------------------------------------------
// on_odometry: paso de predicción EKF
// ---------------------------------------------------------------------------

void EkfLocalizer::on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);

  // Intentar inicializar el estado EKF desde TF hasta que esté disponible.
  if (!initialized_) {
    tryInitFromTf();
    // Aunque falle, guardar el stamp para la primera iteración productiva.
    last_stamp_        = stamp;
    stamp_initialized_ = true;
    return;
  }

  // Guard: stamps duplicados o regresivos (común al arrancar la sim).
  // Mismo patrón que mecanum_odometry.cpp:85-92.
  const double dt = (stamp - last_stamp_).seconds();
  if (dt <= 1e-6) {
    last_stamp_ = stamp;
    return;
  }

  // Control u = twist * dt  (twist de /robot/odometry está en frame body, m/s y rad/s).
  Eigen::Vector3d u;
  u(0) = msg->twist.twist.linear.x  * dt;   // dxb  [m]
  u(1) = msg->twist.twist.linear.y  * dt;   // dyb  [m]
  u(2) = msg->twist.twist.angular.z * dt;   // dth  [rad]

  // --- Predicción EKF ---
  // Jacobiano evaluado en x ANTES de la predicción.
  const Eigen::Matrix3d F = computeJf(x_, u);
  x_ = processModel(x_, u);
  P_ = F * P_ * F.transpose() + Q_;   // Q_ sin factor dt (u ya son deltas de distancia)

  last_stamp_ = stamp;
  publishEkfPose(stamp);
}

// ---------------------------------------------------------------------------
// on_landmarks: paso de corrección EKF (secuencial por landmark)
// ---------------------------------------------------------------------------

void EkfLocalizer::on_landmarks(const robmovil_msgs::msg::LandmarkArray::SharedPtr msg)
{
  if (!initialized_ || map_landmarks_.empty()) {
    return;
  }

  const rclcpp::Time stamp(msg->header.stamp);

  bool corrected = false;

  for (const auto& lm : msg->landmarks) {
    const double r_obs = static_cast<double>(lm.range);
    const double b_obs = static_cast<double>(lm.bearing);

    // Descartar landmarks numéricamente inestables (demasiado cercanos → Jh singular).
    if (r_obs < min_range_) {
      continue;
    }

    // Proyectar la observación a frame map usando la pose actual del EKF.
    const double lx = x_(0) + r_obs * std::cos(b_obs + x_(2));
    const double ly = x_(1) + r_obs * std::sin(b_obs + x_(2));

    // Nearest-neighbor con gating por distancia euclidiana en frame map.
    int    best_j  = -1;
    double best_d2 = association_max_dist_ * association_max_dist_;
    for (size_t j = 0; j < map_landmarks_.size(); ++j) {
      const double dx = lx - map_landmarks_[j](0);
      const double dy = ly - map_landmarks_[j](1);
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        best_j  = static_cast<int>(j);
      }
    }

    // Si ningún poste del mapa supera el gating, es un falso positivo → ignorar.
    if (best_j < 0) {
      continue;
    }

    const double mx = map_landmarks_[best_j](0);
    const double my = map_landmarks_[best_j](1);

    // --- Corrección EKF (actualización 2×2 por landmark) ---
    const Eigen::Vector2d              h_pred = measurementModel(x_, mx, my);
    const Eigen::Matrix<double, 2, 3>  H      = computeJh(x_, mx, my);

    const Eigen::Matrix2d              S = H * P_ * H.transpose() + R_;
    const Eigen::Matrix<double, 3, 2>  K = P_ * H.transpose() * S.inverse();

    Eigen::Vector2d innov;
    innov(0) = r_obs - h_pred(0);
    // CRITICO: normalizar el residuo de bearing a (-π, π] antes de aplicar la ganancia.
    innov(1) = angles::normalize_angle(b_obs - h_pred(1));

    x_ += K * innov;
    x_(2) = angles::normalize_angle(x_(2));

    // Forma de Joseph: P = (I-KH)·P·(I-KH)ᵀ + K·R·Kᵀ
    // Preserva simetría y positividad ante errores de punto flotante acumulados.
    const Eigen::Matrix3d I_KH = Eigen::Matrix3d::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose();

    corrected = true;
  }

  // Solo publicar si al menos un landmark fue usado para la corrección.
  // Evita sobreescribir la TF de predicción (on_odometry) con un stamp más antiguo.
  if (corrected) {
    publishEkfPose(stamp);
  }
}

// ---------------------------------------------------------------------------
// publishEkfPose: TF map→base_link_ekf + /robot/odometry_ekf con covarianza
// ---------------------------------------------------------------------------

void EkfLocalizer::publishEkfPose(const rclcpp::Time& stamp)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, x_(2));

  // --- TF map → base_link_ekf ---
  // stamp = stamp del mensaje de entrada; NUNCA this->now() (rompería use_sim_time).
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp    = stamp;
  t.header.frame_id = map_frame_;
  t.child_frame_id  = ekf_frame_;
  t.transform.translation.x = x_(0);
  t.transform.translation.y = x_(1);
  t.transform.translation.z = 0.0;
  t.transform.rotation      = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);

  // --- /robot/odometry_ekf con covarianza poblada ---
  nav_msgs::msg::Odometry odom;
  odom.header.stamp    = stamp;
  odom.header.frame_id = map_frame_;
  odom.child_frame_id  = ekf_frame_;

  odom.pose.pose.position.x  = x_(0);
  odom.pose.pose.position.y  = x_(1);
  odom.pose.pose.position.z  = 0.0;
  odom.pose.pose.orientation = tf2::toMsg(q);

  // Mapear P_ (3×3, orden [x, y, theta]) a covariance[36] row-major 6×6 [x,y,z,roll,pitch,yaw].
  // Índices: 0=xx 1=xy 5=xθ | 6=yx 7=yy 11=yθ | 30=θx 31=θy 35=θθ
  auto& cov = odom.pose.covariance;
  cov[0]  = P_(0, 0);   cov[1]  = P_(0, 1);   cov[5]  = P_(0, 2);
  cov[6]  = P_(1, 0);   cov[7]  = P_(1, 1);   cov[11] = P_(1, 2);
  cov[30] = P_(2, 0);   cov[31] = P_(2, 1);   cov[35] = P_(2, 2);

  ekf_pub_->publish(odom);
}

// ---------------------------------------------------------------------------
// Modelo de proceso: f(x, u)
// Integración Mecanum — idéntica a mecanum_odometry.cpp:108-110
// u = [dxb, dyb, dth]  (deltas en body frame)
// ---------------------------------------------------------------------------

Eigen::Vector3d EkfLocalizer::processModel(const Eigen::Vector3d& x,
                                             const Eigen::Vector3d& u) const
{
  const double th  = x(2);
  const double c   = std::cos(th);
  const double s   = std::sin(th);
  const double dxb = u(0);
  const double dyb = u(1);
  const double dth = u(2);

  Eigen::Vector3d xp;
  xp(0) = x(0) + dxb * c - dyb * s;
  xp(1) = x(1) + dxb * s + dyb * c;
  xp(2) = angles::normalize_angle(x(2) + dth);
  return xp;
}

// ---------------------------------------------------------------------------
// Jacobiano del proceso: Jf = df/dx  (3×3), evaluado en (x, u)
// F = I  excepto:
//   F[0,2] = -(dxb·sin(θ) + dyb·cos(θ))
//   F[1,2] =   dxb·cos(θ) - dyb·sin(θ)
// ---------------------------------------------------------------------------

Eigen::Matrix3d EkfLocalizer::computeJf(const Eigen::Vector3d& x,
                                          const Eigen::Vector3d& u) const
{
  const double th  = x(2);
  const double c   = std::cos(th);
  const double s   = std::sin(th);
  const double dxb = u(0);
  const double dyb = u(1);

  Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
  F(0, 2) = -(dxb * s + dyb * c);   // d(px)/d(theta)
  F(1, 2) =   dxb * c - dyb * s;    // d(py)/d(theta)
  return F;
}

// ---------------------------------------------------------------------------
// Modelo de medición: h(x, m) = [range, bearing]^T al poste (mx, my)
//   dx = mx - px,  dy = my - py,  q = dx²+dy²
//   h = [sqrt(q),  normalize(atan2(dy,dx) - theta)]^T
// ---------------------------------------------------------------------------

Eigen::Vector2d EkfLocalizer::measurementModel(const Eigen::Vector3d& x,
                                                 double mx, double my) const
{
  const double dx = mx - x(0);
  const double dy = my - x(1);
  const double q  = dx * dx + dy * dy;

  Eigen::Vector2d z;
  z(0) = std::sqrt(q);
  z(1) = angles::normalize_angle(std::atan2(dy, dx) - x(2));
  return z;
}

// ---------------------------------------------------------------------------
// Jacobiano de medición: Jh = dh/dx  (2×3)
//   ⎡ -dx/r   -dy/r    0  ⎤    donde r = sqrt(q)
//   ⎣  dy/q   -dx/q   -1  ⎦    donde q = dx²+dy²
// ---------------------------------------------------------------------------

Eigen::Matrix<double, 2, 3> EkfLocalizer::computeJh(const Eigen::Vector3d& x,
                                                      double mx, double my) const
{
  const double dx = mx - x(0);
  const double dy = my - x(1);
  const double q  = dx * dx + dy * dy;
  const double r  = std::sqrt(q);

  Eigen::Matrix<double, 2, 3> H;
  H(0, 0) = -dx / r;    H(0, 1) = -dy / r;    H(0, 2) =  0.0;
  H(1, 0) =  dy / q;    H(1, 1) = -dx / q;    H(1, 2) = -1.0;
  return H;
}

}  // namespace robmovil
