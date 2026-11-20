#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include "localizer.h"

robmovil_ekf::Localizer::Localizer() :
  Node("localizer_node"), initialized(false), u(3)
{
  u(1) = u(2) = u(3) = 0;

  // Get node parameters
  base_frame_ = this->declare_parameter("base_link_frame", std::string("base_link_ekf"));
  map_frame_ = this->declare_parameter("ekf_frame", std::string("map"));
  odom_base_frame_ = this->declare_parameter("odom_base_frame", std::string("base_link"));
  only_prediction = this->declare_parameter("only_prediction", false);
  min_landmark_size = this->declare_parameter("min_landmark_size", 1);
  min_range = this->declare_parameter("min_range", 0.2);
  init_cov = this->declare_parameter("init_cov", 0.01);

  double sigma_x = this->declare_parameter("sigma_x", 0.15);
  double sigma_y = this->declare_parameter("sigma_y", 0.15);
  double sigma_theta = this->declare_parameter("sigma_theta", 0.15);
  double sigma_range = this->declare_parameter("sigma_range", 0.05);
  double sigma_bearing = this->declare_parameter("sigma_bearing", 0.03);
  double association_radius = this->declare_parameter("association_radius", 0.6);

  ekf.set_noise(sigma_x, sigma_y, sigma_theta, sigma_range, sigma_bearing);
  ekf.set_association_radius(association_radius);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  /* /posts lo publica el simulador una sola vez: hace falta transient_local para recibirlo */
  auto posts_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  posts_sub = this->create_subscription<geometry_msgs::msg::PoseArray>( "/posts", posts_qos, std::bind(&Localizer::on_posts, this, std::placeholders::_1));
  landmark_sub = this->create_subscription<robmovil_msgs::msg::LandmarkArray>( "/landmarks", rclcpp::QoS(10), std::bind(&Localizer::on_landmark_array, this, std::placeholders::_1));
  odo_sub = this->create_subscription<nav_msgs::msg::Odometry>( "/robot/odometry", rclcpp::QoS(10), std::bind(&Localizer::on_odometry, this, std::placeholders::_1));

  pose_pub = this->create_publisher<nav_msgs::msg::Odometry>("/robot/odometry_ekf", rclcpp::QoS(10));
}

/** El mapa de postes se conoce a priori en el frame map (consigna 4.a),
 *  en lugar de construirse con el primer sensado como en el taller */
void robmovil_ekf::Localizer::on_posts(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::vector<tf2::Vector3> map;
  for (const auto& p : msg->poses)
    map.emplace_back(p.position.x, p.position.y, 0);

  ekf.set_map(map);
  RCLCPP_INFO(this->get_logger(), "Mapa recibido: %zu postes en frame '%s'", map.size(), msg->header.frame_id.c_str());
}

bool robmovil_ekf::Localizer::init_from_tf()
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(map_frame_, odom_base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF %s -> %s no disponible para inicializar el EKF: %s",
        map_frame_.c_str(), odom_base_frame_.c_str(), ex.what());
    return false;
  }

  ekf.reset(tf.transform.translation.x, tf.transform.translation.y,
            tf2::getYaw(tf.transform.rotation), init_cov);

  RCLCPP_INFO(this->get_logger(), "EKF inicializado desde TF: x=%.3f y=%.3f theta=%.3f",
              tf.transform.translation.x, tf.transform.translation.y, tf2::getYaw(tf.transform.rotation));
  return true;
}

void robmovil_ekf::Localizer::prediction(const rclcpp::Time& now)
{
  /* avanzar el tiempo del filtro desde el tiempo anterior al actual */
  advance_time(now);

  /* hacer prediccion */
  ekf.timeUpdateStep(u);
}

/** La odometria es la entrada de control (consigna 4.b): u = (vx, vy, w) en el frame del robot.
 *  Se predice con cada mensaje usando su stamp, asi delta_t queda en tiempo simulado */
void robmovil_ekf::Localizer::on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  rclcpp::Time stamp(msg->header.stamp);

  if (!initialized)
  {
    initialized = init_from_tf();
    t = stamp;
    return;
  }

  if ((stamp - t).seconds() <= 1e-6)
    return;

  /* la velocidad medida en el intervalo [t, stamp] es la que trae este mensaje */
  u(1) = msg->twist.twist.linear.x;
  u(2) = msg->twist.twist.linear.y;
  u(3) = msg->twist.twist.angular.z;

  prediction(stamp);
  publish_estimate(stamp);
}

void robmovil_ekf::Localizer::on_landmark_array(const robmovil_msgs::msg::LandmarkArray::SharedPtr msg)
{
  if (!initialized || only_prediction || !ekf.has_map() || (int)msg->landmarks.size() < min_landmark_size)
    return;

  /* hacer update(s), secuencialmente por landmark */
  bool corrected = false;
  for (const auto& landmark : msg->landmarks)
  {
    /* muy cerca del robot el jacobiano H se vuelve singular */
    if (landmark.range < min_range)
      continue;

    LocalizerEKF::Vector z(2);
    z(1) = landmark.range;
    z(2) = landmark.bearing;

    if (ekf.set_measure(z))
    {
      ekf.measureUpdateStep(z);
      corrected = true;
    }
  }

  /* sin correcciones no se publica: el stamp del scan es anterior al de la ultima prediccion */
  if (corrected)
    publish_estimate(msg->header.stamp);
}

void robmovil_ekf::Localizer::advance_time(const rclcpp::Time& now)
{
  double delta_t = (now - t).seconds();
  t = now;

  if (delta_t < 0) RCLCPP_ERROR_STREAM(this->get_logger(), "Negative delta_t! " << delta_t);
  else ekf.set_delta_t(delta_t);
}

void robmovil_ekf::Localizer::publish_estimate(const rclcpp::Time& now)
{
  LocalizerEKF::Vector x = ekf.getX();
  LocalizerEKF::Matrix P = ekf.calculateP();

  tf2::Transform map_to_base_link;
  map_to_base_link.setOrigin(tf2::Vector3(x(1), x(2), 0));
  map_to_base_link.setRotation(tf2::Quaternion(tf2::Vector3(0,0,1),x(3)));

  nav_msgs::msg::Odometry pose_msg;
  pose_msg.header.stamp = now;
  pose_msg.header.frame_id = map_frame_;
  pose_msg.child_frame_id = base_frame_;
  tf2::toMsg(map_to_base_link, pose_msg.pose.pose);

  /* covarianza 6x6 [x, y, z, roll, pitch, yaw]: se completan las entradas de x, y, yaw */
  auto& cov = pose_msg.pose.covariance;
  cov.fill(0.0);
  const int idx[3] = {0, 1, 5};
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      cov[idx[i] * 6 + idx[j]] = P(i + 1, j + 1);

  pose_pub->publish(pose_msg);

  // publish TF transform between ekf frame and robot frame
  geometry_msgs::msg::TransformStamped map_to_base_link_msg;
  map_to_base_link_msg.header.frame_id = map_frame_;
  map_to_base_link_msg.header.stamp = now;
  map_to_base_link_msg.child_frame_id = base_frame_;
  map_to_base_link_msg.transform = tf2::toMsg(map_to_base_link);

  tf_broadcaster_->sendTransform(map_to_base_link_msg);
}
