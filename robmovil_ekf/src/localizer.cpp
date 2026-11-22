#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include "localizer.h"

robmovil_ekf::Localizer::Localizer() :
  Node("localizer_node"), initialized(false), u(3)
{
  u(1) = u(2) = u(3) = 0;

  // Get node parameters
  this->declare_parameter("base_link_frame", std::string("base_link_ekf"));
  this->declare_parameter("ekf_frame", std::string("map"));
  this->declare_parameter("odom_base_frame", std::string("base_link"));
  this->declare_parameter("only_prediction", false);
  this->declare_parameter("min_landmark_size", 1);
  this->declare_parameter("min_range", 0.2);
  this->declare_parameter("init_cov", 0.01);
  this->declare_parameter("sigma_x", 0.02);
  this->declare_parameter("sigma_y", 0.02);
  this->declare_parameter("sigma_theta", 0.01);
  this->declare_parameter("sigma_range", 0.01);
  this->declare_parameter("sigma_bearing", 0.005);
  this->declare_parameter("association_radius", 0.6);

  double sigma_x, sigma_y, sigma_theta, sigma_range, sigma_bearing, association_radius;

  this->get_parameter("base_link_frame", base_frame_);
  this->get_parameter("ekf_frame", map_frame_);
  this->get_parameter("odom_base_frame", odom_base_frame_);
  this->get_parameter("only_prediction", only_prediction);
  this->get_parameter("min_landmark_size", min_landmark_size);
  this->get_parameter("min_range", min_range);
  this->get_parameter("init_cov", init_cov);
  this->get_parameter("sigma_x", sigma_x);
  this->get_parameter("sigma_y", sigma_y);
  this->get_parameter("sigma_theta", sigma_theta);
  this->get_parameter("sigma_range", sigma_range);
  this->get_parameter("sigma_bearing", sigma_bearing);
  this->get_parameter("association_radius", association_radius);

  ekf.set_noise(sigma_x, sigma_y, sigma_theta, sigma_range, sigma_bearing);
  ekf.set_association_radius(association_radius);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  /* el sim publica /posts una sola vez al dar play y como volatile: levantar el nodo antes del play */
  posts_sub = this->create_subscription<geometry_msgs::msg::PoseArray>( "/posts", rclcpp::QoS(1).reliable().durability_volatile(), std::bind(&Localizer::on_posts, this, std::placeholders::_1));
  landmark_sub = this->create_subscription<robmovil_msgs::msg::LandmarkArray>( "/landmarks", rclcpp::QoS(10), std::bind(&Localizer::on_landmark_array, this, std::placeholders::_1));
  odo_sub = this->create_subscription<nav_msgs::msg::Odometry>( "/robot/odometry", rclcpp::QoS(10), std::bind(&Localizer::on_odometry, this, std::placeholders::_1));

  pose_pub = this->create_publisher<nav_msgs::msg::Odometry>("/robot/odometry_ekf", rclcpp::QoS(10));
}

/** El mapa de postes se conoce a priori en el frame map (consigna 4.a),
 *  en lugar de construirse con el primer sensado como en el taller */
void robmovil_ekf::Localizer::on_posts(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::vector<tf2::Vector3> map;
  for (int i = 0; i < msg->poses.size(); i++)
  {
    tf2::Vector3 landmark(msg->poses[i].position.x, msg->poses[i].position.y, 0);
    map.push_back(landmark);

    RCLCPP_INFO(this->get_logger(), "Landmark: %f, %f, %f", landmark.getX(), landmark.getY(), landmark.getZ());
  }

  ekf.set_map(map);
  RCLCPP_INFO(this->get_logger(), "Mapa recibido: %zu postes en frame %s", map.size(), msg->header.frame_id.c_str());
}

/** Inicializa el estado con la TF map -> base_link, es decir, con la pose odometrica en el frame map */
bool robmovil_ekf::Localizer::init_from_tf()
{
  geometry_msgs::msg::TransformStamped transformStamped;
  try
  {
    transformStamped = tf_buffer_->lookupTransform(map_frame_, odom_base_frame_, tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "No se pudo transformar %s a %s: %s",
              map_frame_.c_str(), odom_base_frame_.c_str(), ex.what());
    return false;
  }

  double x = transformStamped.transform.translation.x;
  double y = transformStamped.transform.translation.y;
  double theta = tf2::getYaw(transformStamped.transform.rotation);

  ekf.reset(x, y, theta, init_cov);

  RCLCPP_INFO(this->get_logger(), "Initial state x0: ( %f, %f, %f )", x, y, theta);
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
  rclcpp::Time stamp = msg->header.stamp;

  if (!initialized)
  {
    initialized = init_from_tf();
    t = stamp;
    return;
  }

  /* stamps repetidos (comun al arrancar la simulacion) */
  if ((stamp - t).seconds() <= 1e-6)
    return;

  /* la velocidad medida en el intervalo [t, stamp] es la que trae este mensaje */
  u(1) = msg->twist.twist.linear.x;
  u(2) = msg->twist.twist.linear.y;
  u(3) = msg->twist.twist.angular.z;

  prediction(stamp);

  /* publicar estimado actual */
  publish_estimate(stamp);
}

void robmovil_ekf::Localizer::on_landmark_array(const robmovil_msgs::msg::LandmarkArray::SharedPtr msg)
{
  if (!initialized || only_prediction || !ekf.has_map() || msg->landmarks.size() < min_landmark_size)
    return;

  /* hacer update(s), secuencialmente por landmark */
  bool corrected = false;
  for (int i = 0; i < msg->landmarks.size(); i++)
  {
    /* muy cerca del robot el jacobiano H se vuelve singular */
    if (msg->landmarks[i].range < min_range)
      continue;

    LocalizerEKF::Vector z(2);

    z(1) = msg->landmarks[i].range;
    z(2) = msg->landmarks[i].bearing;

    bool found_correspondence = ekf.set_measure(z);

    if (found_correspondence)
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

  nav_msgs::msg::Odometry pose_msg;
  pose_msg.header.stamp = now;
  pose_msg.header.frame_id = map_frame_;
  pose_msg.child_frame_id = base_frame_;

  /* build covariance */
  pose_msg.pose.covariance.fill(0.0);

  /* XY/XY covariance */
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 2; j++)
      pose_msg.pose.covariance[i * 6 + j] = P(i + 1, j + 1);

  /* XY/theta covariance */
  pose_msg.pose.covariance[0 * 6 + 5] = P(1,3);
  pose_msg.pose.covariance[1 * 6 + 5] = P(2,3);

  /* theta/XY covariance */
  pose_msg.pose.covariance[5 * 6 + 0] = P(3,1);
  pose_msg.pose.covariance[5 * 6 + 1] = P(3,2);

  /* theta/theta covariance */
  pose_msg.pose.covariance[5 * 6 + 5] = P(3,3);

  /* set pose */
  tf2::Transform map_to_base_link;
  map_to_base_link.setOrigin(tf2::Vector3(x(1), x(2), 0));
  map_to_base_link.setRotation(tf2::Quaternion(tf2::Vector3(0,0,1),x(3)));
  tf2::toMsg(map_to_base_link, pose_msg.pose.pose);

  /* publish */
  pose_pub->publish( pose_msg );

  // publish TF transform between ekf frame and robot frame
  geometry_msgs::msg::TransformStamped map_to_base_link_msg;
  map_to_base_link_msg.header.frame_id = map_frame_;
  map_to_base_link_msg.header.stamp = now;
  map_to_base_link_msg.child_frame_id = base_frame_;

  map_to_base_link_msg.transform = tf2::toMsg( map_to_base_link );

  tf_broadcaster_->sendTransform( map_to_base_link_msg );
}
