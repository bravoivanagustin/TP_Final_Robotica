#include "mecanum_odometry.h"
#include <cmath>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

using namespace robmovil;

// Geometría del robot (Mecanum cuadrado, lx = ly = WHEEL_L).
// IDEA pendiente: levantar estas constantes a parámetros ROS 2 (declare_parameter)
// para tunearlas sin recompilar y centralizar la geometría del robot.
#define WHEEL_L       0.175    // (lx + ly) / 2; se asume lx = ly
#define WHEEL_RADIUS  0.05
#define ENCODER_TICKS 500.0

MecanumOdometry::MecanumOdometry()
  : Node("mecanum_odometry"),
    x_(0), y_(0), theta_(0),
    ticks_initialized_(false)
{
  // Consigna §1.a: el simulador publica /robot/cmd_vel.
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/robot/cmd_vel", rclcpp::QoS(10),
      std::bind(&MecanumOdometry::on_velocity_cmd, this, std::placeholders::_1));

  vel_pub_left_front_  = this->create_publisher<std_msgs::msg::Float64>("/robot/front_left_wheel/cmd_vel",  rclcpp::QoS(10));
  vel_pub_right_front_ = this->create_publisher<std_msgs::msg::Float64>("/robot/front_right_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_left_rear_   = this->create_publisher<std_msgs::msg::Float64>("/robot/rear_left_wheel/cmd_vel",   rclcpp::QoS(10));
  vel_pub_right_rear_  = this->create_publisher<std_msgs::msg::Float64>("/robot/rear_right_wheel/cmd_vel",  rclcpp::QoS(10));

  encoder_sub_ = this->create_subscription<robmovil_msgs::msg::MultiEncoderTicks>(
      "/robot/encoders", rclcpp::QoS(10),
      std::bind(&MecanumOdometry::on_encoder_ticks, this, std::placeholders::_1));

  pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot/odometry", rclcpp::QoS(10));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void MecanumOdometry::on_velocity_cmd(const geometry_msgs::msg::Twist::SharedPtr twist)
{
  // Cinemática inversa Mecanum (Taheri 2015, eq. 20). Índices 1=FL, 2=FR, 3=RL, 4=RR.
  // 2·WHEEL_L = lx + ly = 0.35 m (asume lx = ly).
  double v_x = twist->linear.x;
  double v_y = twist->linear.y;
  double w   = twist->angular.z;

  double vLeftFront  = (v_x - v_y - 2.0 * w * WHEEL_L) / WHEEL_RADIUS;
  double vRightFront = (v_x + v_y + 2.0 * w * WHEEL_L) / WHEEL_RADIUS;
  double vLeftRear   = (v_x + v_y - 2.0 * w * WHEEL_L) / WHEEL_RADIUS;
  double vRightRear  = (v_x - v_y + 2.0 * w * WHEEL_L) / WHEEL_RADIUS;

  std_msgs::msg::Float64 m;
  m.data = vLeftFront;  vel_pub_left_front_->publish(m);
  m.data = vRightFront; vel_pub_right_front_->publish(m);
  m.data = vLeftRear;   vel_pub_left_rear_->publish(m);
  m.data = vRightRear;  vel_pub_right_rear_->publish(m);
}

void MecanumOdometry::on_encoder_ticks(const robmovil_msgs::msg::MultiEncoderTicks::SharedPtr encoder)
{
  // Primer mensaje: solo guardar el baseline y salir sin integrar.
  if (!ticks_initialized_) {
    ticks_initialized_ = true;
    last_ticks_left_front_  = encoder->ticks[0];
    last_ticks_right_front_ = encoder->ticks[1];
    last_ticks_left_rear_   = encoder->ticks[2];
    last_ticks_right_rear_  = encoder->ticks[3];
    last_ticks_time = encoder->header.stamp;
    return;
  }

  int32_t d_ticks_fl = encoder->ticks[0] - last_ticks_left_front_;
  int32_t d_ticks_fr = encoder->ticks[1] - last_ticks_right_front_;
  int32_t d_ticks_rl = encoder->ticks[2] - last_ticks_left_rear_;
  int32_t d_ticks_rr = encoder->ticks[3] - last_ticks_right_rear_;

  rclcpp::Time current_time(encoder->header.stamp);
  double dt = (current_time - last_ticks_time).seconds();

  // Guard: stamps duplicados (común al arrancar la sim) → solo refrescar el baseline,
  // no integrar, para evitar división por cero y saltos numéricos.
  if (dt <= 1e-6) {
    last_ticks_left_front_  = encoder->ticks[0];
    last_ticks_right_front_ = encoder->ticks[1];
    last_ticks_left_rear_   = encoder->ticks[2];
    last_ticks_right_rear_  = encoder->ticks[3];
    last_ticks_time = current_time;
    return;
  }

  // Distancia lineal recorrida por cada rueda en el intervalo.
  double d1 = (d_ticks_fl / ENCODER_TICKS) * 2.0 * M_PI * WHEEL_RADIUS;  // FL
  double d2 = (d_ticks_fr / ENCODER_TICKS) * 2.0 * M_PI * WHEEL_RADIUS;  // FR
  double d3 = (d_ticks_rl / ENCODER_TICKS) * 2.0 * M_PI * WHEEL_RADIUS;  // RL
  double d4 = (d_ticks_rr / ENCODER_TICKS) * 2.0 * M_PI * WHEEL_RADIUS;  // RR

  // Cinemática directa Mecanum (Taheri 2015, eq. 21) — deltas en frame del cuerpo.
  // 8·WHEEL_L = 4·(lx + ly). La fila 2 (dyb) es la que captura el strafing,
  // y era la que el modelo diferencial anterior descartaba silenciosamente.
  double dxb = ( d1 + d2 + d3 + d4) / 4.0;
  double dyb = (-d1 + d2 + d3 - d4) / 4.0;
  double dth = (-d1 + d2 - d3 + d4) / (8.0 * WHEEL_L);

  // Integración a frame mundo (odom) rotando los deltas por theta actual.
  x_     += dxb * std::cos(theta_) - dyb * std::sin(theta_);
  y_     += dxb * std::sin(theta_) + dyb * std::cos(theta_);
  theta_ += dth;

  // Mensaje Odometry: pose en `odom`, twist en `base_link` (REP-105).
  nav_msgs::msg::Odometry msg;
  msg.header.stamp    = encoder->header.stamp;
  msg.header.frame_id = "odom";
  msg.child_frame_id  = "base_link";

  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);
  msg.pose.pose.orientation = tf2::toMsg(q);

  // Velocidades en frame del cuerpo (base_link).
  msg.twist.twist.linear.x  = dxb / dt;
  msg.twist.twist.linear.y  = dyb / dt;
  msg.twist.twist.linear.z  = 0;
  msg.twist.twist.angular.x = 0;
  msg.twist.twist.angular.y = 0;
  msg.twist.twist.angular.z = dth / dt;

  pub_odometry_->publish(msg);

  // TF odom → base_link, con el mismo stamp que el Odometry para evitar warnings
  // de extrapolación de tf2 bajo use_sim_time.
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp    = encoder->header.stamp;
  t.header.frame_id = "odom";
  t.child_frame_id  = "base_link";
  t.transform.translation.x = x_;
  t.transform.translation.y = y_;
  t.transform.translation.z = 0;
  t.transform.rotation      = msg.pose.pose.orientation;

  tf_broadcaster_->sendTransform(t);

  // Actualizar baseline.
  last_ticks_left_front_  = encoder->ticks[0];
  last_ticks_right_front_ = encoder->ticks[1];
  last_ticks_left_rear_   = encoder->ticks[2];
  last_ticks_right_rear_  = encoder->ticks[3];
  last_ticks_time = current_time;
}
