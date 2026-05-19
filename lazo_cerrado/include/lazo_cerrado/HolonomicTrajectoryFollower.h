#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robmovil_msgs/msg/trajectory.hpp>

namespace robmovil
{

// Clase base abstracta del controlador de trayectoria holonómico.
// Equivalente al `TrajectoryFollower` del taller diferencial, pero con 3 GDL
// de salida (vx, vy, ωz) y publicando en /robot/cmd_vel para que lo consuma
// el módulo 1 (mecanum_odometry).
//
// El timer arranca en el constructor (no espera a recibir trayectoria) para
// que las clases derivadas puedan funcionar en modos que no dependan de la
// trayectoria suscrita (p.ej. goal fijo).
class HolonomicTrajectoryFollower : public rclcpp::Node
{
public:
  HolonomicTrajectoryFollower();

  void stop_timer() { if (timer_) timer_->cancel(); }

protected:
  // Las clases derivadas la implementan. Devolver `false` al terminar la
  // trayectoria: la base cancela el timer y publica un Twist cero final.
  virtual bool control(const rclcpp::Time& t,
                       double& vx, double& vy, double& wz) = 0;

  const rclcpp::Time& getInitialTime() const { return t0_; }
  const robmovil_msgs::msg::Trajectory& getTrajectory() const { return current_trajectory_; }

  // Helper análogo al del diferencial: ubica el siguiente waypoint según
  // tiempo. No se usa en pursuit, queda disponible por simetría con la base
  // pedagógica.
  bool nextPointIndex(const rclcpp::Time& time, size_t& idx) const;

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<robmovil_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;

  rclcpp::Time t0_;
  robmovil_msgs::msg::Trajectory current_trajectory_;

  void handleNewTrajectory(const robmovil_msgs::msg::Trajectory& msg);
  void timerCallback();
};

}  // namespace robmovil
