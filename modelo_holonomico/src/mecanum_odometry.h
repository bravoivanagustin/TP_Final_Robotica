#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robmovil_msgs/msg/multi_encoder_ticks.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace robmovil
{

class MecanumOdometry : public rclcpp::Node
{
  public:

    MecanumOdometry();

    void on_velocity_cmd(const geometry_msgs::msg::Twist::SharedPtr twist);

    void on_encoder_ticks(const robmovil_msgs::msg::MultiEncoderTicks::SharedPtr encoder);

  private:

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<robmovil_msgs::msg::MultiEncoderTicks>::SharedPtr encoder_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub_left_front_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub_right_front_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub_left_rear_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub_right_rear_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;

    double x_, y_, theta_;

    bool ticks_initialized_;
    int32_t last_ticks_left_front_, last_ticks_right_front_, last_ticks_left_rear_, last_ticks_right_rear_;
    rclcpp::Time last_ticks_time;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}
