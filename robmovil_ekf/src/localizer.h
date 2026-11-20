#ifndef __ROBMOVIL_EKF_LOCALIZER_H__
#define __ROBMOVIL_EKF_LOCALIZER_H__

#include <rclcpp/rclcpp.hpp>
#include <robmovil_msgs/msg/landmark_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>

#include "localizer_ekf.h"

namespace robmovil_ekf {
  class Localizer : public rclcpp::Node
  {
    public:
      Localizer();

      void on_landmark_array(const robmovil_msgs::msg::LandmarkArray::SharedPtr msg);
      void on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);
      void on_posts(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    private:
      LocalizerEKF ekf;
      bool initialized;
      bool only_prediction;
      int min_landmark_size;
      double min_range;
      double init_cov;

      rclcpp::Subscription<robmovil_msgs::msg::LandmarkArray>::SharedPtr landmark_sub;
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub;
      rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr posts_sub;
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub;
      std::string base_frame_, map_frame_, odom_base_frame_;

      std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
      std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

      /* Inicializa el estado con la TF map -> base_link (pose odometrica en el frame map) */
      bool init_from_tf();
      void advance_time(const rclcpp::Time& now);
      void publish_estimate(const rclcpp::Time& now);

      void prediction(const rclcpp::Time& time);

      /* ultimo comando de control recibido */
      LocalizerEKF::Vector u;
      rclcpp::Time t;
  };
}

#endif // __ROBMOVIL_EKF_LOCALIZER_H__
