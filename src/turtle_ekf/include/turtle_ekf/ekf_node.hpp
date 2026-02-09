#ifndef TURTLE_EKF__EKF_NODE_HPP_
#define TURTLE_EKF__EKF_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "turtle_ekf/ekf_core.hpp"

#include <Eigen/Dense>
#include <memory>
#include <mutex>

namespace turtle_ekf
{

class EKFNode : public rclcpp::Node
{
public:
    EKFNode();

private:
    void loadParameters();
    void initializeEKF();
    void publishOdometry();
    void broadcastTransform();

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    EKFCore ekf_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::mutex ekf_mutex_;

    double latest_v_{0.0};
    double latest_omega_{0.0};
    double latest_imu_theta_{0.0};

    std::string odom_frame_;
    std::string base_frame_;
    std::string odom_topic_;
    std::string imu_topic_;

    std::vector<double> process_noise_;
    std::vector<double> initial_covariance_;
    double imu_theta_cov_;

    bool use_imu_orientation_;
    bool publish_tf_;

    rclcpp::Time last_odom_time_;
    bool initialized_;

    double imu_theta_offset_;
    bool imu_offset_initialized_;
};

}

#endif
