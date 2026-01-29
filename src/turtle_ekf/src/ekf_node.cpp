#include "turtle_ekf/ekf_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <cmath>

namespace turtle_ekf
{

EKFNode::EKFNode()
    : Node("turtle_ekf_node"),
      last_odom_time_(0, 0, RCL_ROS_TIME),
      initialized_(false),
      imu_theta_offset_(0.0),
      imu_offset_initialized_(false)
{
    loadParameters();
    initializeEKF();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&EKFNode::odomCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10,
        std::bind(&EKFNode::imuCallback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void EKFNode::loadParameters()
{
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_footprint");
    this->declare_parameter("odom_topic", "/wheel_odom");
    this->declare_parameter("imu_topic", "/imu");

    std::vector<double> default_process_noise = {0.001, 0.001, 0.01};
    this->declare_parameter("process_noise_covariance", default_process_noise);

    std::vector<double> default_initial_cov = {0.1, 0.1, 0.1};
    this->declare_parameter("initial_estimate_covariance", default_initial_cov);

    this->declare_parameter("imu_theta_covariance", 0.1);
    this->declare_parameter("use_imu_orientation", true);

    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    imu_topic_ = this->get_parameter("imu_topic").as_string();
    process_noise_ = this->get_parameter("process_noise_covariance").as_double_array();
    initial_covariance_ = this->get_parameter("initial_estimate_covariance").as_double_array();
    imu_theta_cov_ = this->get_parameter("imu_theta_covariance").as_double();
    use_imu_orientation_ = this->get_parameter("use_imu_orientation").as_bool();
}

void EKFNode::initializeEKF()
{
    Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(EKFCore::STATE_SIZE);

    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(EKFCore::STATE_SIZE, EKFCore::STATE_SIZE);
    for (int i = 0; i < EKFCore::STATE_SIZE && i < static_cast<int>(initial_covariance_.size()); ++i) {
        P0(i, i) = initial_covariance_[i];
    }

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(EKFCore::STATE_SIZE, EKFCore::STATE_SIZE);
    for (int i = 0; i < EKFCore::STATE_SIZE && i < static_cast<int>(process_noise_.size()); ++i) {
        Q(i, i) = process_noise_[i];
    }

    ekf_.initialize(initial_state, P0, Q);
}

void EKFNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(ekf_mutex_);

    double v = msg->twist.twist.linear.x;
    double omega = msg->twist.twist.angular.z;
    rclcpp::Time msg_time = msg->header.stamp;

    if (!initialized_) {
        last_odom_time_ = msg_time;
        initialized_ = true;
        return;
    }

    double dt = (msg_time - last_odom_time_).seconds();

    if (dt <= 0.0 || dt > 1.0) {
        last_odom_time_ = msg_time;
        return;
    }

    ekf_.predict(v, omega, dt);
    last_odom_time_ = msg_time;

    latest_v_ = v;
    latest_omega_ = omega;

    publishOdometry();
    broadcastTransform();
}

void EKFNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if (!use_imu_orientation_) {
        return;
    }

    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    double yaw = tf2::getYaw(q);

    if (!imu_offset_initialized_) {
        imu_theta_offset_ = yaw;
        imu_offset_initialized_ = true;
        return;
    }

    double imu_theta = yaw - imu_theta_offset_;
    imu_theta = std::atan2(std::sin(imu_theta), std::cos(imu_theta));

    std::lock_guard<std::mutex> lock(ekf_mutex_);

    if (!initialized_) {
        return;
    }

    Eigen::VectorXd z(1);
    z(0) = imu_theta;

    Eigen::MatrixXd H(1, EKFCore::STATE_SIZE);
    H.setZero();
    H(0, EKFCore::THETA) = 1.0;

    Eigen::MatrixXd R(1, 1);
    R(0, 0) = imu_theta_cov_;

    ekf_.correct(z, H, R);

    latest_imu_theta_ = imu_theta;
}

void EKFNode::publishOdometry()
{
    Eigen::VectorXd state = ekf_.getState();
    Eigen::MatrixXd P = ekf_.getCovariance();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = state(EKFCore::X);
    odom_msg.pose.pose.position.y = state(EKFCore::Y);
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, state(EKFCore::THETA));
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.pose.covariance[0] = P(EKFCore::X, EKFCore::X);
    odom_msg.pose.covariance[1] = P(EKFCore::X, EKFCore::Y);
    odom_msg.pose.covariance[5] = P(EKFCore::X, EKFCore::THETA);
    odom_msg.pose.covariance[6] = P(EKFCore::Y, EKFCore::X);
    odom_msg.pose.covariance[7] = P(EKFCore::Y, EKFCore::Y);
    odom_msg.pose.covariance[11] = P(EKFCore::Y, EKFCore::THETA);
    odom_msg.pose.covariance[30] = P(EKFCore::THETA, EKFCore::X);
    odom_msg.pose.covariance[31] = P(EKFCore::THETA, EKFCore::Y);
    odom_msg.pose.covariance[35] = P(EKFCore::THETA, EKFCore::THETA);

    odom_msg.twist.twist.linear.x = latest_v_;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = latest_omega_;

    odom_msg.twist.covariance[0] = 0.01;
    odom_msg.twist.covariance[7] = 0.01;
    odom_msg.twist.covariance[35] = 0.01;

    odom_pub_->publish(odom_msg);
}

void EKFNode::broadcastTransform()
{
    Eigen::VectorXd state = ekf_.getState();

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;

    transform.transform.translation.x = state(EKFCore::X);
    transform.transform.translation.y = state(EKFCore::Y);
    transform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, state(EKFCore::THETA));
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transform);
}

}
