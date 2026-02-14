#include "turtle_ekf/ekf_core.hpp"
#include <cmath>

namespace turtle_ekf
{

EKFCore::EKFCore()
    : state_(Eigen::VectorXd::Zero(STATE_SIZE)),
      P_(Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE)),
      Q_(Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE)),
      initialized_(false)
{
}

void EKFCore::initialize(const Eigen::VectorXd& initial_state,
                         const Eigen::MatrixXd& initial_covariance,
                         const Eigen::MatrixXd& process_noise)
{
    state_ = initial_state;
    P_ = initial_covariance;
    Q_ = process_noise;
    initialized_ = true;
}

void EKFCore::predict(double v, double omega, double dt)
{
    if (!initialized_ || dt <= 0.0) {
        return;
    }

    double theta = state_(THETA);
    double theta_new = normalizeAngle(theta + omega * dt);
    double cos_theta_new = std::cos(theta_new);
    double sin_theta_new = std::sin(theta_new);

    state_(X) += v * cos_theta_new * dt;
    state_(Y) += v * sin_theta_new * dt;
    state_(THETA) = theta_new;
    state_(VX) = v * cos_theta_new;
    state_(VY) = v * sin_theta_new;
    state_(OMEGA_Z) = omega;

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    F(X, THETA) = -v * std::sin(theta) * dt;
    F(Y, THETA) = v * std::cos(theta) * dt;
    F(VX, THETA) = -v * sin_theta_new;
    F(VY, THETA) = v * cos_theta_new;

    P_ = F * P_ * F.transpose() + Q_;
}

void EKFCore::correct(const Eigen::VectorXd& z,
                      const Eigen::MatrixXd& H,
                      const Eigen::MatrixXd& R)
{
    if (!initialized_) {
        return;
    }

    Eigen::VectorXd y = z - H * state_;

    for (int i = 0; i < H.rows(); ++i) {
        if (std::abs(H(i, THETA)) > 0.5) {
            y(i) = normalizeAngle(y(i));
        }
    }

    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    state_ = state_ + K * y;
    state_(THETA) = normalizeAngle(state_(THETA));

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    P_ = (I - K * H) * P_;
}

double EKFCore::normalizeAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

}
