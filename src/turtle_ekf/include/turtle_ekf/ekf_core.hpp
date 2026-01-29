#ifndef TURTLE_EKF__EKF_CORE_HPP_
#define TURTLE_EKF__EKF_CORE_HPP_

#include <Eigen/Dense>
#include <vector>

namespace turtle_ekf
{

/**
 * @brief 3-state Extended Kalman Filter for differential drive robot localization.
 *
 * State: [x, y, theta] - position and orientation in world frame
 * Input: (v, omega) - linear and angular velocity from wheel odometry (measured, not estimated)
 * Measurement: theta from IMU orientation
 *
 * This design is optimal for differential drive robots because:
 * - Wheel encoders provide accurate v, omega directly (no need to estimate)
 * - IMU provides accurate theta (fused to correct drift)
 * - More states would add noise without improving accuracy
 */
class EKFCore
{
public:
    static constexpr int STATE_SIZE = 3;

    enum StateIndex
    {
        X = 0,
        Y = 1,
        THETA = 2
    };

    EKFCore();

    void initialize(const Eigen::VectorXd& initial_state,
                    const Eigen::MatrixXd& initial_covariance,
                    const Eigen::MatrixXd& process_noise);

    /**
     * @brief Predict state using measured velocities as control input.
     * @param v Linear velocity (m/s) from wheel odometry
     * @param omega Angular velocity (rad/s) from wheel odometry
     * @param dt Time step (seconds)
     */
    void predict(double v, double omega, double dt);

    void correct(const Eigen::VectorXd& z,
                 const Eigen::MatrixXd& H,
                 const Eigen::MatrixXd& R);

    Eigen::VectorXd getState() const { return state_; }
    Eigen::MatrixXd getCovariance() const { return P_; }

    void setState(const Eigen::VectorXd& state) { state_ = state; }
    void setProcessNoise(const Eigen::MatrixXd& Q) { Q_ = Q; }

private:
    double normalizeAngle(double angle);

    Eigen::VectorXd state_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    bool initialized_;
};

}

#endif
