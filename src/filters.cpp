#include "filters.h"

/******* Kalman filter ********* */
void KalmanFilter::predict() {
    // Predict the state: x' = F * x
    x_ = F_ * x_;

    // Predict the estimation covariance: P' = F * P * F' + Q
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::Vector3d &z) {
    // Compute Kalman gain: K = P * H' * (H * P * H' + R)^-1
    Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix3d K = P_ * H_.transpose() * S.inverse();

    // Update the state estimate: x = x' + K * (z - H * x')
    x_ = x_ + K * (z - H_ * x_);

    // Update the estimation covariance: P = (I - K * H) * P
    P_ = (Eigen::Matrix3d::Identity() - K * H_) * P_;
}

Eigen::Vector3d KalmanFilter::getState() {
    return x_;
}

/******* Low pass filter ********* */
void LowPassFilter::applyFilter(double &x, double &y, double &theta, double new_x, double new_y, double new_theta) 
{
    if (!initialized_) {
        // Initialize with the first measurement
        x_ = new_x;
        y_ = new_y;
        theta_ = new_theta;
        initialized_ = true;
    } else {
        // Apply lowpass filter
        x_ = alpha_ * new_x + (1.0 - alpha_) * x_;
        y_ = alpha_ * new_y + (1.0 - alpha_) * y_;
        theta_ = alpha_ * new_theta + (1.0 - alpha_) * theta_;
    }

    x = x_;
    y = y_;
    theta = theta_;
}

/****** EKF fusion *******/
EKF::EKF(int state_dim, int measurement_dim_1, int measurement_dim_2)
    : state_dim_(state_dim), measurement_dim_1_(measurement_dim_1), measurement_dim_2_(measurement_dim_2),
      F_(Eigen::MatrixXd::Identity(state_dim_, state_dim_)),
      Q_(Eigen::MatrixXd::Identity(state_dim_, state_dim_)),
      P_(Eigen::MatrixXd::Identity(state_dim_, state_dim_)),
      x_(Eigen::VectorXd::Zero(state_dim_)),
      H1_(Eigen::MatrixXd::Identity(measurement_dim_1_, state_dim_)),
      R1_(Eigen::MatrixXd::Identity(measurement_dim_1_, measurement_dim_1_)),
      H2_(Eigen::MatrixXd::Identity(measurement_dim_2_, state_dim_)),
      R2_(Eigen::MatrixXd::Identity(measurement_dim_2_, measurement_dim_2_)),
      I_(Eigen::MatrixXd::Identity(state_dim_, state_dim_)) 
{
    // Initialize measurement matrices 
    // H1_ << 1, 0, 0,
    //        0, 1, 0,
    //        0, 0, 1;

    // H2_ << 1, 0, 0,
    //        0, 1, 0,
    //        0, 0, 1;
}

void EKF::setCovariance(double predict_cov, double first_meas_cov, double second_meas_cov)
{
    Q_ = predict_cov * Q_;
    R1_ = first_meas_cov * R1_;
    R2_ = second_meas_cov * R2_;
}

// Predict step
void EKF::predict() {
    P_ = F_ * P_ * F_.transpose() + Q_;
}

// Update step with measurement from source 1
void EKF::updateWithMeasurement1(const Eigen::VectorXd &z1) {
    Eigen::VectorXd y1 = z1 - H1_ * x_;
    Eigen::MatrixXd S1 = H1_ * P_ * H1_.transpose() + R1_;
    Eigen::MatrixXd K1 = P_ * H1_.transpose() * S1.inverse();
    x_ = x_ + K1 * y1;
    P_ = (I_ - K1 * H1_) * P_;
}

// Update step with measurement from source 2
void EKF::updateWithMeasurement2(const Eigen::VectorXd &z2) {
    Eigen::VectorXd y2 = z2 - H2_ * x_;
    Eigen::MatrixXd S2 = H2_ * P_ * H2_.transpose() + R2_;
    Eigen::MatrixXd K2 = P_ * H2_.transpose() * S2.inverse();
    x_ = x_ + K2 * y2;
    P_ = (I_ - K2 * H2_) * P_;
}

// Get the current state estimate
Eigen::VectorXd EKF::getState() const {
    return x_;
}

// Get the current state covariance
Eigen::MatrixXd EKF::getCovariance() const {
    return P_;
}