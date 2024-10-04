#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <Eigen/Dense>

/** kalman filter to reduce noises in the rack detection output  **/
class KalmanFilter 
{
public:
    KalmanFilter() {
        // State vector: [x, y, angle]
        x_.setZero(3);
        
        // State transition model (Identity for no control input)
        F_.setIdentity(3, 3);

        // Measurement matrix (directly measuring the state)
        H_.setIdentity(3, 3);

        // Process covariance matrix (tuned based on system dynamics)
        Q_.setIdentity(3, 3);
        Q_ *= 10;

        // Measurement covariance matrix (tuned based on sensor noise)
        R_.setIdentity(3, 3);
        R_ *= 0.1;

        // Initial estimation covariance matrix
        P_.setIdentity(3, 3);
    }

    void predict();

    void update(const Eigen::Vector3d &z);

    Eigen::Vector3d getState() ;

private:
    Eigen::Vector3d x_;        // State vector [x, y, angle]
    Eigen::Matrix3d F_;        // State transition model
    Eigen::Matrix3d H_;        // Measurement matrix
    Eigen::Matrix3d Q_;        // Process covariance matrix
    Eigen::Matrix3d R_;        // Measurement covariance matrix
    Eigen::Matrix3d P_;        // Estimation covariance matrix
};


class LowPassFilter {
public:
    LowPassFilter(double alpha) : alpha_(alpha), initialized_(false) {}
    // Apply the filter to new data
    void applyFilter(double &x, double &y, double &theta, double new_x, double new_y, double new_theta);

private:
    double alpha_;       // Smoothing factor
    double x_, y_, theta_;
    bool initialized_;
};

class EKF {
public:
    EKF(int state_dim, int measurement_dim_1, int measurement_dim_2);
    void setCovariance(double predict_cov, double first_meas_cov, double second_meas_cov);
    void predict();
    void updateWithMeasurement1(const Eigen::VectorXd &z1);
    void updateWithMeasurement2(const Eigen::VectorXd &z2);
    Eigen::VectorXd getState() const;
    Eigen::MatrixXd getCovariance() const;

private:
    int state_dim_;
    int measurement_dim_1_;
    int measurement_dim_2_;

    Eigen::MatrixXd F_;  // State transition model
    Eigen::MatrixXd Q_;  // Process noise covariance
    Eigen::MatrixXd P_;  // State covariance matrix
    Eigen::VectorXd x_;  // State vector

    Eigen::MatrixXd H1_; // Measurement matrix for source 1
    Eigen::MatrixXd R1_; // Measurement noise covariance for source 1

    Eigen::MatrixXd H2_; // Measurement matrix for source 2
    Eigen::MatrixXd R2_; // Measurement noise covariance for source 2

    Eigen::MatrixXd I_;  // Identity matrix for updates
};



