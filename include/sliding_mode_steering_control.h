#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <utils.h>
#include <eigen3/Eigen/Core>

class SlidingModeSteeringControl
{
private:
    /* Forklift parameters */
    double wheel_base_;

    /* Robot feedback state*/
    double robot_linear_speed_, steering_wheel_angle_, wheel_speed_;

    nav_msgs::Path path_;

    /* Robot model matrix */
    Eigen::Vector2f A_;

    /* Sliding Surface */
    Eigen::Vector2f S_;
    Eigen::Vector2f sign_S_;

    /* Ref Path Derivative */
    Eigen::Vector2f Xr_dot_;
    double path_sample_time_;

    /* Control params */
    double min_steer_, max_steer_;
    double ld1_, k1_, k2_;
    int lookahead_point_;
    int closest_point_;

public:
    SlidingModeSteeringControl(double min_steer, double max_steer);
    ~SlidingModeSteeringControl();
    void setRobotWheelBase(double wb);
    void setRobotState(double lin_sp, double steer_angle, double wheel_sp);
    void setRefPath(nav_msgs::Path ref_path);
    void setControlParams(double ld, double k1, double k2);
    void setClosestPoint(int closest_point);
    void calControl();

    /* Control Output*/
    double u_;
    double steering_;
};