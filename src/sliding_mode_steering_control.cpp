#include "sliding_mode_steering_control.h"

SlidingModeSteeringControl::SlidingModeSteeringControl(double min_steer, double max_steer)
{
    min_steer_ = min_steer; max_steer_ = max_steer;
}

SlidingModeSteeringControl::~SlidingModeSteeringControl()
{
}

void SlidingModeSteeringControl::setRobotWheelBase(double wb){wheel_base_ = wb;}

void SlidingModeSteeringControl::setRobotState(double lin_sp, double steer_angle, double wheel_sp)
{
    robot_linear_speed_ = lin_sp;
    steering_wheel_angle_ = steer_angle;
    wheel_speed_ = -wheel_sp;           //revert sign
}

void SlidingModeSteeringControl::setRefPath(nav_msgs::Path ref_path){path_ = ref_path;}

void SlidingModeSteeringControl::setControlParams(double ld1, double k1, double k2)
{
    ld1_ = ld1;     k1_ = k1;     k2_ = k2;
}

void SlidingModeSteeringControl::setClosestPoint(int closest_point) {closest_point_ = closest_point;}

void SlidingModeSteeringControl::calControl()
{
    if(path_.poses.size() == 0) return;

    /* Setup lookahead point */
    lookahead_point_ = closest_point_ + 5;
    // while (path_.poses.at(lookahead_point_).pose.position.x <= 0) lookahead_point_++;
    if (lookahead_point_ > path_.poses.size() - 1)
        lookahead_point_ = path_.poses.size() - 1;
    if (lookahead_point_ < 0) 
    {
        ROS_ERROR("Lookahead control point is negative !");
        return;
    }
    /* Calculate the derivative of the ref path */
    ros::param::get("/docking_planner/dt", path_sample_time_);      // get planner sampling time
    double roll_tm, pitch_tm, pre_yaw_ref, yaw_ref;
    if (lookahead_point_ == 0)
        Xr_dot_ << 0, 0;
    else 
    {
        quaternionToRPY(path_.poses.at(lookahead_point_ - 1).pose.orientation, roll_tm, pitch_tm, pre_yaw_ref);
        quaternionToRPY(path_.poses.at(lookahead_point_).pose.orientation, roll_tm, pitch_tm, yaw_ref);
        Xr_dot_ << (path_.poses.at(lookahead_point_).pose.position.y - path_.poses.at(lookahead_point_-1).pose.position.y)/path_sample_time_,
                   (yaw_ref - pre_yaw_ref)/path_sample_time_;
    }

    /* Matrix A */
    A_ << wheel_speed_, wheel_speed_/wheel_base_;
    ROS_INFO("A_ (x , y): %f, %f", A_(0), A_(1));
    ROS_INFO("Xr_dot_ (x , teta): %f, %f", Xr_dot_(0), Xr_dot_(1));

    /* Sliding surface */
    Eigen::Vector2f E_;     // Error vector
    // E = x - xr. Consider in the local frame -> x = 0 -> E = -xr
    E_ << -path_.poses.at(lookahead_point_).pose.position.y,
          -yaw_ref;
    S_ = ld1_ * E_;
    sign_S_ << std::copysign(1, S_(0)),
                std::copysign(1, S_(1));
    ROS_INFO("Error (x , teta): %f, %f", E_(0), E_(1));
    ROS_INFO("Sliding Surface (x , teta): %f, %f", S_(0), S_(1));
    ROS_INFO("Sign S (x , teta): %f, %f", sign_S_(0), sign_S_(1));

    Eigen::Vector2f A_T;
    A_T = A_.transpose();
    double A1 = A_T.dot(A_);
    if (wheel_speed_ < 0.001) 
    {
        steering_ = 0;
    }
    else
    {
        double A2 = 1/A1;
        ROS_INFO("A1: %f", A1);
        ROS_INFO("A2: %f", A2);

        u_ = (A2*A_T).dot((Xr_dot_ - k1_* S_ - k2_*sign_S_));
        ROS_INFO("u_: %f", u_);
        if (u_ > 1) u_ = 0.99;
        if (u_ < -1) u_ = -0.99;
        ROS_INFO("u_ correct: %f", u_);
        steering_ = asin(u_);
        if (steering_ > max_steer_) steering_ = max_steer_;
        if (steering_ < min_steer_) steering_ = min_steer_;
    }

    ROS_INFO("steering_: %f", steering_);
}