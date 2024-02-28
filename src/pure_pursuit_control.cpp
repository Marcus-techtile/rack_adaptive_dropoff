#include "pure_pursuit_control.h"

PurePursuitController::PurePursuitController(){}

PurePursuitController::PurePursuitController(ros::NodeHandle &paramGet)
{
    /* Param get */
    paramGet.param("/forklift_params/wheel_base", l_wheelbase_, 1.311);
    paramGet.param<double>("look_ahead_time", look_ahead_time_, 0.8);
    paramGet.param<double>("look_ahead_reverse_time", look_ahead_reverse_time_, 1.2);
    paramGet.param<double>("min_look_ahead_dis", min_look_ahead_dis_, 0.4);
    paramGet.param<double>("max_look_ahead_dis", max_look_ahead_dis_, 1.5);

    paramGet.param<double>("kp", kp_, 0.0);
    paramGet.param<double>("ki", ki_, 0.0);

    paramGet.param<double>("max_steering", max_steering_, 1.5);
    paramGet.param<double>("min_steering", min_steering_, -1.5);

    paramGet.param<double>("goal_correct_yaw", goal_correct_yaw_, 0.3);
    paramGet.param<bool>("use_point_interpolate", use_point_interpolate_, true);

}

void PurePursuitController::resetPP()
{
    sum_e_la = 0;
    steering_angle_ = 0;
    alpha_ = 0;
}

void PurePursuitController::setOdom(nav_msgs::Odometry odom)
{
    odom_ = odom;
    raw_cur_vel_ = odom.twist.twist.linear.x;
    double cut_off = 0.1;
    double lpf_gain = 1 - exp(-0.025 * 2 * M_PI * cut_off);
    lpf_output_ += (raw_cur_vel_ - lpf_output_) * lpf_gain;
    cur_vel_ = abs(lpf_output_);
}

void PurePursuitController::setRefPath(nav_msgs::Path path)
{
    path_ = path;
}

double PurePursuitController::getSteeringAngle()
{
    return steering_angle_;
}

void PurePursuitController::setRefVel(double ref_vel)
{
    ref_vel_ = ref_vel;
    // cur_vel_ = ref_vel;
}

void PurePursuitController::setClosestPoint(int closest_point)
{
    closest_point_ = closest_point;
}

geometry_msgs::Point PurePursuitController::interpolateLkhPoint(const geometry_msgs::Point & p1,
                                                                     const geometry_msgs::Point & p2,
                                                                     double r)
{
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - D * D);
  p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

void PurePursuitController::setGoalCorrectYaw(double dis_cor)
{
    goal_correct_yaw_ = dis_cor;
}

void PurePursuitController::calControl()
{
    if(path_.poses.size() == 0) return;
    double distance_to_goal = sqrt(path_.poses.at(path_.poses.size()-1).pose.position.x*path_.poses.at(path_.poses.size()-1).pose.position.x
                            + path_.poses.at(path_.poses.size()-1).pose.position.y*path_.poses.at(path_.poses.size()-1).pose.position.y);
    double lk_time = look_ahead_time_;

    if (cur_vel_*ref_vel_ < 0) cur_vel_ = -cur_vel_;

    if (ref_vel_ < 0) lk_time = look_ahead_reverse_time_;
    // current velocity, can be tried with ref velocity from the fuzzy controller
    look_ahead_distance_ = cur_vel_*lk_time;
    if (abs(look_ahead_distance_) < min_look_ahead_dis_) look_ahead_distance_ = min_look_ahead_dis_;
    if (abs(look_ahead_distance_) > max_look_ahead_dis_) look_ahead_distance_ = max_look_ahead_dis_;

    // ROS_INFO("PP Look ahead distance: %f", look_ahead_distance_);

    // Find the first point which has the distance longer than the lookahead distance
    for (point_index_ = closest_point_; point_index_ < path_.poses.size(); point_index_++)
    {
        if (abs(look_ahead_distance_) <= sqrt(path_.poses.at(point_index_).pose.position.x*path_.poses.at(point_index_).pose.position.x
                                    + path_.poses.at(point_index_).pose.position.y*path_.poses.at(point_index_).pose.position.y)) break;  
    }
    // if (ref_vel_ < 0)
    // {
    //     if (point_index_ > max_lk_reverse_point_) point_index_ = max_lk_reverse_point_;
    // } 
    if (point_index_ >= path_.poses.size()) point_index_ = path_.poses.size() - 1;

    // Interpolate the point which has the distance = lookahead_distance
    if (use_point_interpolate_ && point_index_ > 0)
    {
        pre_point_index_ = point_index_ - 1;
        point0.x = path_.poses.at(pre_point_index_).pose.position.x;
        point0.y = path_.poses.at(pre_point_index_).pose.position.y;
        point1.x = path_.poses.at(point_index_).pose.position.x;
        point1.y = path_.poses.at(point_index_).pose.position.y;
        point_lkh = interpolateLkhPoint(point0, point1, look_ahead_distance_);
    }
    else
    {
        point_lkh.x = path_.poses.at(point_index_).pose.position.x;
        point_lkh.y = path_.poses.at(point_index_).pose.position.y;
    }

    // If close to the goal. Correct the lookahead point
    if (distance_to_goal <= goal_correct_yaw_)
    {
        point_index_ = path_.poses.size() - 1;
        point_lkh.x = path_.poses.at(point_index_).pose.position.x;
        point_lkh.y = path_.poses.at(point_index_).pose.position.y;
        // ROS_INFO("MAX POINT INDEX CORRECTED!!!");
    }   

    look_ahead_distance_ = sqrt(point_lkh.x*point_lkh.x + point_lkh.y*point_lkh.y);
    // if (abs(look_ahead_distance_) < min_look_ahead_dis_) look_ahead_distance_ = min_look_ahead_dis_;
    // if (abs(look_ahead_distance_) > max_look_ahead_dis_) look_ahead_distance_ = max_look_ahead_dis_;
        
    // correct lookahead distance when near goal
    // if (point_index_ == path_.poses.size() - 1) look_ahead_distance_ = sqrt(point_lkh.x*point_lkh.x + point_lkh.y*point_lkh.y);


    ROS_INFO("PP Look ahead point distance: %f", look_ahead_distance_);
    ROS_INFO("PP Look ahead point: %d", point_index_);
    // ROS_INFO("Max lkh reverse point: %d", max_lk_reverse_point_);
    
    // Calculate alpha and set the ending condition to avoid the singularity
    double roll_tmp, pitch_tmp;
    if (abs(point_lkh.x) > 0.01) alpha_= atan(point_lkh.y/ point_lkh.x);
    else quaternionToRPY(path_.poses.at(point_index_).pose.orientation, roll_tmp, pitch_tmp, alpha_);

    lateral_heading_error_.data = point_lkh.y;
    // lateral_heading_error_.data = path_.poses.at(closest_point_).pose.position.y;
    lateral_error_.data = path_.poses.at(closest_point_).pose.position.y;
    
    if (distance_to_goal <= goal_correct_yaw_)
    {
        quaternionToRPY(path_.poses.at(path_.poses.size()-1).pose.orientation, roll_tmp, pitch_tmp, alpha_);
        // look_ahead_distance_ = goal_correct_yaw_;
    }
        
    
    if (ref_vel_ < 0) alpha_ = -alpha_;  

    ROS_INFO("PP alpha: %f", alpha_);
    steering_angle_ = atan(2*l_wheelbase_*sin(alpha_)/look_ahead_distance_);

    ROS_INFO("Steering_angle: %f", steering_angle_);
    sum_e_la += lateral_heading_error_.data * 0.02;

    double steering_angle_corrected = steering_angle_ + kp_*lateral_heading_error_.data 
                                            + ki_*sum_e_la;
    ROS_INFO("Corrected steering_angle: %f", steering_angle_corrected);
    ROS_INFO("LATERAL ERROR: %f", lateral_heading_error_.data);

    steering_angle_ = steering_angle_corrected;

    if (steering_angle_ > max_steering_) steering_angle_ = max_steering_;
    if (steering_angle_ < min_steering_) steering_angle_ = min_steering_;

}