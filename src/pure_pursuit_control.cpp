#include "pure_pursuit_control.h"

PurePursuitController::PurePursuitController(){}

PurePursuitController::PurePursuitController(ros::NodeHandle &paramGet)
{
    /* Param get */
    paramGet.param<double>("docking_freq", freq_, 0.0);
    dt_ = 1/freq_;
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
    paramGet.param<bool>("use_ref_angle_from_path", use_ref_angle_from_path_, true);
    paramGet.param<bool>("re_cal_lookahead_dis", re_cal_lookahead_dis_, true);
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
}

void PurePursuitController::setClosestPoint(int closest_index)
{
    closest_index_ = closest_index;
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

void PurePursuitController::setLookaheadTime(double lk_t)
{
    lk_time = lk_t;
}

double PurePursuitController::calLookaheadDistance(double lk_t, double cur_spd)
{
    double lk_dis = abs(cur_spd) * lk_time;     // adaptive lookahead distance
    if (lk_dis < min_look_ahead_dis_) return min_look_ahead_dis_;
    if (lk_dis > max_look_ahead_dis_) return min_look_ahead_dis_;
    return lk_dis;
}

double PurePursuitController::calLookaheadCurvature(geometry_msgs::Point lookahead_point)
{
    const double sq_dis = (lookahead_point.x * lookahead_point.x) +
                            (lookahead_point.y * lookahead_point.y);
    // Assume that the initial point = robot position
    // Curvature of circle (k = 1 / R)
    if (sq_dis > 0.01) return 2.0 * lookahead_point.y / sq_dis;
    else return 0.0;
    
}

geometry_msgs::PoseStamped PurePursuitController::calLookaheadPoint(int nearest_index, double & lookahead_distance, nav_msgs::Path path)
{
    // Find the first point which has the distance longer than the lookahead distance
    auto point_it = std::find_if(path.poses.begin() + nearest_index, path.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_distance;});

    this->point_index_ = std::distance(path.poses.begin(), point_it);
    if (this->point_index_ > path.poses.size() - 1) this->point_index_ = path.poses.size() - 1;
    
    // Return the last point if no it satisfied
    if (point_it == path.poses.end()) {point_it = std::prev(path.poses.end());}
    else if (this->use_point_interpolate_ && point_it != path.poses.begin()) 
    {
        auto prev_point_it = std::prev(point_it);
        auto point = interpolateLkhPoint(prev_point_it->pose.position,
                                        point_it->pose.position, lookahead_distance);
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = point_it->header.frame_id;
        pose.header.stamp = point_it->header.stamp;
        pose.pose.position = point;
        return pose;
    }
    return *point_it;
}


void PurePursuitController::calControl()
{
    if(path_.poses.size() == 0) return;
    double distance_to_goal = sqrt(path_.poses.at(path_.poses.size()-1).pose.position.x*path_.poses.at(path_.poses.size()-1).pose.position.x
                            + path_.poses.at(path_.poses.size()-1).pose.position.y*path_.poses.at(path_.poses.size()-1).pose.position.y);

    look_ahead_distance_ = calLookaheadDistance(lk_time, cur_vel_);

    pp_lookahead_pose_ = calLookaheadPoint(closest_index_, look_ahead_distance_, path_);
    point_lkh = pp_lookahead_pose_.pose.position;

    look_ahead_curvature_ = calLookaheadCurvature(point_lkh);
    
    if (re_cal_lookahead_dis_)
     look_ahead_distance_ = sqrt(point_lkh.x*point_lkh.x + point_lkh.y*point_lkh.y);

    // Calculate alpha and set the ending condition to avoid the singularity
    if (use_ref_angle_from_path_) alpha_ = tf2::getYaw(path_.poses.at(point_index_).pose.orientation);
    else
    {
        if (abs(distance_to_goal) > goal_correct_yaw_) alpha_= atan(point_lkh.y/ point_lkh.x);
        else alpha_ = tf2::getYaw(path_.poses.at(point_index_).pose.orientation);
    }
    
    // Visualize the lookahead pose
    pp_lookahead_pose_.pose.position.x = point_lkh.x;
    pp_lookahead_pose_.pose.position.y = point_lkh.y;
    pp_lookahead_pose_.pose.position.z = 0;
    pp_lookahead_pose_.pose.orientation = rpyToQuaternion(0, 0, alpha_);

    lateral_heading_error_.data = point_lkh.y;
    // lateral_heading_error_.data = path_.poses.at(closest_index_).pose.position.y;
    lateral_error_.data = path_.poses.at(closest_index_).pose.position.y;
    
    // if (distance_to_goal <= goal_correct_yaw_)
    // {
    //     quaternionToRPY(path_.poses.at(path_.poses.size()-1).pose.orientation, roll_tmp, pitch_tmp, alpha_);
    // }
        
    
    if (ref_vel_ < 0) alpha_ = -alpha_;  

    PP_steering_angle_ = atan(2*l_wheelbase_*sin(alpha_)/look_ahead_distance_);

    sum_e_la += lateral_heading_error_.data * 0.02;

    double steering_angle_corrected = PP_steering_angle_ + kp_*lateral_heading_error_.data 
                                            + ki_*sum_e_la;

    steering_angle_ = steering_angle_corrected;

    if (steering_angle_ > max_steering_) steering_angle_ = max_steering_;
    if (steering_angle_ < min_steering_) steering_angle_ = min_steering_;
}