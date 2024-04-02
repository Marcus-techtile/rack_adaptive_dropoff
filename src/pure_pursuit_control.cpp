#include "pure_pursuit_control.h"

PurePursuitController::PurePursuitController(){}

PurePursuitController::PurePursuitController(ros::NodeHandle &paramGet)
{
    /* Param get */
    paramGet.param<double>("docking_freq", freq_, 0.0);
    dt_ = 1/freq_;
    paramGet.param("/forklift_params/wheel_base", l_wheelbase_, 1.311);
    paramGet.param<double>("min_look_ahead_dis", min_look_ahead_dis_, 0.4);
    paramGet.param<double>("max_look_ahead_dis", max_look_ahead_dis_, 1.5);
    paramGet.param<double>("max_steering", max_steering_, 1.5);
    paramGet.param<double>("min_steering", min_steering_, -1.5);
    paramGet.param<double>("goal_correct_yaw", goal_correct_yaw_, 0.3);
    paramGet.param<double>("kp", kp_, 0.0);
    paramGet.param<double>("ki", ki_, 0.0);
    paramGet.param<double>("i_sw_offset", i_sw_offset_, 0.01);
    paramGet.param<double>("path_lateral_offset", path_lateral_offset_, 0.01);
    paramGet.param<bool>("use_track_path_pid", use_track_path_pid_, true);
    paramGet.param<bool>("use_point_interpolate", use_point_interpolate_, true);
    paramGet.param<bool>("use_ref_angle_from_path", use_ref_angle_from_path_, true);
    paramGet.param<bool>("re_cal_lookahead_dis", re_cal_lookahead_dis_, true);
}

void PurePursuitController::resetPP()
{
    i_part_ = 0;
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

void PurePursuitController::limitLookaheadDistance(double min_dis, double max_dis)
{
    this->min_look_ahead_dis_ = min_dis;
    this->max_look_ahead_dis_ = max_dis;
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
    // Lookahead Distance computation
    look_ahead_distance_ = calLookaheadDistance(lk_time, cur_vel_);

    // Get Lookahead Pose and Point
    pp_lookahead_pose_ = calLookaheadPoint(closest_index_, look_ahead_distance_, path_);
    point_lkh = pp_lookahead_pose_.pose.position;

    // Lookahead Curvature Computation
    look_ahead_curvature_ = calLookaheadCurvature(point_lkh);
    
    // Recalculate lookahead distance with actual point
    if (re_cal_lookahead_dis_ && min_look_ahead_dis_ < 0.35)
     look_ahead_distance_ = sqrt(point_lkh.x*point_lkh.x + point_lkh.y*point_lkh.y);

    // Calculate the reference lookahead angle
    if (use_ref_angle_from_path_) alpha_ = tf2::getYaw(path_.poses.at(point_index_).pose.orientation);
    else
    {
        distance_to_goal_ = hypot(path_.poses.at(path_.poses.size()-1).pose.position.x,
                                path_.poses.at(path_.poses.size()-1).pose.position.y);
        if (abs(distance_to_goal_) > goal_correct_yaw_) alpha_= atan(point_lkh.y/ point_lkh.x);
        else alpha_ = tf2::getYaw(path_.poses.at(point_index_).pose.orientation);
    }
    
    // Correct the lookahead pose angle
    pp_lookahead_pose_.pose.orientation = rpyToQuaternion(0, 0, alpha_);

    // Change sign angle if moving backward
    if (ref_vel_ < 0) alpha_ = -alpha_;  

    // Calculate the output steering
    PP_steering_angle_ = atan(2*l_wheelbase_*sin(alpha_)/look_ahead_distance_);

    // PID control for support the convergence to reference path
    lateral_heading_error_.data = point_lkh.y;

    if (use_track_path_pid_)
    {
        double path_lateral_tracking_error = path_.poses.at(closest_index_).pose.position.y;
        if (abs(path_lateral_tracking_error) > path_lateral_offset_) pid_error_ = path_lateral_tracking_error;
        else pid_error_ = lateral_heading_error_.data;
        // if (distance_to_goal_ < goal_correct_yaw_) pid_error_ = lateral_heading_error_.data;
    }
    else pid_error_ = lateral_heading_error_.data;
    if (pid_error_*pre_pid_error_ < 0 && abs(pid_error_ - pre_pid_error_) > i_sw_offset_) i_part_ = 0;

    p_part_ = kp_*pid_error_;
    i_part_ += ki_*pid_error_ * dt_;
    pre_pid_error_ = pid_error_;
    double steering_angle_corrected = PP_steering_angle_ + p_part_ + i_part_;

    steering_angle_ = steering_angle_corrected;

    if (steering_angle_ > max_steering_) steering_angle_ = max_steering_;
    if (steering_angle_ < min_steering_) steering_angle_ = min_steering_;
}