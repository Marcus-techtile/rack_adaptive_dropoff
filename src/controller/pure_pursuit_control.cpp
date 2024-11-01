#include "controller/pure_pursuit_control.h"

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
    paramGet.param<double>("k_lat", k_lat_, 0.7);
    paramGet.param<double>("k_angle", k_angle_, 0.3);
    paramGet.param<bool>("use_point_interpolate", use_point_interpolate_, true);
    paramGet.param<bool>("use_ref_angle_from_path", use_ref_angle_from_path_, true);
    paramGet.param<bool>("re_cal_lookahead_dis", re_cal_lookahead_dis_, true);

    paramGet.param<bool>("cal_lookahead_using_heading_thres", cal_lookahead_using_heading_thres_, true);
    paramGet.param<double>("heading_thres", heading_thres_, 0.5);
}

void PurePursuitController::resetPP()
{
    path_avai_ = false;
    steering_angle_ = 0;
    alpha_ = 0;
    pre_pid_error_ = 0;
    pid_error_ = 0;
    p_part_ = 0;
    i_part_ = 0;
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

void PurePursuitController::setSpeed(geometry_msgs::Twist cur_speed)
{
    raw_cur_vel_ = cur_speed.linear.x;
    double cut_off = 0.1;
    double lpf_gain = 1 - exp(-0.025 * 2 * M_PI * cut_off);
    lpf_output_ += (raw_cur_vel_ - lpf_output_) * lpf_gain;
    cur_vel_ = abs(lpf_output_);
}

void PurePursuitController::setRefPath(nav_msgs::Path path)
{
    path_ = path;
    path_avai_ = true;
}

void PurePursuitController::setLocalGoalPose(geometry_msgs::PoseStamped local_goal)
{
    local_goal_ = local_goal;
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

void PurePursuitController::setRefAngleMode(bool use_angle_from_path)
{
    this->use_ref_angle_from_path_ = use_angle_from_path;
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
    if (lk_dis > max_look_ahead_dis_) return max_look_ahead_dis_;
    return lk_dis;
}

double PurePursuitController::calLookaheadDistanceUsingHeading()
{
    // Find the min and max lookahead index
    int min_index, max_index;
    geometry_msgs::PoseStamped min_path_pose = calLookaheadPoint(closest_index_, min_look_ahead_dis_, path_, min_index);
    geometry_msgs::PoseStamped max_path_pose = calLookaheadPoint(closest_index_, max_look_ahead_dis_, path_, max_index);
    // ROS_INFO("min_index: %d", min_index);
    // ROS_INFO("max_index: %d", max_index);
    int lookahead_index_tmp = min_index;
    for (int i = min_index; i <= max_index; i++)
    {
        double lk_heading_tmp = abs(tf2::getYaw(path_.poses.at(i).pose.orientation));
        if (lk_heading_tmp > heading_thres_)
        {
            lookahead_index_tmp = i;
            break;
        }
        if (i == max_index)
        {
            lookahead_index_tmp = i;
        }

    }
    double lk_dis_tmp = hypot(path_.poses.at(lookahead_index_tmp).pose.position.x, path_.poses.at(lookahead_index_tmp).pose.position.y);
    return lk_dis_tmp;
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

geometry_msgs::PoseStamped PurePursuitController::calLookaheadPoint(int nearest_index, double & lookahead_distance, nav_msgs::Path path, int &output_index)
{
    // Find the first point which has the distance longer than the lookahead distance
    auto point_it = std::find_if(path.poses.begin() + nearest_index, path.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_distance;});

    output_index = std::distance(path.poses.begin(), point_it);
    if (output_index > path.poses.size() - 1) output_index = path.poses.size() - 1;
    
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
    if (path_avai_)
    {
        if (cal_lookahead_using_heading_thres_)
            look_ahead_distance_ = calLookaheadDistanceUsingHeading();
        else look_ahead_distance_ = calLookaheadDistance(lk_time, cur_vel_);

        // Get Lookahead Pose and Point
        pp_lookahead_pose_ = calLookaheadPoint(closest_index_, look_ahead_distance_, path_, point_index_);
    }
    else //if not using path
        pp_lookahead_pose_ = local_goal_;

    
    point_lkh = pp_lookahead_pose_.pose.position;

    // Lookahead Curvature Computation
    look_ahead_curvature_ = calLookaheadCurvature(point_lkh);
        
    // Recalculate lookahead distance with actual point
    if (re_cal_lookahead_dis_)
     look_ahead_distance_ = sqrt(point_lkh.x*point_lkh.x + point_lkh.y*point_lkh.y);

    // Calculate the reference lookahead angle
    distance_to_goal_ = hypot(local_goal_.pose.position.x, local_goal_.pose.position.y);
    if (use_ref_angle_from_path_) alpha_ = tf2::getYaw(pp_lookahead_pose_.pose.orientation);
    else
    {
        if (abs(distance_to_goal_) > goal_correct_yaw_) alpha_= atan(point_lkh.y/ point_lkh.x);
        else alpha_ = tf2::getYaw(pp_lookahead_pose_.pose.orientation);
    }
    
    // Correct the lookahead pose angle
    pp_lookahead_pose_.pose.orientation = rpyToQuaternion(0, 0, alpha_);

    // Change sign angle if moving backward
    if (ref_vel_ < 0) alpha_ = -alpha_;  

    // Calculate the output steering
    PP_steering_angle_ = atan(2*l_wheelbase_*sin(alpha_)/look_ahead_distance_);

    // PID control for support the convergence to reference path
    lateral_heading_error_.data = point_lkh.y;
    pid_error_ = lateral_heading_error_.data;
    
    p_part_ = kp_*pid_error_;
    i_part_ += ki_*pid_error_ * dt_;
    pre_pid_error_ = pid_error_;
    double steering_angle_corrected = PP_steering_angle_;
    if (abs(distance_to_goal_) < goal_correct_yaw_) steering_angle_corrected = k_angle_*PP_steering_angle_ + k_lat_*(p_part_ + i_part_);

    // if (use_ref_angle_from_path_)
    //     steering_angle_ = steering_angle_corrected;
    // else steering_angle_ = PP_steering_angle_;

    steering_angle_ = steering_angle_corrected;

    if (steering_angle_ > max_steering_) steering_angle_ = max_steering_;
    if (steering_angle_ < min_steering_) steering_angle_ = min_steering_;
}
