#include "docking_control.h"

DockingControl::DockingControl(ros::NodeHandle &nh, tf2_ros::Buffer &tf, double sec): nh_(nh), tf_buffer_c(tf), tf_time_out_(sec)
{
    /* Get Param */
    nh_.param<double>("docking_freq", docking_freq_, 50.0);
    dt_ = 1 / docking_freq_;     

    nh_.param<bool>("use_cost_function", use_cost_function_, false);                  
    nh_.param<double>("predict_time", predict_time_, 2.0);
    ROS_INFO("Docking Control Frequency: %f", docking_freq_);
    ROS_INFO("Docking Control Sampling Time: %f", dt_);
    ROS_INFO("Docking Control Predict Time: %f", predict_time_);

    nh_.param("/forklift_params/wheel_base", l_wheelbase_, 1.311);
    
    nh_.param<bool>("publish_cmd", publish_cmd_, false);
    nh_.param<double>("look_ahead_time", pp_look_ahead_time_, 3.0);
    nh_.param<double>("max_steering", max_steering_, 1.5);
    nh_.param<double>("min_steering", min_steering_, -1.5);
    nh_.param<double>("max_steering_speed", max_steering_speed_, 0.2);
    nh_.param<double>("min_steering_speed", min_steering_speed_, -0.2);
    nh_.param<double>("max_vel", max_vel_, 0.3);
    nh_.param<double>("min_vel", min_vel_, 0.15);
    nh_.param<double>("goal_correct_yaw", goal_correct_yaw_, 0.3);
    nh_.param<double>("max_angular_vel", max_angular_vel_, 0.2);
    nh_.param<bool>("adaptive_ref_angle", adaptive_ref_angle_, true);

    nh_.param<double>("fuzzy_lookahead_dis", fuzzy_lookahead_dis_, 0.3);
    nh_.param<bool>("limit_sp_curve", limit_sp_curve_, true);
    nh_.param<double>("backward_offset", backward_offset_, -0.02);
    nh_.param<double>("max_linear_acc", max_linear_acc_, 0.5);
    nh_.param<double>("min_linear_acc", min_linear_acc_, -0.5);

    nh_.param<double>("max_pocket_dock_vel", max_pocket_dock_vel_, 0.2);
    nh_.param<double>("max_pocket_dock_steering", max_pocket_dock_steering_, 0.2);

    nh_.param<double>("gain_track", gain_track_, 2.0);
    nh_.param<double>("gain_vel", gain_vel_, 1.0);
    nh_.param<double>("gain_heading", gain_heading_, 2.0);
    nh_.param<double>("gain_penetrate_linear_vel", gain_penetrate_linear_vel_, 2.0);

    nh_.param<double>("velocity_penalty_dis_thres", velocity_penalty_dis_thres_, 0.2);

    /* ROS Publisher */
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_cmd_vel_not_scale = nh_.advertise<geometry_msgs::Twist>("/pallet_docking/cmd_vel_unscaled", 1);
    pub_local_path_ = nh_.advertise<nav_msgs::Path>("/pallet_docking/local_ref_path", 1);
    pub_docking_local_path_ = nh_.advertise<nav_msgs::Path>("/pallet_docking/docking_control_path", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/pallet_docking/marker", 1);
    pub_pp_lookahead_distance_ = nh_.advertise<std_msgs::Float32>("/pallet_docking/purepursuit_lookahead_distance", 1);
    pub_pp_lookahead_angle_ = nh_.advertise<std_msgs::Float32>("/pallet_docking/purepursuit_lookahead_angle", 1);
    pub_pp_lookahead_curvature_ = nh_.advertise<std_msgs::Float32>("/pallet_docking/purepursuit_lookahead_curvature", 1);
    pub_pp_steering_ = nh_.advertise<std_msgs::Float32>("/pallet_docking/pp_steering_angle", 1);
    pub_pp_lookahead_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/pallet_docking/pp_lookahead_pose", 1);
    pub_nearest_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/pallet_docking/nearest_pose", 1);

    pub_boundary_point_ = nh.advertise<geometry_msgs::PoseArray>("/pallet_docking/boundary_points", 1);
    pub_footprint_ = nh.advertise<geometry_msgs::PolygonStamped>("/pallet_docking/footprint", 1);
    pub_estimated_footprint_ = nh.advertise<geometry_msgs::PolygonStamped>("/pallet_docking/estimated_footprint", 1);

    /* Define the Controller */
    fuzzy_controller = FuzzyControl(nh_);
    pure_pursuit_control = PurePursuitController(nh_);

    /* Initialize parameters */
    resetController();
}

DockingControl::~DockingControl(){}

void DockingControl::resetController()
{
    invalid_control_signal_ = false;
    controller_on_.data = false;
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.angular.z = 0.0;
    if(publish_cmd_) pub_cmd_vel_.publish(cmd_vel_);
    pure_pursuit_control.resetPP();
    ref_path_.poses.clear();
    ref_path_avai_ = false;
    steering_ = 0;
    abs_ref_vel_ = 0;
    final_ref_vel_ = 0;
}

void DockingControl::setRefPath(nav_msgs::Path path)
{
    ref_path_ = path;
    if (ref_path_.poses.size() > 1) ref_path_avai_ = true;
    else ref_path_avai_ = false;
}

void DockingControl::setLocalGoal(geometry_msgs::PoseStamped local_goal)
{
    local_goal_ = local_goal;
}

void DockingControl::setVel(geometry_msgs::Twist robot_speed)
{
    robot_speed_ = robot_speed;
}

/***** Check Required Data *****/
bool DockingControl::checkData()
{
    if (!ref_path_avai_)
    {
        ROS_DEBUG("No ref path!!!");
        return false;
    }
    if (ref_path_.poses.size() < 1)
    {
        ROS_DEBUG("Path is not feasible for the controller");
        return false;
    }
    return true;
}

nav_msgs::Path DockingControl::convertPathtoLocalFrame(nav_msgs::Path global_path)
{
    nav_msgs::Path local_ref_path;
    local_ref_path.header.frame_id = this->path_frame_;
    local_ref_path.header.stamp = ros::Time(0);
    local_ref_path.poses.clear();
    for (int i = 0; i < ref_path_.poses.size(); i++)
    {
        global_path.poses.at(i).header.stamp = ros::Time(0);
        try
        {
            local_ref_path.poses.push_back(tf_buffer_c.transform(global_path.poses.at(i), path_frame_, ros::Duration(tf_time_out_)));
        }
        catch (tf2::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }
    }
    int final_index = local_ref_path.poses.size() - 1;
    local_ref_path.poses.at(final_index) = local_goal_;
    return local_ref_path;
}

int DockingControl::nearestPointIndexFind(nav_msgs::Path local_path)
{
    int closest_index = 0;
    double min_dist = hypot(local_path.poses.at(closest_index).pose.position.x,
                            local_path.poses.at(closest_index).pose.position.y);
    for (int i = 2; i < local_path.poses.size(); i++)
    {
        double dist = hypot(local_path.poses.at(i).pose.position.x, local_path.poses.at(i).pose.position.y);
        if (dist < min_dist)
        {
            min_dist = dist;
            closest_index = i;
        }
    }
    if (local_path.poses.size() - 1 < closest_index) closest_index = local_path.poses.size() - 1;
    geometry_msgs::PoseStamped nearest_pose;
    nearest_pose = local_path.poses.at(closest_index);
    pub_nearest_pose_.publish(nearest_pose);
    return closest_index;
}

void DockingControl::steeringControl()
{
    pure_pursuit_control.setSpeed(robot_speed_);
    pure_pursuit_control.setRefPath(local_ref_path_);
    pure_pursuit_control.setLocalGoalPose(local_goal_);
    pure_pursuit_control.setRefVel(final_ref_vel_);
    pure_pursuit_control.setClosestPoint(nearest_index_);
    pure_pursuit_control.setLookaheadTime(pp_look_ahead_time_);
    if (!approaching_done_.data)
    { 
        if (adaptive_ref_angle_) pure_pursuit_control.setRefAngleMode(false);
    } 
    else
    {
        if (adaptive_ref_angle_) pure_pursuit_control.setRefAngleMode(true);
    } 
    pure_pursuit_control.calControl();

    steering_angle_ = pure_pursuit_control.steering_angle_;

    // Smooth the steering output. Limit steering speed
    double steering_speed = (steering_angle_ - steering_)/dt_;
    if (steering_speed > max_steering_speed_) steering_speed = max_steering_speed_;
    if (steering_speed < min_steering_speed_) steering_speed = min_steering_speed_;
    steering_ = steering_ + steering_speed * dt_;
    
    // Visualize and debugging topic
    std_msgs::Float32 pp_steer;
    pp_steer.data = pure_pursuit_control.PP_steering_angle_;
    pub_pp_steering_.publish(pp_steer);     // pp steering angle before being added PID

    std_msgs::Float32 pp_lkh_distance, pp_lkh_angle, pp_lkh_curve;
    pp_lkh_distance.data = pure_pursuit_control.look_ahead_distance_;
    pub_pp_lookahead_distance_.publish(pp_lkh_distance);

    pp_lkh_angle.data = pure_pursuit_control.alpha_;
    pub_pp_lookahead_angle_.publish(pp_lkh_angle);

    pp_lkh_curve.data = pure_pursuit_control.look_ahead_curvature_;
    pub_pp_lookahead_curvature_.publish(pp_lkh_curve);

    pp_lkh_pose_ = pure_pursuit_control.pp_lookahead_pose_;
    pp_lkh_pose_.header.frame_id = path_frame_;
    pp_lkh_pose_.header.stamp = ros::Time::now();
    pub_pp_lookahead_pose_.publish(pp_lkh_pose_);
}

void DockingControl::linearSpeedControl()
{
    double fuzzy_lk_dis = fuzzy_lookahead_dis_;
    double max_dist = sqrt(local_ref_path_.poses.at(local_ref_path_.poses.size()-1).pose.position.x*local_ref_path_.poses.at(local_ref_path_.poses.size()-1).pose.position.x
                        + local_ref_path_.poses.at(local_ref_path_.poses.size()-1).pose.position.y*local_ref_path_.poses.at(local_ref_path_.poses.size()-1).pose.position.y);
    
    if (fuzzy_lk_dis > max_dist) fuzzy_lk_dis = max_dist;
    for (lk_index_vel_ = nearest_index_; lk_index_vel_ < local_ref_path_.poses.size(); lk_index_vel_++)
    {
        if (abs(fuzzy_lk_dis) <= sqrt(local_ref_path_.poses.at(lk_index_vel_).pose.position.x*local_ref_path_.poses.at(lk_index_vel_).pose.position.x
                                    + local_ref_path_.poses.at(lk_index_vel_).pose.position.y*local_ref_path_.poses.at(lk_index_vel_).pose.position.y)) break;  
    }
    
    if (abs(local_ref_path_.poses.at(local_ref_path_.poses.size()-1).pose.position.x) < fuzzy_lk_dis)
        lk_index_vel_ = local_ref_path_.poses.size()-1;
    
    ROS_DEBUG("Fuzzy lk_index_vel_: %d", lk_index_vel_);

    // fuzzy_lk_dis = pure_pursuit_control.look_ahead_distance_;
    fuzzy_controller.inputSolveGoal(abs(fuzzy_lk_dis));
    if (limit_sp_curve_)
        fuzzy_controller.inputsolveSteering(pure_pursuit_control.look_ahead_curvature_);
    else fuzzy_controller.inputsolveSteering(0.0);
    fuzzy_controller.inputResults();
    
    if (limit_sp_curve_)
        ref_velocity_ = fuzzy_controller.cal_fuzzy_output();
    else
        ref_velocity_ = fuzzy_controller.cal_fuzzy_output() * cos(steering_);
    
    ROS_DEBUG("Linear speed from Fuzzy: %f", ref_velocity_);
    // Smooth the velocity output. Limit linear acceleration
    double linear_acc = (ref_velocity_ - abs_ref_vel_)/dt_;
    if (linear_acc > max_linear_acc_) linear_acc = max_linear_acc_;
    if (linear_acc < min_linear_acc_) linear_acc = min_linear_acc_;
    abs_ref_vel_ = abs_ref_vel_ + linear_acc * dt_;

    // if (abs_ref_vel_ > max_vel_) abs_ref_vel_ = max_vel_;
    // if (abs_ref_vel_ < min_vel_) abs_ref_vel_ = min_vel_;

    final_ref_vel_ = abs_ref_vel_;
    ROS_DEBUG("linear_acc: %f", linear_acc);
    ROS_DEBUG("abs_ref_vel_: %f", abs_ref_vel_);
    ROS_DEBUG("Linear speed limit acc: %f", final_ref_vel_);
    if (local_ref_path_.poses.at(lk_index_vel_).pose.position.x < backward_offset_)
        if (final_ref_vel_ > 0) final_ref_vel_ = -final_ref_vel_;
    
    ROS_DEBUG("Linear speed limit acc: %f", final_ref_vel_);
}

void DockingControl::limitControlSignal()
{
    /************ ANGULAR VELOCITY LIMIT ************/
    if (abs(final_ref_vel_) < min_vel_) final_ref_vel_ = std::copysign(min_vel_, final_ref_vel_);

    if (steering_ != 0)
    {
        double max_vel_limit = abs(max_angular_vel_ * l_wheelbase_ / tan(steering_));
        if (abs(final_ref_vel_) >= max_vel_limit) final_ref_vel_ = max_vel_limit * (final_ref_vel_/abs(final_ref_vel_));
    }

    /*********** LIMIT CONTROL SIGNAL WHEN DOCKING TO POCKET FOR SAFETY ************/
    if ((approaching_done_.data && final_ref_vel_ >= 0) ||
        (!approaching_done_.data && final_ref_vel_ < 0)) 
    {
        if (abs(steering_) >= max_pocket_dock_steering_) steering_ = max_pocket_dock_steering_*(abs(steering_)/steering_);
        if (abs(final_ref_vel_) >= max_pocket_dock_vel_) final_ref_vel_ = max_pocket_dock_vel_ * (final_ref_vel_/abs(final_ref_vel_));
    }  

    if (final_ref_vel_ > max_vel_) final_ref_vel_ = max_vel_;
    if (final_ref_vel_ < min_vel_) final_ref_vel_ = min_vel_;
    ROS_DEBUG("Linear speed after limit: %f", final_ref_vel_); 
}

void DockingControl::controllerCal()
{
    std::lock_guard<std::mutex> lock_controller_exec(mutex_);
    /********* Check controller is turned on/off **********/
    if (!controller_on_.data) return;

    /********* Check input data ***********/
    if (!checkData()) return;

    /*********** CONVERT GLOBAL PATH TO BASE_LINK PATH ***********/ 
    local_ref_path_ = convertPathtoLocalFrame(ref_path_);
    pub_local_path_.publish(local_ref_path_);

    /*********** NEAREST POINT FINDING **********/
    nearest_index_ = nearestPointIndexFind(local_ref_path_);
    ROS_DEBUG ("Nearest index: %d", nearest_index_);

    steeringControl();
    linearSpeedControl();
    limitControlSignal();

    /*********** NAN OUTPUT HANDLE *************/
    if(isnan(steering_) || isnan(final_ref_vel_))
    {
        invalid_control_signal_ = true;
        ROS_ERROR("NAN NUMBER ! STOP THE CONTROLLER");
        controller_on_.data = false;
        return;
    }

    cmd_vel_.linear.x = final_ref_vel_; 
    cmd_vel_.angular.z = steering_;

    ROS_DEBUG("Control CMD_VEL (v,w): %f, %f", cmd_vel_.linear.x,
                                        cmd_vel_.angular.z);

    // if (!use_cost_function_) {
    //     if (publish_cmd_) pub_cmd_vel_.publish(cmd_vel_);
    //     predict_path_ = predictPath(cmd_vel_);
    //     pub_docking_local_path_.publish(predict_path_);
    // }
    // else
    // {
        if (publish_cmd_) pub_cmd_vel_not_scale.publish(cmd_vel_);
        std::vector<geometry_msgs::Twist> control_sampling = generateControlSample(pre_cmd_vel_, cmd_vel_);
        geometry_msgs::Twist cmd_out =  evaluateControlSampleAndOutputControl(control_sampling);
        
        if (publish_cmd_) pub_cmd_vel_.publish(cmd_out);

        pre_cmd_vel_ = cmd_out;
    // }


}

/************************ Boundary generation ********************/
geometry_msgs::PoseArray DockingControl::generateBoundaryPoints(const geometry_msgs::PoseStamped& goal_pose, double boundary_distance, double step_size)
{
    geometry_msgs::PoseArray boundary_points;

    boundary_points.header.frame_id = path_frame_;  
    boundary_points.header.stamp = ros::Time::now();

    double x_goal = goal_pose.pose.position.x;
    double y_goal = goal_pose.pose.position.y;
    double yaw = tf::getYaw(goal_pose.pose.orientation);

    // Calculate direction vectors
    double dx = cos(yaw);
    double dy = sin(yaw);

    double perp_dx = -dy;
    double perp_dy = dx;

    // Calculate line points
    for (double i = -2.0; i <= 2.0; i += step_size) {
        geometry_msgs::Pose point1, point2;

        point1.position.x = x_goal + boundary_distance * perp_dx + i * dx;
        point1.position.y = y_goal + boundary_distance * perp_dy + i * dy;
        point1.orientation = goal_pose.pose.orientation;

        point2.position.x = x_goal - 3*boundary_distance * perp_dx + i * dx;
        point2.position.y = y_goal - 3*boundary_distance * perp_dy + i * dy;
        point2.orientation = goal_pose.pose.orientation;

        // Add poses to the PoseArray
        boundary_points.poses.push_back(point1);
        boundary_points.poses.push_back(point2);
    }

    return boundary_points;
}

void DockingControl::publishBoundaryPoseArray(const geometry_msgs::PoseArray& boundary_points, ros::Publisher& publisher)
{
    // Publish the PoseArray
    publisher.publish(boundary_points);
}

/** Footprint Estimation **/
geometry_msgs::PolygonStamped DockingControl::generateFootprint(double top_left_x, double top_left_y,
                                                                double top_right_x, double top_right_y,
                                                                double bt_right_x, double bt_right_y,
                                                                double bt_left_x, double bt_left_y,
                                                                 const geometry_msgs::PoseStamped& robot_pose) 
{
    geometry_msgs::PolygonStamped footprint;
    footprint.header.frame_id = path_frame_;
    footprint.header.stamp = ros::Time::now();

    // Define the corners of the rectangle relative to the robot's center
    std::vector<geometry_msgs::Point32> points(4);

    // Top left
    points[0].x = top_left_x;
    points[0].y = top_left_y;

    // Top right
    points[1].x = top_right_x;
    points[1].y = top_right_y;

    // Bt left
    points[2].x = bt_right_x;
    points[2].y = bt_right_y;

    // Bt right
    points[3].x = bt_left_x;
    points[3].y = bt_left_y;

    // Transform points based on the robot's pose
    for (auto& point : points) {
        double cos_theta = cos(tf::getYaw(robot_pose.pose.orientation));
        double sin_theta = sin(tf::getYaw(robot_pose.pose.orientation));

        double x_new = robot_pose.pose.position.x + (point.x * cos_theta - point.y * sin_theta);
        double y_new = robot_pose.pose.position.y + (point.x * sin_theta + point.y * cos_theta);

        point.x = x_new;
        point.y = y_new;
    }

    // Add the points to the footprint polygon
    footprint.polygon.points.insert(footprint.polygon.points.end(), points.begin(), points.end());
    return footprint;
}

void DockingControl::publishFootprint(geometry_msgs::PolygonStamped footprint, ros::Publisher& publisher)
{
    publisher.publish(footprint);
}
/** Trajectory prediction using sampled control **/
// Generate control sample
/** TODO: Change sampling control function */
std::vector<geometry_msgs::Twist> DockingControl::generateControlSample(geometry_msgs::Twist current_cmd, 
                                                                            geometry_msgs::Twist cal_cmd)
{
    // std::vector<double> linear_vel_sample = linspace(current_cmd.linear.x, cal_cmd.linear.x, 5, true);
    double lower_linear_thres = current_cmd.linear.x - max_linear_acc_*dt_;
    if (lower_linear_thres < min_vel_) lower_linear_thres = min_vel_;
    std::vector<double> linear_vel_sample = linspace(cal_cmd.linear.x, cal_cmd.linear.x, 1, true);
    // std::vector<double> steering_sample = linspace(cal_cmd.angular.z, cal_cmd.angular.z, 1, true);
    std::vector<double> steering_sample;
    steering_sample.push_back(cal_cmd.angular.z);
    std::vector<geometry_msgs::Twist> control_sample;
    geometry_msgs::Twist control_tmp;
    for (int i = 0; i <= linear_vel_sample.size() - 1; i++)
    {
        for (int j = 0; j <= steering_sample.size() - 1; j++)
        {
            control_tmp.linear.x = linear_vel_sample.at(i);
            control_tmp.angular.z = steering_sample.at(j);
            control_sample.push_back(control_tmp);
        }
    }
    return control_sample;
}

// Predict path sample
nav_msgs::Path DockingControl::predictPath(geometry_msgs::Twist cmd_in)
{
    double pre_time, predict_time_max;
    pre_time = predict_time_;
    if (cmd_in.linear.x <= 0) predict_time_max = 0;
    else predict_time_max = pure_pursuit_control.look_ahead_distance_/cmd_in.linear.x;

    if (pre_time > predict_time_max) pre_time = predict_time_max;

    int number_sample = pre_time/dt_;
    nav_msgs::Path est_path;
    est_path.header.frame_id = path_frame_;
    est_path.header.stamp = ros::Time(0);
    est_path.poses.clear();

    geometry_msgs::PoseStamped pose_est;
    pose_est.header.frame_id = path_frame_;

    double v_r = cmd_in.linear.x;
    double steer = cmd_in.angular.z;
    double pre_yaw = 0, pre_x = 0, pre_y = 0;
    double yaw_est = 0;
    
    pose_est.pose.orientation = rpyToQuaternion(0,0,0);
    est_path.poses.push_back(pose_est);

    if (number_sample > 0)
    {
        for (int i = 0; i <= number_sample - 1; i++)
        {
            if(abs(steer) != M_PI/2) yaw_est = pre_yaw + dt_*(v_r*tan(steer)/l_wheelbase_);
            else yaw_est = pre_yaw;
            pose_est.pose.orientation = rpyToQuaternion(0, 0, yaw_est);
            
            pose_est.pose.position.x = pre_x + dt_*v_r*cos(pre_yaw);
            pose_est.pose.position.y = pre_y + dt_*v_r*sin(pre_yaw);
            est_path.poses.push_back(pose_est);

            pre_yaw = yaw_est;
            pre_x = pose_est.pose.position.x;
            pre_y = pose_est.pose.position.y;
        }
    }
    return est_path;
}

// Evaluate Path sample
double DockingControl::evaluateTrajectory(const nav_msgs::Path& trajectory, 
                                          const geometry_msgs::PoseStamped& target_pose, 
                                          const geometry_msgs::PoseArray& obstacles,
                                          geometry_msgs::Twist cmd_control,
                                          std::vector<double> &scaling_vel) 
{
    // Goal distance cost
    double headingCost = std::hypot(target_pose.pose.position.x - trajectory.poses.back().pose.position.x,
                                    target_pose.pose.position.y - trajectory.poses.back().pose.position.y);

    // Initialize minimum distance to obstacle
    std::vector<double> min_distance_samp;
    
    // Iterate through each pose in the trajectory
    double min_distance_to_obs;
    for (const auto& poseStamped : trajectory.poses) {
        const auto& pose = poseStamped.pose;

        // Generate the robot footprints for all the poses of each estimated trajectory
        double r_width = 1.0;
        geometry_msgs::PolygonStamped footprint = generateFootprint(
                                                    0.4, r_width/2,
                                                    0.4, -r_width/2,
                                                    -1.6, -r_width/2,
                                                    -1.6, r_width/2,
                                                    poseStamped);

        // Check each point in the footprint for proximity to obstacles
        min_distance_to_obs = std::numeric_limits<double>::infinity();
        for (const auto& footprint_point : footprint.polygon.points) {
            for (const auto& obs : obstacles.poses) { 
                double distance_to_obs = std::hypot(footprint_point.x - obs.position.x,
                                                       footprint_point.y - obs.position.y);

                min_distance_to_obs = std::min(min_distance_to_obs, distance_to_obs);
            }
        }
        min_distance_samp.push_back(min_distance_to_obs);  //save min distance for each footprint
    }

    // Calculate scaling factor for each footprint
    for (const auto& min_distance : min_distance_samp)
    {
        double scaling_tmp = 1;
        if (min_distance < velocity_penalty_dis_thres_) scaling_tmp = min_distance/velocity_penalty_dis_thres_;
        // ROS_INFO("min_distance_to_obs: %f", min_distance);
        scaling_vel.push_back(scaling_tmp);
    }

    // Velocity penalty based on proximity to obstacles
    double proximityVelocityPenalty = 0.0;

    // double const obstacle_distance_thresh = 0.3;
    // scaling_vel = 1;
    // if (min_distance_to_obs < obstacle_distance_thresh) {
    //     double velocity = cmd_control.linear.x; // Assuming linear.x represents the robot's velocity
    //     proximityVelocityPenalty = velocity / (min_distance_to_obs + 0.001);  // Add a small value to avoid division by zero
    //     scaling_vel = min_distance_to_obs/obstacle_distance_thresh;
    //     // ROS_INFO("scaling vel: %f", scaling_vel);
    // }

    ///TODO: Scale velocity to zero if current distance to obsacle < threshold

    // Velocity cost to prefer faster trajectory
    double velocityCost = cmd_control.linear.x; // Assuming linear.x represents the robot's final velocity

    if (min_distance_to_obs < 0.05) {
        min_distance_to_obs = -100; // Mark as unsafe
    }

    // ROS_INFO("headingCost: %f", headingCost);
    // ROS_INFO("velocityCost: %f", velocityCost);
    // ROS_INFO("proximityVelocityPenalty: %f", proximityVelocityPenalty);
    // Cost function
    double totalCost = (
        -gain_heading_ * headingCost +
         gain_track_ * min_distance_to_obs +
         gain_vel_ * velocityCost +
        -gain_penetrate_linear_vel_ * proximityVelocityPenalty  // Subtract penalty
    );

    return totalCost;
}

// Evaluate Control sample and re output control command
geometry_msgs::Twist DockingControl::evaluateControlSampleAndOutputControl(std::vector<geometry_msgs::Twist> control_samp)
{
    // Predict all the path with input control sample

    std::vector<nav_msgs::Path> path_sample;
    for (int i = 0; i <= control_samp.size() - 1; i++) {
        path_sample.push_back(predictPath(control_samp.at(i)));
    }

    // generate boundary. For temporary. will move to another function
    double step_size = 0.1;
    boundary_points_ = generateBoundaryPoints(local_goal_, 1.3/2, step_size);
    publishBoundaryPoseArray(boundary_points_, pub_boundary_point_);

     // prefer higher linear velocity
    double best_cost = -100;
    int best_index = 0;

    std::vector<std::vector<double>> scaling_vector_sample;
    std::vector<double> scaling_vector;
    for (int i = 0; i <= path_sample.size() - 1; i++)
    {
        double cost_tmp = evaluateTrajectory(path_sample.at(i), 
                                                pure_pursuit_control.pp_lookahead_pose_, 
                                                boundary_points_,
                                                control_samp.at(i),
                                                scaling_vector);
        scaling_vector_sample.push_back(scaling_vector);
        // ROS_INFO("linear_vel_cost %d: %f", i, linear_vel_cost.at(i));
        // ROS_INFO("cost_tmp %d: %f", i, cost_tmp);
        if (cost_tmp > best_cost)
        {
            best_cost = cost_tmp;
            best_index = i;
        }
    }
    if (!use_cost_function_) best_index = control_samp.size() - 1; //if not using cost function. Just output the original control signal 
    ROS_INFO("Best index: %d", best_index);
    
    // Scale control signal of each pose in the selected trajectory calculated from the best control sample in control_samp
    std::vector<geometry_msgs::Twist> control_scaled_vector;
    for (int i = 0; i < path_sample.at(best_index).poses.size(); i++)
    {
        control_scaled_vector.push_back(control_samp.at(best_index));   // All elements are equal to the best control sample
        control_scaled_vector.at(i).linear.x *= scaling_vector_sample.at(best_index).at(i);
        // ROS_INFO("%d: Origin linear: %f, Scaled linear: %f, Scaling Factor: %f",
        //     i, control_samp.at(best_index).linear.x, control_scaled_vector.at(i).linear.x, scaling_vector_sample.at(best_index).at(i));
    }
    ROS_INFO("0: Origin linear: %f, Scaled linear: %f, Scaling Factor: %f",
             control_samp.at(best_index).linear.x, control_scaled_vector.at(0).linear.x, scaling_vector_sample.at(best_index).at(0));
    // control_samp.at(best_index).linear.x *= scaling_vector_sample.at(best_index).at(0);
    
    predict_path_ = predictPath(control_samp.at(best_index));
    pub_docking_local_path_.publish(predict_path_);
    double r_width = 1.0;
    geometry_msgs::PolygonStamped current_pose_footprint = 
                                generateFootprint(0.4, r_width/2,
                                0.4, -r_width/2,
                                -1.6, -r_width/2,
                                -1.6, r_width/2,
                                predict_path_.poses.at(0));
    publishFootprint(current_pose_footprint, pub_footprint_);
    geometry_msgs::PolygonStamped last_pose_footprint = 
                                generateFootprint(0.4, r_width/2,
                                0.4, -r_width/2,
                                -1.6, -r_width/2,
                                -1.6, r_width/2,
                                predict_path_.poses.back());
    publishFootprint(last_pose_footprint, pub_estimated_footprint_);

    
    return control_scaled_vector.at(0); 
}

void DockingControl::visualize()
{
    /********** VISUALIZE LOOKAHEAD POINT MARKER ************/
    visualization_msgs::Marker points;
    points.header.frame_id =  path_frame_;
    points.header.stamp = ros::Time::now();
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.type = visualization_msgs::Marker::POINTS;
    points.color.b = 1.0f;
    points.color.a = 1.0;
    points.scale.x = 0.05;
    points.scale.y = 0.05;

    geometry_msgs::Point point_fuzzy;
    geometry_msgs::Point point_pp;
    geometry_msgs::Point closest_point;
    point_fuzzy.x = local_ref_path_.poses.at(lk_index_vel_).pose.position.x;
    point_fuzzy.y = local_ref_path_.poses.at(lk_index_vel_).pose.position.y;
    point_fuzzy.z = 0;
    point_pp.x = local_ref_path_.poses.at(pure_pursuit_control.point_index_).pose.position.x;
    point_pp.y = local_ref_path_.poses.at(pure_pursuit_control.point_index_).pose.position.y;
    point_pp.z = 0;
    closest_point.x = local_ref_path_.poses.at(nearest_index_).pose.position.x;
    closest_point.y = local_ref_path_.poses.at(nearest_index_).pose.position.y;
    closest_point.z = 0;

    std::vector<geometry_msgs::Point> my_points;
    my_points.push_back(point_fuzzy);
    my_points.push_back(point_pp);
    my_points.push_back(closest_point);

    for (int i = 0; i < my_points.size(); i++)
    {
        std_msgs::ColorRGBA c;
        if( i == 0)
        {
            c.r = 1.0;
            c.g = 1.0;
        }
        else if(i == 1)
            c.b = 1.0;
        else
            c.r = 1.0;
        c.a = 1.0;

        points.points.push_back(my_points[i]);
        points.colors.push_back(c);
    }
    marker_pub_.publish(points);
}


