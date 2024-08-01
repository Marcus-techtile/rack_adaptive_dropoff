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
    nh_.param<double>("look_ahead_time_straigh_line", pp_look_ahead_time_straigh_line_, 5.0);
    nh_.param<double>("pp_min_lk_distance_approaching", pp_min_lk_distance_approaching_, 0.2);
    nh_.param<double>("pp_min_lk_distance_docking", pp_min_lk_distance_docking_, 0.4);
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

    /* ROS Publisher */
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_local_path_ = nh_.advertise<nav_msgs::Path>("/pallet_docking/local_ref_path", 1);
    pub_docking_local_path_ = nh_.advertise<nav_msgs::Path>("/pallet_docking/docking_control_path", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/pallet_docking/marker", 1);
    pub_pp_lookahead_distance_ = nh_.advertise<std_msgs::Float32>("/pallet_docking/purepursuit_lookahead_distance", 1);
    pub_pp_lookahead_angle_ = nh_.advertise<std_msgs::Float32>("/pallet_docking/purepursuit_lookahead_angle", 1);
    pub_pp_lookahead_curvature_ = nh_.advertise<std_msgs::Float32>("/pallet_docking/purepursuit_lookahead_curvature", 1);
    pub_pp_steering_ = nh_.advertise<std_msgs::Float32>("/pallet_docking/pp_steering_angle", 1);
    pub_pp_lookahead_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/pallet_docking/pp_lookahead_pose", 1);
    pub_nearest_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/pallet_docking/nearest_pose", 1);

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
    pure_pursuit_control.setRefVel(final_ref_vel_);
    pure_pursuit_control.setClosestPoint(nearest_index_);
    if (!approaching_done_.data)
    {
        pure_pursuit_control.setLookaheadTime(pp_look_ahead_time_);
        pure_pursuit_control.limitLookaheadDistance(pp_min_lk_distance_approaching_, 2.5);
        if (adaptive_ref_angle_) pure_pursuit_control.setRefAngleMode(false);
    } 
    else
    {
        pure_pursuit_control.setLookaheadTime(pp_look_ahead_time_straigh_line_);
        pure_pursuit_control.limitLookaheadDistance(pp_min_lk_distance_docking_, 2.5);
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

    fuzzy_lk_dis = pure_pursuit_control.look_ahead_distance_;
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

    if (!use_cost_function_) {
        if (publish_cmd_) pub_cmd_vel_.publish(cmd_vel_);
        predict_path_ = predictPath(cmd_vel_);
        pub_docking_local_path_.publish(predict_path_);
    }
    else
    {
        std::vector<geometry_msgs::Twist> control_sampling = generateControlSample(pre_cmd_vel_, cmd_vel_);
        geometry_msgs::Twist cmd_out =  evaluateControlSampleAndOutputControl(control_sampling);
        
        if (publish_cmd_) pub_cmd_vel_.publish(cmd_out);
        predict_path_ = predictPath(cmd_out);
        pub_docking_local_path_.publish(predict_path_);
        pre_cmd_vel_ = cmd_out;
    }


}

nav_msgs::Path DockingControl::predictPath(geometry_msgs::Twist cmd_in)
{
    predict_time_ = pure_pursuit_control.look_ahead_distance_/cmd_in.linear.x;
    int number_sample = predict_time_/dt_;
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

std::vector<geometry_msgs::Twist> DockingControl::generateControlSample(geometry_msgs::Twist current_cmd, 
                                                                            geometry_msgs::Twist cal_cmd)
{
    // std::vector<double> linear_vel_sample = linspace(current_cmd.linear.x, cal_cmd.linear.x, 5, true);
    std::vector<double> linear_vel_sample = linspace(current_cmd.linear.x, cal_cmd.linear.x, 10, true);
    std::vector<double> steering_sample;
    if (cal_cmd.angular.z >= 0)
        steering_sample = linspace(0, cal_cmd.angular.z, 100, true);
    else steering_sample = linspace(cal_cmd.angular.z, 0, 100, true);
    // std::cout << "Linear vel sample: " << linear_vel_sample << std::endl;
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

geometry_msgs::Twist DockingControl::evaluateControlSampleAndOutputControl(std::vector<geometry_msgs::Twist> control_samp)
{
    // Predict all the path with input control sample
    std::vector<nav_msgs::Path> path_sample;
    for (int i = 0; i <= control_samp.size() - 1; i++) {
        path_sample.push_back(predictPath(control_samp.at(i)));
    }
    // Get final position of the path
    std::vector<geometry_msgs::PoseStamped> final_pose_sample;
    for (int i = 0; i <= path_sample.size() - 1; i++) {
        final_pose_sample.push_back(path_sample.at(i).poses.back());
    }

    // Calculate the cost
    std::vector<double> tracking_cost;
    std::vector<double> heading_cost;
    std::vector<double> linear_vel_cost; // prefer higher linear velocity
    double best_cost = -100;
    int best_index = 0;
    for (int i = 0; i <= final_pose_sample.size() - 1; i++)
    {
        heading_cost.push_back(abs(tf2::getYaw(final_pose_sample.at(i).pose.orientation) -
                                tf2::getYaw(pp_lkh_pose_.pose.orientation)));
        tracking_cost.push_back(hypot(final_pose_sample.at(i).pose.position.x - pp_lkh_pose_.pose.position.x,
                                final_pose_sample.at(i).pose.position.y - pp_lkh_pose_.pose.position.y));
        linear_vel_cost.push_back(control_samp.at(i).linear.x);
        double cost_tmp = -gain_heading_*heading_cost.at(i)  -gain_track_*tracking_cost.at(i) + gain_vel_*linear_vel_cost.at(i);
        // ROS_INFO("tracking_cost %d: %f", i, tracking_cost.at(i));
        // ROS_INFO("linear_vel_cost %d: %f", i, linear_vel_cost.at(i));
        // ROS_INFO("cost_tmp %d: %f", i, cost_tmp);
        if (cost_tmp > best_cost)
        {
            best_cost = cost_tmp;
            best_index = i;
        }
    }
    ROS_INFO("Best index: %d", best_index);
    return control_samp.at(best_index); 
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


