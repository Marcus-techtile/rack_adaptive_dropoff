#include "docking_control.h"

DockingControl::DockingControl(ros::NodeHandle &paramGet)
{
    /* Get Param */
    paramGet.param<double>("docking_freq", docking_freq_, 50.0);
    dt_ = 1 / docking_freq_;                        // Calculate sampling time
    ROS_INFO("Docking Control Frequency: %f", docking_freq_);
    ROS_INFO("Docking Control Sampling Time: %f", dt_);

    paramGet.param<std::string>("path_frame", path_frame_, "base_link_p");
    paramGet.param("/forklift_params/wheel_base", l_wheelbase_, 1.311);
    
    paramGet.param<bool>("publish_cmd", publish_cmd_, false);
    paramGet.param<double>("look_ahead_time", pp_look_ahead_time_, 3.0);
    paramGet.param<double>("look_ahead_time_straigh_line", pp_look_ahead_time_straigh_line_, 5.0);
    paramGet.param<double>("pp_min_lk_distance_approaching", pp_min_lk_distance_approaching_, 0.2);
    paramGet.param<double>("pp_min_lk_distance_docking", pp_min_lk_distance_docking_, 0.4);
    paramGet.param<double>("max_steering", max_steering_, 1.5);
    paramGet.param<double>("min_steering", min_steering_, -1.5);
    paramGet.param<double>("max_steering_speed", max_steering_speed_, 0.2);
    paramGet.param<double>("min_steering_speed", min_steering_speed_, -0.2);
    paramGet.param<double>("max_vel", max_vel_, 0.3);
    paramGet.param<double>("min_vel", min_vel_, 0.15);
    paramGet.param<double>("goal_correct_yaw", goal_correct_yaw_, 0.3);
    paramGet.param<double>("max_angular_vel", max_angular_vel_, 0.2);
    paramGet.param<bool>("adaptive_ref_angle", adaptive_ref_angle_, true);

    paramGet.param<double>("fuzzy_lookahead_dis", fuzzy_lookahead_dis_, 0.3);
    paramGet.param<bool>("limit_sp_curve", limit_sp_curve_, true);
    paramGet.param<double>("backward_offset", backward_offset_, -0.02);
    paramGet.param<double>("max_linear_acc", max_linear_acc_, 0.5);
    paramGet.param<double>("min_linear_acc", min_linear_acc_, -0.5);

    paramGet.param<double>("max_pocket_dock_vel", max_pocket_dock_vel_, 0.2);
    paramGet.param<double>("max_pocket_dock_steering", max_pocket_dock_steering_, 0.2);

    /* ROS Publisher */
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_local_path_ = nh_.advertise<nav_msgs::Path>("pallet_docking/local_ref_path", 1);
    pub_debug_ = nh_.advertise<std_msgs::Float32>("pallet_docking/debug", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/pallet_docking/marker", 1);
    pub_pp_lookahead_distance_ = nh_.advertise<std_msgs::Float32>("pallet_docking/purepursuit_lookahead_distance", 1);
    pub_pp_lookahead_angle_ = nh_.advertise<std_msgs::Float32>("pallet_docking/purepursuit_lookahead_angle", 1);
    pub_pp_lookahead_curvature_ = nh_.advertise<std_msgs::Float32>("pallet_docking/purepursuit_lookahead_curvature", 1);
    pub_pp_steering_ = nh_.advertise<std_msgs::Float32>("pallet_docking/pp_steering_angle", 1);
    pub_pp_lookahead_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("pallet_docking/pp_lookahead_pose", 1);
    pub_nearest_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("pallet_docking/nearest_pose", 1);

    /* ROS Subscriber */
    sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/gazebo/forklift_controllers/odom", 1, &DockingControl::odomCallback, this);
    sub_ref_path_ = nh_.subscribe<nav_msgs::Path>("/pallet_docking/quintic_path", 1, &DockingControl::refPathCallback, this);
    sub_controller_on_ = nh_.subscribe<std_msgs::Bool>("/pallet_docking/controller_turn_on", 1, &DockingControl::controllerOnCallback, this);
    sub_approaching_status_ = nh_.subscribe<std_msgs::Bool>("/pallet_docking/pallet_approaching_done", 1, &DockingControl::approachingStatusCallback, this);
    joint_states_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &DockingControl::JointStateCallBack, this);

    /* Define the Controller */
    fuzzy_controller = FuzzyControl(paramGet);
    pure_pursuit_control = PurePursuitController(paramGet);

    /* Initialize parameters */
    init_reconfig_ = true;
    ref_path_avai_ = false;
    controller_on_.data = false;
    approaching_done_.data = false;
    pub_stop_ = false;
}

DockingControl::~DockingControl(){}

/***** Callback function *****/
void DockingControl::odomCallback(const nav_msgs::Odometry::ConstPtr& msg_odom)
{
    odom_sub_.pose = msg_odom->pose;
    odom_sub_.twist = msg_odom->twist;
    odom_avai_ = true;
}

void DockingControl::JointStateCallBack(const sensor_msgs::JointState::ConstPtr &msg)
{
    for (int i = 0; i < msg->name.size(); i++) {
        if (msg->name.at(i) == "drive_steer_joint" ) {
        steering_sub_ = msg->position.at(i);
        ROS_DEBUG("JointStateCallBack: steering angle: %f", steering_sub_);
        break;
        }
    }
}

void DockingControl::refPathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    ref_path_ = *msg;
    if (ref_path_.poses.size() > 1)
        ref_path_avai_ = true;
    else ref_path_avai_ = false;
}

void DockingControl::controllerOnCallback(const std_msgs::Bool::ConstPtr& msg)
{
    controller_on_.data = msg->data;
    ROS_INFO("CONTROLLER IS TRIGGERED: %d !", controller_on_.data);
}

void DockingControl::approachingStatusCallback(const std_msgs::Bool::ConstPtr& msg)
{
    approaching_done_.data = msg->data;
}

/***** Control Function *****/
void DockingControl::resetController()
{
    invalid_control_signal_ = false;
    controller_on_.data = false;
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.angular.z = 0.0;
    pub_cmd_vel_.publish(cmd_vel_);
    pure_pursuit_control.resetPP();
    ref_path_.poses.clear();
    ref_path_avai_ = false;
    steering_ = 0;
    final_ref_vel_ = 0;
}

/***** Check Required Data *****/
bool DockingControl::checkData()
{
    if(!odom_avai_)
    {
        ROS_WARN("No Odometry!!!");
        return false; 
    }
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
            local_ref_path.poses.push_back(tf_buffer_c.transform(global_path.poses.at(i), path_frame_, ros::Duration(1)));
        }
        catch (tf2::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }
    }
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
    int nearest_pose_index = nearestPointIndexFind(local_ref_path_);
    ROS_DEBUG ("Nearest index: %d", nearest_pose_index);

    /******** STEERING CONTROL *********/  
    std_msgs::Float32 debug_p;
    debug_p.data = pure_pursuit_control.lateral_heading_error_.data;
    pub_debug_.publish(debug_p);
    pure_pursuit_control.setOdom(odom_sub_);
    pure_pursuit_control.setRefPath(local_ref_path_);
    pure_pursuit_control.setRefVel(final_ref_vel_);
    pure_pursuit_control.setClosestPoint(nearest_pose_index);
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

    // Publish PP steering for debug 
    std_msgs::Float32 pp_steer;
    pp_steer.data = pure_pursuit_control.PP_steering_angle_;
    pub_pp_steering_.publish(pp_steer);     // pp steering angle before being added PID

    // Visualize PP lookahead distance and lookahead angle
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

    /******** VELOCITY CONTROL ****/
    int lk_index;
    double fuzzy_lk_dis = fuzzy_lookahead_dis_;
    double max_dist = sqrt(local_ref_path_.poses.at(local_ref_path_.poses.size()-1).pose.position.x*local_ref_path_.poses.at(local_ref_path_.poses.size()-1).pose.position.x
                        + local_ref_path_.poses.at(local_ref_path_.poses.size()-1).pose.position.y*local_ref_path_.poses.at(local_ref_path_.poses.size()-1).pose.position.y);
    
    if (fuzzy_lk_dis > max_dist) fuzzy_lk_dis = max_dist;
    for (lk_index = nearest_pose_index; lk_index < local_ref_path_.poses.size(); lk_index++)
    {
        if (abs(fuzzy_lk_dis) <= sqrt(local_ref_path_.poses.at(lk_index).pose.position.x*local_ref_path_.poses.at(lk_index).pose.position.x
                                    + local_ref_path_.poses.at(lk_index).pose.position.y*local_ref_path_.poses.at(lk_index).pose.position.y)) break;  
    }
    
    if (abs(local_ref_path_.poses.at(local_ref_path_.poses.size()-1).pose.position.x) < fuzzy_lk_dis)
        lk_index = local_ref_path_.poses.size()-1;
    
    fuzzy_controller.inputSolveGoal(abs(fuzzy_lk_dis));
    if (limit_sp_curve_)
        fuzzy_controller.inputsolveSteering(pure_pursuit_control.look_ahead_curvature_);
    else fuzzy_controller.inputsolveSteering(0.0);
    fuzzy_controller.inputResults();
    
    if (limit_sp_curve_)
        ref_velocity_ = fuzzy_controller.cal_fuzzy_output();
    else
        ref_velocity_ = fuzzy_controller.cal_fuzzy_output() * cos(steering_);
    
    // Smooth the velocity output. Limit linear acceleration
    double linear_acc = (ref_velocity_ - abs_ref_vel_)/dt_;
    if (linear_acc > max_linear_acc_) linear_acc = max_linear_acc_;
    if (linear_acc < min_linear_acc_) linear_acc = min_linear_acc_;
    abs_ref_vel_ = abs_ref_vel_ + linear_acc * dt_;

    final_ref_vel_ = abs_ref_vel_;
    if (local_ref_path_.poses.at(lk_index).pose.position.x < backward_offset_)
        if (final_ref_vel_ > 0) final_ref_vel_ = -final_ref_vel_;

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

    /*********** NAN OUTPUT HANDLE *************/
    if(isnan(steering_) || isnan(final_ref_vel_))
    {
        invalid_control_signal_ = true;
        ROS_ERROR("NAN NUMBER ! RESET CONTROLLER");
        controller_on_.data = false;
        resetController();
        return;
    }

    cmd_vel_.linear.x = final_ref_vel_;
    cmd_vel_.angular.z = steering_;

    if (publish_cmd_) pub_cmd_vel_.publish(cmd_vel_);

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
    point_fuzzy.x = local_ref_path_.poses.at(lk_index).pose.position.x;
    point_fuzzy.y = local_ref_path_.poses.at(lk_index).pose.position.y;
    point_fuzzy.z = 0;
    point_pp.x = local_ref_path_.poses.at(pure_pursuit_control.point_index_).pose.position.x;
    point_pp.y = local_ref_path_.poses.at(pure_pursuit_control.point_index_).pose.position.y;
    point_pp.z = 0;
    closest_point.x = local_ref_path_.poses.at(nearest_pose_index).pose.position.x;
    closest_point.y = local_ref_path_.poses.at(nearest_pose_index).pose.position.y;
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


