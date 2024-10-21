#include "docking_planner.h"

/****** Constructor *******/
// tf: tf_buffer
// sec: tf_buffer timeout
DockingManager::DockingManager(ros::NodeHandle &nh, tf2_ros::Buffer &tf, double sec): nh_(nh), tf_(tf), tf_time_out_(sec)
{   
    /* Docking Control init */
    docking_control_ = std::make_shared<DockingControl>(nh_, tf_, tf_time_out_);
    /* Quintic Planner init */
    quintic_planner_ = std::make_shared<QuinticPlanner>(nh_, tf_, tf_time_out_);
    /* Publisher */
    pub_docking_state = nh_.advertise<std_msgs::String>("/pallet_docking/docking_state", 1);
    pub_docking_done = nh_.advertise<std_msgs::Bool>("/pallet_docking/pallet_docking_done", 1);
    pub_approaching_done = nh_.advertise<std_msgs::Bool>("/pallet_docking/pallet_approaching_done", 1);
    pub_cmd_vel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_goal_pose_array_ = nh_.advertise<geometry_msgs::PoseArray>("/pallet_docking/goal_pose_array", 10);
    pub_global_goal_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/pallet_docking/global_goal_pose", 10);
    pub_local_goal_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/pallet_docking/local_goal_pose", 10);
    pub_docking_error_ = nh_.advertise<geometry_msgs::Vector3>("/pallet_docking/docking_error", 1);
    pub_approaching_error_ = nh_.advertise<geometry_msgs::Vector3>("/pallet_docking/approaching_error", 1);
    pub_final_docking_error_ = nh_.advertise<geometry_msgs::Vector3>("/pallet_docking/final_docking_error", 1);
    
    sub_rack_deviation_ = nh_.subscribe<rack_detection_msg::RackDeviation>("/rack_detection/fusioned_rack_deviation", 1, &DockingManager::rackDeviationCallback, this);
    sub_rack_line_ = nh_.subscribe<rack_detection_msg::RackLine>("/rack_detection/fusioned_rack_line", 1, &DockingManager::rackLineCallback, this);

    /*Define failure cases */
    failure_map_[0] = "PATH_IS_NOT_FEASIBLE";
    failure_map_[1] = "BAD_DOCKING_ACCURACY";
    failure_map_[2] = "DOCKING_TIME_OUT";
    failure_map_[3] = "INVALID_CONTROL_OUTPUT";
    
    // Init the Docking
    initDocking();
}

DockingManager::~ DockingManager(){}

/***** SET DOCKING SERVER PARAMS ******/
void DockingManager::config()
{
    nh_.param<std::string>("rack_deviation_topic", rack_deviation_topic_, "/rack_detection/fusioned_rack_deviation");
    /* Get Param */
    nh_.param<std::string>("/move_base_flex/AD/global_frame_id", global_frame_, "map");
}

/***** RESET PALLET DOCKING ******/
void DockingManager::initDocking()
{
    current_pallet_docking_state_ = IDLE;
    docking_state.data = "IDLE";
    pub_docking_state.publish(docking_state);

    docking_result = dockingResult::PROCESS;
    
    start_pallet_docking_ = false;
    start_returning_ = false;

    pose_setup_ = false;

    norm_goal_frame_ = false;
    
    docking_done.data = false;          
    approaching_done.data = false;
    docking_failed.data = false;

    docking_control_->resetController();
    quintic_planner_->resetPlanner();

    check_inside_goal_range_ = false;
    count_outside_goal_range_ = 0;

    approaching_error_.x = approaching_error_.y = approaching_error_.z = 0;
    final_error_.x = final_error_.y = final_error_.z = 0;

    // reset the transition state
    pallet_pose_avai_ = false;
    approach_done_ = false;     // transition from APPROACHING to DOCKING, to SET_GOAL
    docking_done_ = false;       // transition from DOCKING to SET_GOAL
    goal_setup_ = false;        // transition from SET_GOAL to UPDATE_GOAL
    goal_avai_ = false;         // transition from UPDATE_GOAL to GEN_PATH
    goal_reach_ = false;         // transition from GEN_PATH to STOP
    goal_failed_ = false;        // transition from GEN_PATH to RECOVER
}

/***** RESET PLANNING AND CONTROL ******/
void DockingManager::resetPlanAndControl()
{
    docking_control_->resetController();
    quintic_planner_->resetPlanner();
    goal_setup_ = false;
    count_outside_goal_range_ = 0;
}

/**** Params Setup ******/
bool DockingManager::setupPoses(geometry_msgs::PoseStamped approaching_pose,
                    geometry_msgs::PoseStamped docking_pose)
{
    if (approaching_pose.header.frame_id == "" || docking_pose.header.frame_id == "" )
        
    {
        ROS_WARN("Pose frame is empty");
        pose_setup_ = false;
        return false;
    }
    else
    {
        approaching_goal_ = approaching_pose;
        docking_goal_ = docking_pose;
        pose_setup_ = true;
        return true;
    }  
}

void DockingManager::setGoalRange(double dd)
{
    goal_range_ = dd;
}

void DockingManager::setApproachingTolerance(double dx, double dy, double dyaw)
{
    app_x_tolerance_ = dx;
    app_y_tolerance_ = dy;
    app_angle_tolerance_ = dyaw;
}

void DockingManager::setDockingTolerance(double &dx, double &dy, double &dyaw)
{
    if (dx < limit_tol_x_ || dy < limit_tol_y_ || dyaw < limit_tol_angle_) 
    {
        if (dx < limit_tol_x_) dx = limit_tol_x_;
        if (dy < limit_tol_y_) dy = limit_tol_y_;
        if (dyaw < limit_tol_angle_) dyaw = limit_tol_angle_;
        ROS_WARN("Invalid tolerances. Limit to minimum values (%f, %f, %f)", dx, dy, dyaw);
    }
    docking_x_tolerance_ = dx;
    docking_y_tolerance_ = dy;
    docking_angle_tolerance_ = dyaw;
}

void DockingManager::setLocalFrame(std::string local_frame)
{
    path_frame_ = local_frame;
    docking_control_->path_frame_ = local_frame;
    quintic_planner_->path_frame_ = local_frame;
}

void DockingManager::setGLobalFrame(std::string global_frame)
{
    global_frame_ = global_frame;
    quintic_planner_->global_frame_ = global_frame;
    docking_control_->global_frame_ = global_frame;
}


/***** UPDATE GOAL EACH CONTROL PERIOD *****/
void DockingManager::getAndPubDockingErrors(double error_x, double error_y, double error_yaw)
{
    if (!approach_done_)
    {
        approaching_error_.x = error_x; approaching_error_.y = error_y; approaching_error_.z = error_yaw;
        pub_approaching_error_.publish(approaching_error_);
    }
    else
    {
        final_error_.x = error_x; final_error_.y = error_y; final_error_.z = error_yaw;
        pub_final_docking_error_.publish(final_error_);
    }
}

void DockingManager::updateGoal()
{
    /* Transform goal pose to path_frame */
    global_goal_pose_.header.stamp = ros::Time(0);
    try
    {
        tf_.lookupTransform(path_frame_, global_frame_, ros::Time(0));
        tf_.transform(global_goal_pose_, local_update_goal_pose_, path_frame_, ros::Duration(tf_time_out_));
    }
    catch (tf2::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    if (rack_deviation_avai_ && rack_line_avai_)
    {
        /*** Update the Rack deviation for the local goal ****/
        double a = rack_line_.vector.x, b = rack_line_.vector.y; //rack line direction vector
        double xc = rack_line_.centroid.x, yc = rack_line_.centroid.y; //rack line centroid
        double xb = local_update_goal_pose_.pose.position.x;
        double yb = local_update_goal_pose_.pose.position.y;

        // project the local goal to the rack line to correct the lateral angle
        double t = ((xb - xc) * a + (yb - yc) * b) / (a * a + b * b);
        double xp = xc + t * a;
        double yp = yc + t * b;

        local_update_goal_pose_.pose.position.x = xp;
        local_update_goal_pose_.pose.position.y = yp;
        local_update_goal_pose_.pose.orientation = rpyToQuaternion(0,0, rack_deviation_.orientation_deviation);
    }
    else
    {
        // ROS_WARN("Rack detection output is not available. Use the global goal");
    }
    rack_deviation_avai_ = false;
    rack_line_avai_ = false;


    pub_global_goal_pose_.publish(global_goal_pose_);
    pub_local_goal_pose_.publish(local_update_goal_pose_);
    goal_avai_ = true; 
}

void DockingManager::checkGoalReach()
{
    std::lock_guard<std::mutex> lock_goal_reack_check(mutex_);
    double error_x = local_update_goal_pose_.pose.position.x;
    double error_y = local_update_goal_pose_.pose.position.y;
    double error_yaw = tf::getYaw(local_update_goal_pose_.pose.orientation);
    double error_sq = sqrt(error_x*error_x + error_y*error_y);
    geometry_msgs::Vector3 docking_error;
    docking_error.x = error_x;
    docking_error.y = error_y;
    docking_error.z = error_yaw;
    pub_docking_error_.publish(docking_error);
    if (docking_mode_ == 0)     // pallet docking mode
    {
        if (!approach_done_)        // tolerance for approaching
        {
            x_tolerance_ = app_x_tolerance_;
            y_tolerance_ = app_y_tolerance_;
            angle_tolerance_ = app_angle_tolerance_;
        }
        else                        // tolerance for final docking
        {
            x_tolerance_ = docking_x_tolerance_;
            y_tolerance_ = docking_y_tolerance_;
            angle_tolerance_ = docking_angle_tolerance_;
        }
    }
    else                            // returning mode
    {
        x_tolerance_ = app_x_tolerance_;
        y_tolerance_ = app_y_tolerance_;
        angle_tolerance_ = app_angle_tolerance_;
    }

    if (error_sq <= goal_range_) 
    {
        check_inside_goal_range_ = true;
        if (abs(error_yaw) < angle_tolerance_ && abs(error_x) < x_tolerance_ && abs(error_y) < y_tolerance_)
        {
            check_inside_goal_range_ = false;
            goal_reach_ = true;
            getAndPubDockingErrors(error_x, error_y, error_yaw);              
            ROS_INFO("Docking Error [x , y, yaw]: %f (m), %f (m), %f (rad)", error_x, error_y, error_yaw);
            return;
        } 
    }

    if (check_inside_goal_range_)
    {
        if (!approach_done_)
        {
            if (error_sq > goal_range_) count_outside_goal_range_++;
        }
        else
        {
            if (error_x < -0.01 || error_sq > goal_range_) count_outside_goal_range_++;
        }
        if (count_outside_goal_range_ > 10)      // Prevent jumping in and out goal range
        {
            count_outside_goal_range_ = 0;
            ROS_WARN("Failed!!! !");
            getAndPubDockingErrors(error_x, error_y, error_yaw); 
            ROS_INFO("Docking Error [x , y, yaw]: %f (m), %f (m), %f (rad)", error_x, error_y, error_yaw);
            check_inside_goal_range_ = false;
            goal_failed_ = true;
            return; 
        }
    }
}

bool DockingManager::isApproachingReach()
{
    if (approaching_done.data) return true;
    else return false;
}

bool DockingManager::isGoalReach()
{
    if (approaching_done.data && docking_done.data) return true;
    else return false;
}


/***** PLANNER STATE ******/
////// Idle State /////
void DockingManager::idleState()
{
    docking_state.data = "IDLE";
    if (start_pallet_docking_ || start_returning_) current_pallet_docking_state_ = APPROACHING;
    else return;
}

////// Approaching State /////
void DockingManager::approachingState()
{
    docking_state.data = "APPROACHING";
    if (!approaching_done.data)
    {
        current_pallet_docking_state_ = SET_GOAL;  
    }
    else 
    {
        approaching_done.data = true;
        current_pallet_docking_state_ = DOCKING;
    }
    docking_control_->approaching_done_.data = approach_done_;
}

////// Docking State /////
void DockingManager::dockingState()
{
    docking_state.data = "DOCKING";
    current_pallet_docking_state_ = SET_GOAL;
}

////// Set goal State /////
/***** SETUP GOAL FOR DOCKING *****/

void DockingManager::goalSetup()
{   
    // Normalize the frame into a consistent one
    if (!norm_goal_frame_)
    {
        approaching_goal_.header.stamp = ros::Time(0);
        docking_goal_.header.stamp = ros::Time(0);
        try
        {
            tf_.transform(approaching_goal_, tf_approaching_goal_, global_frame_, ros::Duration(tf_time_out_));
            tf_.transform(docking_goal_, tf_docking_goal_, global_frame_, ros::Duration(tf_time_out_));
        }
        catch (tf2::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }
        norm_goal_frame_ = true;
    }
    geometry_msgs::PoseArray goal_array;
    goal_array.header.frame_id = tf_approaching_goal_.header.frame_id;
    goal_array.header.stamp = ros::Time::now();
    goal_array.poses.push_back(tf_approaching_goal_.pose);
    goal_array.poses.push_back(tf_docking_goal_.pose);
    pub_goal_pose_array_.publish(goal_array);
       
    // Setup goal pose for each stage
    if (!approach_done_) global_goal_pose_ = tf_approaching_goal_;
    else   global_goal_pose_ = tf_docking_goal_;
    
    pub_global_goal_pose_.publish(global_goal_pose_);
    goal_setup_ = true;  
}

void DockingManager::setGoalState()
{
    docking_state.data = "SET_GOAL";
    if (!pose_setup_) return;
    if (!goal_setup_)
    {
        goalSetup();
        goal_setup_ = true;
    } 
    
    if (goal_setup_)
    {
        current_pallet_docking_state_ = GEN_PATH_AND_PUB_CONTROL;
        goal_setup_ = false;
    } 
}

////// Gen Path State /////
void DockingManager::setRobotSpeed(geometry_msgs::Twist robot_speed)
{
    docking_control_->setVel(robot_speed);
}

void DockingManager::quinticPlannerSetup()
{
    if (returning_mode_.data)
    {
        if (quintic_planner_->starting_vel_ > 0) quintic_planner_->starting_vel_ = -quintic_planner_->starting_vel_;
        if (quintic_planner_->stopping_vel_ > 0) quintic_planner_->stopping_vel_ = -quintic_planner_->stopping_vel_;
    }
    else
    {
        if (quintic_planner_->starting_vel_ < 0) quintic_planner_->starting_vel_ = -quintic_planner_->starting_vel_;
        if (quintic_planner_->stopping_vel_ < 0) quintic_planner_->stopping_vel_ = -quintic_planner_->stopping_vel_;
    }
    quintic_planner_->setParams(0.0, 0.0, 0.0, quintic_planner_->starting_vel_, quintic_planner_->starting_acc_,
            local_update_goal_pose_.pose.position.x, local_update_goal_pose_.pose.position.y, tf::getYaw(local_update_goal_pose_.pose.orientation),
            quintic_planner_->stopping_vel_, quintic_planner_->stopping_acc_);
    // if (!quintic_planner_->path_avai_)
        quintic_planner_->genPath();
    quintic_planner_->visualize(global_goal_pose_);
}

void DockingManager::goalReachHandling()
{
    resetPlanAndControl();
    goal_reach_ = false;
    ROS_INFO("Goal reach !!!");
    if (!approach_done_) approach_done_ = true;
    else docking_done_ = true;
    current_pallet_docking_state_ = STOP;
}

void DockingManager::goalFailHandling()
{
    resetPlanAndControl();
    failure_code_ = 1;
    current_pallet_docking_state_ = FAILURE;
    ROS_INFO("Goal failed !!!");
}

void DockingManager::genPathAndPubControlState()
{
    docking_state.data = "GEN_PATH_AND_PUB_CONTROL";
    updateGoal();
    if (!goal_avai_) return;
    goal_avai_ = false;  //reset goal update for the next cycle
    checkGoalReach();
    quinticPlannerSetup();
    if (!quintic_planner_->path_feasible_)
    {
        ROS_WARN("PATH IS NOT FEASIBLE");
        failure_code_ = 0;
        current_pallet_docking_state_ = FAILURE;
        return;
    } 
    if (goal_reach_)
    {
        goalReachHandling();
        return;
    } 
    if (goal_failed_)
    {
        goalFailHandling();
        return;
    } 

    // Start the controller
    if (quintic_planner_->path_avai_ && !docking_control_->controller_on_.data)
    {
        docking_control_->setRefPath(quintic_planner_->global_quintic_path_);
        docking_control_->setLocalGoal(local_update_goal_pose_);
        docking_control_->controller_on_.data = true;
    }
    docking_control_->setRefPath(quintic_planner_->quintic_path_);
    docking_control_->setLocalGoal(local_update_goal_pose_);            // Fix local goal for the final pose of the ref path
    docking_control_->controllerCal();
    if (docking_control_->invalid_control_signal_)
    {
        docking_state.data = "FAILURE";
        failure_code_ = 3;
        current_pallet_docking_state_ = FAILURE;
        return;
    }
}

////// Recover State /////
void DockingManager::recoverState()
{
    docking_state.data = "RECOVER";
}

////// End State /////
void DockingManager::endState()
{
    docking_state.data = "END";
}

////// Stop State /////
void DockingManager::stopState()
{
    docking_state.data = "STOP";
    resetPlanAndControl();
    ros::Duration(1.0).sleep();
    if (docking_done_)
    {
        docking_done.data = true;
        current_pallet_docking_state_ = END;
        if (isGoalReach()) docking_result = dockingResult::SUCCESS;
    } 
    else 
    {
        current_pallet_docking_state_ = APPROACHING;
        ros::Duration(1.0).sleep();
    } 
}

////// Failure State /////
void DockingManager::failureState()
{
    resetPlanAndControl();
    ROS_ERROR("FAILURE: %s. ABORTED!", failure_map_[failure_code_].c_str());
    docking_state.data = "FAILURE: " + failure_map_[failure_code_];
    docking_failed.data = true;
    switch (failure_code_)
    {
        case 0:
            docking_result = dockingResult::FAIL_DOCKING_PATH_IS_NOT_FEASIBLE;
            break;
        case 1:
            docking_result = dockingResult::FAIL_DOCKING_BAD_ACCURACY;
            break;
        case 2: 
            docking_result = dockingResult::FAIL_TF_ERROR;
            break;
        case 3:
            docking_result = dockingResult::FAIL_INVALID_CONTROL_OUTPUT;
            break;
    }
    current_pallet_docking_state_ = END;
}

////// FSM setup ///////
bool DockingManager::startFSM()
{
    // if (!start_docking_FSM) return false;
    // if (start_pallet_docking_ && !start_returning_) 
    // {
    //     returning_mode_.data = false;        //docking
    //     return true;
    // }
    // else if (!start_pallet_docking_ && start_returning_) 
    // {
    //     returning_mode_.data = true;             // returning. move back
    //     return true;
    // } 
    // else
    // {
    //     ROS_WARN("BAD DOCKING STATE! Both pallet docking and returning is turned on or off!");
    //     return false;
    // } 
    start_pallet_docking_ = true;
    return true;
}

/***** DOCKING FSM ******/
void DockingManager::dockingFSM()
{
    if (!startFSM()) return;            // check FSM starting state and FSM mode

    if (old_docking_state.data != docking_state.data)
    {
        pub_docking_state.publish(docking_state);
        ROS_INFO("_DOCKING STATE: %s", docking_state.data.c_str());
    }
    old_docking_state = docking_state;
    
    // FM transition
    switch(current_pallet_docking_state_)
    {   case IDLE:
            idleState();
            break;
        case APPROACHING:
            approachingState();
            break;
        case DOCKING:
            dockingState();
            break;
        case SET_GOAL:
            setGoalState();
            break;
        case GEN_PATH_AND_PUB_CONTROL:
            genPathAndPubControlState();
            break;
        case STOP:
            stopState();
            break;
        case RECOVER:
            recoverState();
            break;
        case END:
            endState();
            break;
        case FAILURE:
            failureState();
            break;
        default:
            break; 
    } 
    pub_docking_done.publish(docking_done); 
    pub_approaching_done.publish(approaching_done);
}

/***** RETURN *****/
geometry_msgs::Twist DockingManager::getCmdVel()
{
    ROS_DEBUG("PLanner CMD_VEL (v,w): %f, %f", docking_control_->cmd_vel_.linear.x,
                                        docking_control_->cmd_vel_.angular.z);
    return docking_control_->cmd_vel_;
}

geometry_msgs::Vector3 DockingManager::getDockingFinalError()
{
    if (current_pallet_docking_state_ != END)
    {
        ROS_WARN("Docking is still in progress. Return 0 for final errors");
        final_error_.x = final_error_.y = final_error_.z = 0;
    }
    return final_error_;
}

uint8_t DockingManager::getDockingResult() {return static_cast<int>(docking_result);}


void DockingManager::rackDeviationCallback(const rack_detection_msg::RackDeviation::ConstPtr &msg)
{
    rack_deviation_ = *msg;
    rack_deviation_avai_ = true;
}

void DockingManager::rackLineCallback(const rack_detection_msg::RackLine::ConstPtr &msg)
{
    rack_line_ = *msg;
    rack_line_avai_ = true;
}
