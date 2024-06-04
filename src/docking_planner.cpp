#include "docking_planner.h"

DockingManager::DockingManager(ros::NodeHandle &nh): nh_(nh)
{
    setParam(nh_);
    
    /* Publisher */
    pub_docking_state = nh_.advertise<std_msgs::String>("/pallet_docking/docking_state", 1);
    pub_docking_done = nh_.advertise<std_msgs::Bool>("/pallet_docking/pallet_docking_done", 1);
    pub_approaching_done = nh_.advertise<std_msgs::Bool>("/pallet_docking/pallet_approaching_done", 1);
    pub_cmd_vel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_controller_on_ = nh_.advertise<std_msgs::Bool>("/pallet_docking/controller_turn_on", 1);
    pub_goal_pose_array_ = nh_.advertise<geometry_msgs::PoseArray>("/pallet_docking/goal_pose_array", 10);
    pub_global_goal_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/pallet_docking/global_goal_pose", 10);
    pub_local_goal_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/pallet_docking/local_goal_pose", 10);
    pub_docking_error_ = nh_.advertise<geometry_msgs::Vector3>("/pallet_docking/docking_error", 1);

    /* Subscriber */
    sub_docking_server_result_ = nh.subscribe<pallet_dock_msgs::PalletDockingActionResult>("/pallet_dock_action_server/pallet_docking/result", 1, &DockingManager::dockingServerResultCallback, this);
    sub_docking_server_goal_ = nh_.subscribe<pallet_dock_msgs::PalletDockingActionGoal>("/pallet_dock_action_server/pallet_docking/goal", 1, &DockingManager::dockingServerGoalCallback, this);

    /* Service*/
    service_ = nh_.advertiseService("/pallet_docking_service", &DockingManager::dockingServiceCb, this);
    
    /*Define failure cases */
    failure_map_[2] = "PATH_IS_NOT_FEASIBLE";
    failure_map_[3] = "BAD_DOCKING_ACCURACY";

    // Init the Docking
    initDocking();
}

DockingManager::~ DockingManager(){}

/***** SET DOCKING PARAMS ******/
void DockingManager::setParam(ros::NodeHandle &nh)
{
    /* Get Param */
    nh_.param<std::string>("global_frame", global_frame_, "odom");
    nh_.param<std::string>("path_frame", path_frame_, "base_link_p");

    /* Goal params */
    nh_.param<double>("approaching_min_dis", approaching_min_dis_, 1.2);
    nh_.param<double>("moveback_straight_distance", moveback_straight_distance_, 1.0);

    /* Tolerance params */
    /* TODO: Get tolerances from the action server */
    nh_.param<double>("distance_tolerance", distance_tolerance_, 0.08);
    nh_.param<double>("angle_tolerance", angle_tolerance_, 5*180/M_PI);
    nh_.param<double>("final_angle_tolerance", final_angle_tolerance_, 1*180/M_PI);
    nh_.param<double>("x_tolerance", x_tolerance_, 0.05);
    nh_.param<double>("final_x_tolerance", final_x_tolerance_, 0.05);
    nh_.param<double>("y_tolerance", y_tolerance_, 0.04);
    nh_.param<double>("final_y_tolerance", final_y_tolerance_, 0.04);
}

/***** DOCKING SERVER RESULT CALLBACK ******/
void DockingManager::dockingServerResultCallback(const pallet_dock_msgs::PalletDockingActionResult::ConstPtr& msg)
{
    docking_server_result_ = *msg;
    if (docking_server_result_.result.code >= 2)
    {
        initDocking();
        ROS_WARN("Failure Result from Docking action server! Stop the FSM!");
    }
}

/***** DOCKING SERVER GOAL CALLBACK ******/
void DockingManager::dockingServerGoalCallback(const pallet_dock_msgs::PalletDockingActionGoal::ConstPtr& msg)
{
    docking_server_goal_ = *msg;
    docking_mode_ = docking_server_goal_.goal.mode;
    if (docking_mode_ == 0)         // mode pallet docking
    {
        start_pallet_docking_ = true;
        start_returning_ = false;
        pallet_pose_ = docking_server_goal_.goal.docking_pose;
        // rotate angle 180 degree
        double r, p, yaw;
        quaternionToRPY(pallet_pose_.pose.orientation, r, p, yaw);
        yaw = yaw - M_PI;
        pallet_pose_.pose.orientation = rpyToQuaternion(r, p, yaw);
    }
    else                            // mode returning
    {
        start_pallet_docking_ = false;
        start_returning_ = true;
        pallet_pose_ = docking_server_goal_.goal.docking_pose;
    }
    // pub_global_goal_pose_.publish(pallet_pose_);

    approaching_min_dis_ = docking_server_goal_.goal.move_back_distance;  // enable moveback motion for pallet docking
    pallet_pose_avai_ = true;
    ROS_INFO("Receive docking goal");
}

/***** DOCKING SERVICE CALLBACK *****/
bool DockingManager::dockingServiceCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    if(req.data)
    {
        res.success = true;
        res.message = "Start docking!"; 
        start_docking_FSM = true;
        ROS_INFO("Docking is turned on");
    }
    else
    {
        initDocking();
        resetPlanAndControl();
        start_docking_FSM = false;
        res.success = true;
        res.message = "Stop docking!";
        returning_mode_.data = false;
        ROS_INFO("Docking is turned off");
    }
    return true;
}


/***** UPDATE GOAL EACH CONTROL PERIOD *****/
void DockingManager::updateGoal()
{
    /* Transform goal pose to path_frame */
    global_goal_pose_.header.stamp = ros::Time(0);
    try
    {
        local_update_goal_pose_ = docking_tf_buffer.transform(global_goal_pose_, path_frame_, ros::Duration(1));
    }
    catch (tf2::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    geometry_msgs::PoseWithCovarianceStamped goal_pose_pub;
    goal_pose_pub.header = local_update_goal_pose_.header;
    goal_pose_pub.pose.pose = local_update_goal_pose_.pose;
    pub_local_goal_pose_.publish(goal_pose_pub);
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
            ros::param::get("/docking_planner/x_tolerance", x_tolerance_);
            ros::param::get("/docking_planner/y_tolerance", y_tolerance_);
            ros::param::get("/docking_planner/angle_tolerance", angle_tolerance_);
        
        }
        else                        // tolerance for final docking
        {
            ros::param::get("/docking_planner/final_x_tolerance", x_tolerance_);
            ros::param::get("/docking_planner/final_y_tolerance", y_tolerance_);
            ros::param::get("/docking_planner/final_angle_tolerance", angle_tolerance_);
        }
    }
    else                            // returning mode
    {
        // Relax the tolerance with returning motion
        ros::param::get("/docking_planner/x_tolerance", x_tolerance_);
        ros::param::get("/docking_planner/y_tolerance", y_tolerance_);
        ros::param::get("/docking_planner/angle_tolerance", angle_tolerance_);
    }

    if (error_sq <= distance_tolerance_) 
    {
        check_inside_goal_range_ = true;
        if (abs(error_yaw) < angle_tolerance_ && abs(error_x) < x_tolerance_ && abs(error_y) < y_tolerance_)
        {
            check_inside_goal_range_ = false;
            goal_reach_ = true;
            ROS_INFO("Docking Error [x , y, yaw]: %f (m), %f (m), %f (rad)", error_x, error_y, error_yaw);
            return;
        } 
    }

    if (check_inside_goal_range_)
    {
        if (!approach_done_)
            if (error_sq > distance_tolerance_) count_outside_goal_range_++;
        else
            if (error_x < 0) count_outside_goal_range_++;
        if (count_outside_goal_range_ > 10)      // Prevent jumping in and out goal range
        {
            count_outside_goal_range_ = 0;
            ROS_WARN("Failed!!! !");
            ROS_INFO("Docking Error [x , y, yaw]: %f (m), %f (m), %f (rad)", error_x, error_y, error_yaw);
            check_inside_goal_range_ = false;
            goal_failed_ = true;
            return; 
        }
    }
}

/***** RESET PALLET DOCKING ******/
void DockingManager::initDocking()
{
    current_pallet_docking_state_ = IDLE;
    docking_state.data = "IDLE";
    pub_docking_state.publish(docking_state);
    
    start_pallet_docking_ = false;
    start_returning_ = false;

    pose_setup_ = false;

    norm_goal_frame_ = false;
    
    docking_done.data = false;          
    approaching_done.data = false;
    docking_failed.data = false;

    docking_control.resetController();
    quintic_planner.resetPlanner();

    check_inside_goal_range_ = false;
    count_outside_goal_range_ = 0;

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
    docking_control.resetController();
    quintic_planner.resetPlanner();
    goal_setup_ = false;
    count_outside_goal_range_ = 0;
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
    if (!approach_done_)
    {
        // goal_distance = dis_approach_offset_;
        current_pallet_docking_state_ = SET_GOAL;  
    }
    else 
    {
        approaching_done.data = true;
        current_pallet_docking_state_ = DOCKING;
    }
}

////// Docking State /////
void DockingManager::dockingState()
{
    docking_state.data = "DOCKING";
    current_pallet_docking_state_ = SET_GOAL;
}

////// Set goal State /////
/***** SETUP GOAL FOR DOCKING *****/
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

void DockingManager::goalSetup()
{   
    // Normalize the frame into a consistent one
    if (!norm_goal_frame_)
    {
        approaching_goal_.header.stamp = ros::Time(0);
        docking_goal_.header.stamp = ros::Time(0);
        try
        {
            tf_approaching_goal_ = docking_tf_buffer.transform(approaching_goal_, global_frame_, ros::Duration(1));
            tf_docking_goal_ = docking_tf_buffer.transform(docking_goal_, global_frame_, ros::Duration(1));
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

bool DockingManager::ExtendApproachingDistance()
{
    if (check_approaching_goal_distance_ < approaching_min_dis_ 
                        && !approach_done_ && !returning_mode_.data)  // Move back to increase the distance
    {   
        // goalSetup(goal_distance, pallet_pose_);
        updateGoal();
        /* TODO: Update the calculation for approaching distance
        */
       double abs_dis = abs(local_update_goal_pose_.pose.position.x)*tf::getYaw(local_update_goal_pose_.pose.orientation);
        if (abs_dis < approaching_min_dis_)
        {
            ROS_INFO_ONCE("MOVE BACKWARD. THE MOVEMENT DISTANCE IS TOO SHORT !!!");
            geometry_msgs::Twist cmd_fw;
            cmd_fw.linear.x = -0.2;
            cmd_fw.angular.z = 0.0;
            pub_cmd_vel.publish(cmd_fw);
            return false;
        }
        else
        {
            ROS_INFO("Perpendicular distance to the approaching goal: %f", abs_dis);
            return true;
        }
    }
    return true;
}

void DockingManager::setGoalState()
{
    docking_state.data = "SET_GOAL";
    if (!pose_setup_) return;
    if (!goal_setup_)
    {
        goalSetup();
        double yaw_tm;
        // yaw_tm = tf::getYaw(local_update_goal_pose_.pose.orientation);
        // check_approaching_goal_distance_ = abs(local_update_goal_pose_.pose.position.x*cos(yaw_tm));
        goal_setup_ = true;
    } 
    
    if (goal_setup_ && ExtendApproachingDistance())
    {
        current_pallet_docking_state_ = GEN_PATH_AND_PUB_CONTROL;
        goal_setup_ = false;
    } 
}

////// Gen Path State /////
void DockingManager::quinticPlannerSetup()
{
    if (returning_mode_.data)
    {
        if (quintic_planner.starting_vel_ > 0) quintic_planner.starting_vel_ = -quintic_planner.starting_vel_;
        if (quintic_planner.stopping_vel_ > 0) quintic_planner.stopping_vel_ = -quintic_planner.stopping_vel_;
    }
    else
    {
        if (quintic_planner.starting_vel_ < 0) quintic_planner.starting_vel_ = -quintic_planner.starting_vel_;
        if (quintic_planner.stopping_vel_ < 0) quintic_planner.stopping_vel_ = -quintic_planner.stopping_vel_;
    }
    quintic_planner.setParams(0.0, 0.0, 0.0, quintic_planner.starting_vel_, quintic_planner.starting_acc_,
            local_update_goal_pose_.pose.position.x, local_update_goal_pose_.pose.position.y, tf::getYaw(local_update_goal_pose_.pose.orientation),
            quintic_planner.stopping_vel_, quintic_planner.stopping_acc_);
    if (!quintic_planner.path_avai_)
        quintic_planner.genPath();
    quintic_planner.visualize(global_goal_pose_);
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
    failure_code_ = 3;
    current_pallet_docking_state_ = FAILURE;
    ROS_INFO("Goal failed !!!");
}

void DockingManager::genPathAndPubControlState()
{
    docking_state.data = "GEN_PATH_AND_PUB_CONTROL";
    updateGoal();
    if (!goal_avai_) return;
    checkGoalReach();
    quinticPlannerSetup();
    if (!quintic_planner.path_feasible_)
    {
        ROS_WARN("PATH IS NOT FEASIBLE");
        failure_code_ = 2;
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
    if (quintic_planner.path_avai_ && !docking_control.controller_on_.data)
    {
        docking_control.controller_on_.data = true;
        // pub_controller_on_.publish(controller_on_);
    }
    docking_control.controllerCal();
    if (docking_control.invalid_control_signal_)
    {
        docking_state.data = "FAILURE";
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
    docking_done.data = true;
}

////// Stop State /////
void DockingManager::stopState()
{
    docking_state.data = "STOP";
    resetPlanAndControl();
    ros::Duration(1.0).sleep();
    if (docking_done_) current_pallet_docking_state_ = END;
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
    ROS_INFO_ONCE("FAILURE: %s. ABORTED!", failure_map_[failure_code_].c_str());
    docking_state.data = "FAILURE: " + failure_map_[failure_code_];
    docking_failed.data = true;
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

geometry_msgs::Twist DockingManager::getCmdVel()
{
    return docking_control.cmd_vel_;
}


// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "docking_manager");
//     ros::NodeHandle n ("~");
//     ros::Rate loop_rate(20);

//     DockingManager docking_magager(n);
//     while(ros::ok())
//     {
//         docking_magager.dockingFSM();
//         ros::spinOnce();
//         loop_rate.sleep();
//     }
  
//   return 0;
// }
