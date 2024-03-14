#include "docking_planner_node.h"

DockingManager::DockingManager(ros::NodeHandle &nh): nh_(nh), quintic_planner(nh)
{
    /* Get Param */
    nh_.param<std::string>("global_frame", global_frame_, "odom");
    nh_.param<std::string>("path_frame", path_frame_, "base_link_p");

    /* Goal params */
    nh_.param<double>("approaching_min_dis", approaching_min_dis_, 1.2);
    nh_.param<double>("dis_approach_offset", dis_approach_offset_, 1.5);
    nh_.param<double>("dis_docking_offset", dis_docking_offset_, 0.5);
    nh_.param<double>("moveback_straight_distance", moveback_straight_distance_, 1.0);

    /* Tolerance params */
    nh_.param<double>("distance_tolerance", distance_tolerance_, 0.08);
    nh_.param<double>("angle_tolerance", angle_tolerance_, 5*180/M_PI);
    nh_.param<double>("final_angle_tolerance", final_angle_tolerance_, 1*180/M_PI);
    nh_.param<double>("x_tolerance", x_tolerance_, 0.05);
    nh_.param<double>("final_x_tolerance", final_x_tolerance_, 0.05);
    nh_.param<double>("y_tolerance", y_tolerance_, 0.04);
    nh_.param<double>("final_y_tolerance", final_y_tolerance_, 0.04);
    
    /* Publisher */
    pub_docking_state = nh_.advertise<std_msgs::String>("/pallet_docking/docking_state", 1);
    pub_docking_done = nh_.advertise<std_msgs::Bool>("/pallet_docking/pallet_docking_done", 1);
    pub_approaching_done = nh_.advertise<std_msgs::Bool>("/pallet_docking/pallet_approaching_done", 1);
    pub_cmd_vel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_controller_on_ = nh_.advertise<std_msgs::Bool>("/pallet_docking/controller_turn_on", 1);
    pub_goal_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pallet_docking/goal_pose", 1);
    pub_global_goal_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/pallet_docking/global_goal_pose", 1);
    pub_docking_error_ = nh_.advertise<geometry_msgs::Vector3>("/pallet_docking/docking_error", 1);

    /* Subscriber */
    sub_pallet_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/pallet_detection_relay/pallet_pose", 1, &DockingManager::palletPoseCallback, this);

    sub_docking_server_result_ = nh.subscribe<pallet_dock_msgs::PalletDockingActionResult>("/pallet_dock_action_server/pallet_docking/result", 1, &DockingManager::dockingServerResultCallback, this);
    sub_docking_server_goal_ = nh_.subscribe<pallet_dock_msgs::PalletDockingActionGoal>("/pallet_dock_action_server/pallet_docking/goal", 1, &DockingManager::dockingServerGoalCallback, this);

    /* Service*/
    service_ = nh_.advertiseService("/pallet_docking_service", &DockingManager::dockingServiceCb, this);
    
    /*Define failure cases */
    failure_map_[0] = "CANNOT_CALL_DETECTION_SERVICE";
    failure_map_[1] = "CANNOT_DETECT_PALLET";
    failure_map_[2] = "PATH_IS_NOT_FEASIBLE";
    failure_map_[3] = "BAD_DOCKING_ACCURACY";

    // Init the Docking
    initDocking();
}

DockingManager::~ DockingManager(){}

/***** PALLET POSE CALLBACK ******/
void DockingManager::palletPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (get_pallet_pose_)
    {
        pallet_pose_ = *msg;
        double r, p, yaw;
        quaternionToRPY(pallet_pose_.pose.orientation, r, p, yaw);
        yaw = yaw - M_PI;
        pallet_pose_.pose.orientation = rpyToQuaternion(r, p, yaw);
        if (!pallet_pose_.header.frame_id.empty())
        {
            get_pallet_pose_ = false;
            pallet_pose_avai_ = true;
        }
        else ROS_INFO ("Pallet pose is low quality !!!");
    }
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
    pub_global_goal_pose_.publish(pallet_pose_);

    dis_approach_offset_ = docking_server_goal_.goal.approaching_distance;
    dis_docking_offset_ = docking_server_goal_.goal.pallet_depth_offset;
    if (dis_docking_offset_ > 0)
    {
        ROS_WARN("Docking offset is positive. This param should be negative to ensure the docking depth. Revert it to negative!");
        dis_docking_offset_ = -dis_docking_offset_;
    }
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

/***** SETUP GOAL FOR DOCKING *****/
void DockingManager::goalSetup(double distance_pallet, geometry_msgs::PoseStamped pallet_pose)
{
    pallet_pose.header.stamp = ros::Time(0);
    geometry_msgs::PoseStamped local_pallet_pose;

    if (!global_pallet_pose_setup_)        // Convert pallet pose to global_frame
    {
        try
        {
            global_pallet_pose_ = docking_tf_buffer.transform(pallet_pose, global_frame_, ros::Duration(1));
        }
        catch (tf2::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        global_pallet_pose_setup_ = true;
    }

    global_pallet_pose_.header.stamp = ros::Time(0);  
    try                                     // Convert pallet pose to local_frame
    {
        local_pallet_pose = docking_tf_buffer.transform(global_pallet_pose_, path_frame_, ros::Duration(1));
    }
    catch (tf2::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    if (docking_mode_ == 0)                 // pallet docking mode
    {
            /* Calculate the goal in local frame */
        double r_tmp, p_tmp, y_tmp;
        quaternionToRPY(local_pallet_pose.pose.orientation, r_tmp, p_tmp, y_tmp);
        // ROS_INFO("Relative yaw: %f", y_tmp*180/M_PI);

        local_pallet_pose.pose.position.y  = local_pallet_pose.pose.position.y; //offset for ifm cam

        local_static_goal_pose_.pose.position.x = local_pallet_pose.pose.position.x - 
                                                        distance_pallet*cos(y_tmp);
        
        local_static_goal_pose_.pose.position.y = local_pallet_pose.pose.position.y - 
                                                        distance_pallet*sin(y_tmp);
        local_static_goal_pose_.pose.orientation = local_pallet_pose.pose.orientation;
    }
    else                                    // returning mode
    {
        if (!approach_done_)
        {
            local_static_goal_pose_.pose.position.x = -moveback_straight_distance_;
            local_static_goal_pose_.pose.position.y = 0.0;
            local_static_goal_pose_.pose.orientation = rpyToQuaternion(0.0, 0.0, 0.0);
        }
        else
        {
            local_static_goal_pose_.pose.position.x = local_pallet_pose.pose.position.x;
            local_static_goal_pose_.pose.position.y = local_pallet_pose.pose.position.y;
            local_static_goal_pose_.pose.orientation = local_pallet_pose.pose.orientation;
        }
    } 
            
    /* Convert the goal to global frame */
    local_static_goal_pose_.header.stamp = ros::Time(0);;
    local_static_goal_pose_.header.frame_id = path_frame_;

    try
    {
        global_goal_pose_ = docking_tf_buffer.transform(local_static_goal_pose_, global_frame_, ros::Duration(1));
    }
    catch (tf2::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    // pub_global_goal_pose_.publish(global_goal_pose_);
    goal_setup_ = true;  
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

    double r_tmp, p_tmp, y_tmp;
    quaternionToRPY(local_update_goal_pose_.pose.orientation, r_tmp, p_tmp, y_tmp);

    goal_pose_.x = local_update_goal_pose_.pose.position.x;
    goal_pose_.y = local_update_goal_pose_.pose.position.y;
    goal_pose_.z = y_tmp;

    local_update_goal_pose_.pose.position.z = 0;

    geometry_msgs::PoseWithCovarianceStamped goal_pose_pub;
    goal_pose_pub.header = local_update_goal_pose_.header;
    goal_pose_pub.pose.pose = local_update_goal_pose_.pose;
    pub_goal_pose_.publish(goal_pose_pub);
    goal_avai_ = true; 
}

void DockingManager::checkGoalReach()
{
    double error_x = goal_pose_.x;
    double error_y = goal_pose_.y;
    double error_yaw = goal_pose_.z;
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
        if (error_sq > distance_tolerance_) count_outside_goal_range_++;
        if (count_outside_goal_range_ > 20)      // Prevent jumping in and out goal range
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

    global_pallet_pose_setup_ = false;
    
    docking_done.data = false;          
    approaching_done.data = false;

    controller_on_.data = false;
    pub_controller_on_.publish(controller_on_);

    quintic_planner.resetPlanner();

    check_inside_goal_range_ = false;
    count_outside_goal_range_ = 0;

    // reset the transition state
    get_pallet_pose_ = false;
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
    controller_on_.data = false;
    pub_controller_on_.publish(controller_on_);
    quintic_planner.resetPlanner();
    goal_setup_ = false;
    count_outside_goal_range_ = 0;
}

/***** DOCKING FSM ******/
void DockingManager::dockingFSM()
{
    if (!start_docking_FSM) return;
    
    if (!start_pallet_docking_ && !start_returning_) {}
    else if (start_pallet_docking_ && start_returning_)
    {
        ROS_WARN("BAD DOCKING STATE! Both pallet docking and returning is turned on!");
        return;
    }
    else if (start_pallet_docking_ && !start_returning_) returning_mode_.data = false;        //docking
    else returning_mode_.data = true;             // returning. move back

    if (old_docking_state.data != docking_state.data)
    {
        pub_docking_state.publish(docking_state);
        ROS_INFO("_DOCKING STATE: %s", docking_state.data.c_str());
    }
    old_docking_state = docking_state;
    
    // FM transition
    switch(current_pallet_docking_state_)
    {   case IDLE:
            docking_state.data = "IDLE";
            if (start_pallet_docking_ || start_returning_) current_pallet_docking_state_ = GET_PALLET_POSE;
            else  break;
            break;
        case GET_PALLET_POSE:
            docking_state.data = "GET_PALLET_POSE";
            if (!get_pallet_pose_) get_pallet_pose_ = true;             
            if (pallet_pose_avai_)
            {
                current_pallet_docking_state_ = APPROACHING;
                pallet_pose_avai_ = false;
                get_pallet_pose_ = false;  
                break;
            }
            break;
        case APPROACHING:
            docking_state.data = "APPROACHING";
            if (!approach_done_)
            {
                goal_distance = dis_approach_offset_;
                current_pallet_docking_state_ = SET_GOAL;  
            }
            else 
            {
                approaching_done.data = true;
                current_pallet_docking_state_ = DOCKING;
            }
            break;
        case DOCKING:
            docking_state.data = "DOCKING";
            goal_distance = dis_docking_offset_;
            current_pallet_docking_state_ = SET_GOAL;
            break;
        case SET_GOAL:
            docking_state.data = "SET_GOAL";
            if (!goal_setup_)
            {
                goalSetup(goal_distance, pallet_pose_);
                double r_tm, p_tm, yaw_tm;
                quaternionToRPY(local_static_goal_pose_.pose.orientation, r_tm, p_tm, yaw_tm);
                check_approaching_goal_distance_ = abs(local_static_goal_pose_.pose.position.x*cos(yaw_tm));
            } 

            if (check_approaching_goal_distance_ < approaching_min_dis_ 
                                && !approach_done_ && !returning_mode_.data)  // Move back to increase the distance
            {   
                // goalSetup(goal_distance, pallet_pose_);
                updateGoal();
                double r_tm, p_tm, yaw_tm;
                quaternionToRPY(local_update_goal_pose_.pose.orientation, r_tm, p_tm, yaw_tm);
                /* TODO: Update the calculation for approaching distance
                */
                if (abs(local_update_goal_pose_.pose.position.x*cos(yaw_tm)) < approaching_min_dis_)
                {
                    ROS_INFO_ONCE("MOVE BACKWARD. THE MOVEMENT DISTANCE IS TOO SHORT !!!");
                    geometry_msgs::Twist cmd_fw;
                    cmd_fw.linear.x = -0.2;
                    cmd_fw.angular.z = 0.0;
                    pub_cmd_vel.publish(cmd_fw);
                    break;
                }
                else
                {
                    ROS_INFO("Perpendicular distance to the approaching goal: %f", abs(local_update_goal_pose_.pose.position.x*cos(yaw_tm)));
                }
            }
            if (goal_setup_)
            {
                current_pallet_docking_state_ = GEN_PATH_AND_PUB_CONTROL;
                goal_setup_ = false;
            } 
            break;
        case GEN_PATH_AND_PUB_CONTROL:
            docking_state.data = "GEN_PATH_AND_PUB_CONTROL";
            updateGoal();
            if (!goal_avai_) break;
            checkGoalReach();
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
                                    goal_pose_.x, goal_pose_.y, goal_pose_.z,
                                    quintic_planner.stopping_vel_, quintic_planner.stopping_acc_);
            if (!quintic_planner.path_avai_)
                quintic_planner.genPath();
            quintic_planner.visualize(global_pallet_pose_);
            if (!quintic_planner.path_feasible_)
            {
                ROS_WARN("PATH IS NOT FEASIBLE");
                failure_code_ = 2;
                current_pallet_docking_state_ = FAILURE;
                break;
            } 
            else    // path feasible
            {   
                // Start the controller
                if (!controller_on_.data)
                {
                    controller_on_.data = true;
                    pub_controller_on_.publish(controller_on_);
                }
            } 
            if (goal_reach_)
            {
                resetPlanAndControl();
                goal_reach_ = false;
                ROS_INFO("Goal reach !!!");
                if (!approach_done_) approach_done_ = true;
                else docking_done_ = true;
                current_pallet_docking_state_ = STOP;
                quintic_planner.resetPlanner();
                break;
            } 
            if (goal_failed_)
            {
                failure_code_ = 3;
                current_pallet_docking_state_ = FAILURE;
                ROS_INFO("Goal failed !!!");
                quintic_planner.resetPlanner();
                break;
            } 
            break;
        case STOP:
            docking_state.data = "STOP";
            // Stop the controller
            resetPlanAndControl();
            ros::Duration(1.0).sleep();
            if (docking_done_) current_pallet_docking_state_ = END;
            else 
            {
                // docking_state = quintic_planner.GET_PALLET_POSE;
                current_pallet_docking_state_ = APPROACHING;
                ros::Duration(1.0).sleep();
            }
            break;
        case RECOVER:
            docking_state.data = "RECOVER";
            // Stop the controller
            // resetPlanAndControl();
            
            break;
        case END:
            docking_state.data = "END";
            docking_done.data = true;
            break;
        case FAILURE:
            resetPlanAndControl();
            ROS_INFO_ONCE("FAILURE: %s. ABORTED!", failure_map_[failure_code_].c_str());
            docking_state.data = "FAILURE: " + failure_map_[failure_code_];
            break;
        default:
            break; 
    } 
    pub_docking_done.publish(docking_done); 
    pub_approaching_done.publish(approaching_done);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "docking_manager");
    ros::NodeHandle n ("~");
    ros::Rate loop_rate(20);

    DockingManager docking_magager(n);
    while(ros::ok())
    {
        docking_magager.dockingFSM();
        ros::spinOnce();
        loop_rate.sleep();
    }
  
  return 0;
}
