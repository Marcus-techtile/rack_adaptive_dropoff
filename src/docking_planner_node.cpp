#include "docking_planner_node.h"

DockingManager::DockingManager(ros::NodeHandle &nh): nh_(nh), quintic_planner(nh)
{
    /* Get Param */
    nh.param<std::string>("path_frame", path_frame_, "base_link_p");
    nh.param<std::string>("docking_area", docking_area_, "INSIDE");
    nh.param<double>("dis_approach_offset", dis_approach_offset_, 1.5);
    nh.param<double>("dis_docking_offset", dis_docking_offset_, 0.5);

    nh.param<double>("distance_tolerance", distance_tolerance_, 0.08);
    nh.param<double>("angle_tolerance", angle_tolerance_, 5*180/M_PI);
    nh.param<double>("x_tolerance", x_tolerance_, 0.05);
    nh.param<double>("y_tolerance", y_tolerance_, 0.04);

    nh.param<double>("detection_timeout", detection_timeout_, 5.0);

    nh.param<double>("fake_goal_x", fake_goal_x_, 2.0);
    nh.param<double>("fake_goal_y", fake_goal_y_, 0.1);
    nh.param<double>("fake_goal_yaw", fake_goal_yaw_, 0.1);
    nh.param<bool>("use_fake_goal", use_fake_goal_, false);
    

    /* Publisher */
    pub_docking_state = nh_.advertise<std_msgs::String>("/pallet_docking/docking_state", 1);
    pub_docking_done = nh_.advertise<std_msgs::Bool>("/pallet_docking/pallet_docking_done", 1);
    pub_approaching_done = nh_.advertise<std_msgs::Bool>("/pallet_docking/pallet_approaching_done", 1);
    pub_cmd_vel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_controller_on_ = nh_.advertise<std_msgs::Bool>("/pallet_docking/controller_turn_on", 1);
    pub_goal_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pallet_docking/goal_pose", 1);
    pub_fake_goal_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/pallet_docking/fake_goal_pose", 1);

    /* Subscriber */
    sub_cmd_vel = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &DockingManager::cmdCallback, this);
    sub_pallet_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/pallet_detection_relay/pallet_pose", 1, &DockingManager::palletPoseCallback, this);

    sub_docking_server_result_ = nh.subscribe<pallet_dock_msgs::PalletDockingActionResult>("/pallet_dock_action_server/pallet_docking/result", 1, &DockingManager::dockingServerResultCallback, this);

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

void DockingManager::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_vel_sub = *msg;
}

void DockingManager::palletPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (get_pallet_pose_ && !use_fake_goal_)
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

void DockingManager::dockingServerResultCallback(const pallet_dock_msgs::PalletDockingActionResult::ConstPtr& msg)
{
    docking_server_result_ = *msg;
    if (docking_server_result_.result.code >= 2)
    {
        initDocking();
        ROS_WARN("Failure Result from Docking action server! Stop the FSM!");
    }
}

bool DockingManager::dockingServiceCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    if(req.data)
    {
        initDocking();
        start_docking_FSM = true;
        res.success = true;
        res.message = "Start docking!";
    }
    else
    {
        initDocking();
        start_docking_FSM = false;
        res.success = true;
        res.message = "Stop docking!";
    }
    return true;
}

void DockingManager::goalSetup(double distance_pallet, geometry_msgs::PoseStamped pallet_pose)
{
    pallet_pose.header.stamp = ros::Time(0);
    geometry_msgs::PoseStamped local_pallet_pose;
    
    try
    {
        local_pallet_pose = docking_tf_buffer.transform(pallet_pose, path_frame_, ros::Duration(1));
    }
    catch (tf2::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    
    double local_goal_y;

    if (local_pallet_pose.pose.position.x >= 0)
    {
            /* Calculate the goal in local frame */
        double r_tmp, p_tmp, y_tmp;
        quaternionToRPY(local_pallet_pose.pose.orientation, r_tmp, p_tmp, y_tmp);
        // ROS_INFO("Relative yaw: %f", y_tmp*180/M_PI);

        local_static_goal_pose_.pose.position.x = local_pallet_pose.pose.position.x - 
                                                        distance_pallet*cos(y_tmp);
        
        local_goal_y = local_pallet_pose.pose.position.y - 
                                                        distance_pallet*sin(y_tmp);
    }
    else
    {
        local_static_goal_pose_.pose.position.x = local_pallet_pose.pose.position.x;
        local_goal_y = local_pallet_pose.pose.position.y;
    }

    if (approach_done_) local_goal_y = 0;       // Need to test more
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    
    if (docking_area_ == "INSIDE") 
    {
        if (abs(local_goal_y) > 0.2)
        {
            goal_sideshift_ = abs(local_goal_y)/local_goal_y * 0.2;
            local_static_goal_pose_.pose.position.y = local_goal_y - goal_sideshift_;
        }
        else
        {
            local_static_goal_pose_.pose.position.y = 0;
            goal_sideshift_ = local_goal_y;
        }
    } 
    else local_static_goal_pose_.pose.position.y = local_goal_y; 
            
    local_static_goal_pose_.pose.orientation = local_pallet_pose.pose.orientation;
    
    /* Convert the goal to global frame */
    local_static_goal_pose_.header.stamp = ros::Time(0);;
    local_static_goal_pose_.header.frame_id = path_frame_;
    std::string global_frame = pallet_pose.header.frame_id;

    try
    {
        global_goal_pose_ = docking_tf_buffer.transform(local_static_goal_pose_, global_frame, ros::Duration(1));
    }
    catch (tf::LookupException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    goal_setup_ = true;  
}

void DockingManager::updateGoal()
{
    /* Transform goal pose to path_frame */
    global_goal_pose_.header.stamp = ros::Time(0);

    try
    {
        local_goal_pose_ = docking_tf_buffer.transform(global_goal_pose_, path_frame_, ros::Duration(1));
    }
    catch (tf::LookupException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    double r_tmp, p_tmp, y_tmp;
    quaternionToRPY(local_goal_pose_.pose.orientation, r_tmp, p_tmp, y_tmp);

    goal_pose_.x = local_goal_pose_.pose.position.x;
    goal_pose_.y = local_goal_pose_.pose.position.y;
    double local_goal_yaw = y_tmp;
    goal_pose_.z = local_goal_yaw;

    local_goal_pose_.pose.position.z = 0;

    geometry_msgs::PoseWithCovarianceStamped goal_pose_pub;
    goal_pose_pub.header = local_goal_pose_.header;
    goal_pose_pub.pose.pose = local_goal_pose_.pose;
    pub_goal_pose_.publish(goal_pose_pub);
    goal_avai_ = true; 

    // std::cout << "goal pose x, y, yaw:" << goal_pose_.x << "  " << goal_pose_.y << "  "
    //                                     << goal_pose_.z << std::endl;
}

void DockingManager::checkGoalReach()
{
    double error_x = goal_pose_.x;
    double error_y = goal_pose_.y;
    double error_yaw = goal_pose_.z;
    double error_sq = sqrt(error_x*error_x + error_y*error_y);
    if (approach_done_)
    {
        angle_tolerance_ = 0.1;     // For temporary to make the pocket docking more stable
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
        if (error_sq > distance_tolerance_)
        {
            ROS_WARN("Failed!!! !");
            ROS_INFO("Docking Error [x , y, yaw]: %f (m), %f (m), %f (rad)", error_x, error_y, error_yaw);
            check_inside_goal_range_ = false;
            goal_failed_ = true;
            return; 
        }
    }
}

void DockingManager::initDocking()
{
    current_state_ = IDLE;
    docking_state.data = "IDLE";
    pub_docking_state.publish(docking_state);
    
    start_docking_FSM = false;
    
    docking_done.data = false;          
    approaching_done.data = false;

    controller_on_.data = false;
    pub_controller_on_.publish(controller_on_);

    quintic_planner.resetPlanner();

    // reset the transition state
    get_pallet_pose_ = false;
    pallet_pose_avai_ = false;
    approach_done_ = false;     // transition from APPROACHING to DOCKING, to SET_GOAL
    docking_done_ = false;       // transition from DOCKING to SET_GOAL
    goal_setup_ = false;        // transition from SET_GOAL to UPDATE_GOAL
    goal_avai_ = false;         // transition from UPDATE_GOAL to GEN_PATH
    goal_reach_ = false;         // transition from GEN_PATH to STOP
    goal_failed_ = false;        // transition from GEN_PATH to RECOVER

    count_path_gen_fail_ = 0;
    count_goal_failed_ = 0;
}

void DockingManager::resetPlanAndControl()
{

}

void DockingManager::dockingFSM()
{
    if (!start_docking_FSM) return;

    if (old_docking_state.data != docking_state.data)
    {
        pub_docking_state.publish(docking_state);
        ROS_INFO("_DOCKING STATE: %s", docking_state.data.c_str());
    }
    old_docking_state = docking_state;
    
    geometry_msgs::PoseStamped fake_goal;
    switch(current_state_)
    {   case IDLE:
            docking_state.data = "IDLE";
            if (start_docking_FSM) current_state_ = GET_PALLET_POSE;
            else 
            break;
        case GET_PALLET_POSE:
            docking_state.data = "GET_PALLET_POSE";

            if (!get_pallet_pose_) 
            {
                get_pallet_pose_ = true;
                starting_detection_time_ = ros::Time::now();

                if (use_fake_goal_)
                {
                    fake_goal.header.frame_id = path_frame_;
                    fake_goal.header.stamp = ros::Time::now() + ros::Duration(5);
                    fake_goal.pose.position.x = fake_goal_x_;
                    fake_goal.pose.position.y = fake_goal_y_;
                    fake_goal.pose.orientation = rpyToQuaternion(0.0, 0.0, fake_goal_yaw_);
                    try
                    {
                        pallet_pose_ = pose_tf_buffer.transform(fake_goal, "odom", ros::Duration(5));
                    }
                    catch (tf2::TransformException ex)
                    {
                        ROS_ERROR("%s",ex.what());
                    }
                    ///// TODO: add general transform for the path frame and the pallet pose frame. Will not depend on the out frame from the pallet detection
                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    pub_fake_goal_pose_.publish(pallet_pose_);
                    pallet_pose_avai_ = true; //fake goal test
                }
            }
            
            if (pallet_pose_avai_)
            {
                ///// TODO: check 2 pallet poses from approach and dock to determine the pose for dock
                ////////////
                ////////////
                current_state_ = APPROACHING;
                pallet_pose_avai_ = false;
                get_pallet_pose_ = false;  // get pallet pose
                break;
            }
            else
            {
                if ((ros::Time::now() - starting_detection_time_).toSec() > detection_timeout_)
                {
                    failure_code_ = 1;
                    current_state_ = FAILURE;
                    break;
                }
                break;
            } 

        case APPROACHING:
            docking_state.data = "APPROACHING";
            if (!approach_done_)
            {
                goal_distance = dis_approach_offset_;
                current_state_ = SET_GOAL;
            }
            else 
            {
                approaching_done.data = true;
                if (docking_area_ == "INSIDE")
                    current_state_ = SIDESHIFT_CONTROL;
                else current_state_ = DOCKING;
            }
            break;
        case DOCKING:
            docking_state.data = "DOCKING";
            goal_distance = dis_docking_offset_;
            current_state_ = SET_GOAL;
            break;
        case SET_GOAL:
            docking_state.data = "SET_GOAL";
            goalSetup(goal_distance, pallet_pose_);
            if (local_static_goal_pose_.pose.position.x < 0) move_reverse_ = true;
            else move_reverse_ = false;
            if (goal_setup_) current_state_ = GEN_PATH_AND_PUB_CONTROL;
            break;
        case GEN_PATH_AND_PUB_CONTROL:
            docking_state.data = "GEN_PATH_AND_PUB_CONTROL";
            updateGoal();
            if (!goal_avai_) break;
            checkGoalReach();
            
            // if(move_reverse_)
            // {
            //     if (quintic_planner.starting_vel_ > 0) quintic_planner.starting_vel_ = -quintic_planner.starting_vel_;
            //     if (quintic_planner.stopping_vel_ > 0) quintic_planner.stopping_vel_ = -quintic_planner.stopping_vel_;
            // }
            quintic_planner.setParams(0.0, 0.0, 0.0, quintic_planner.starting_vel_, quintic_planner.starting_acc_,
                                    goal_pose_.x, goal_pose_.y, goal_pose_.z,
                                    quintic_planner.stopping_vel_, quintic_planner.stopping_acc_);
            if (!quintic_planner.path_avai_)
                quintic_planner.genPath();
            quintic_planner.visualize(pallet_pose_);
            if (!quintic_planner.path_feasible_)
            {
                ROS_WARN("PATH IS NOT FEASIBLE. RECOVERY %d times", count_path_gen_fail_+1);
                count_path_gen_fail_++; 
                if (count_path_gen_fail_ == 3) 
                {
                    failure_code_ = 2;
                    current_state_ = FAILURE;
                    break;
                }
                current_state_ = RECOVER;
            } 
            else    // path feasible
            {   
                // Start the controller
                if (!controller_on_.data)
                {
                    controller_on_.data = true;
                    pub_controller_on_.publish(controller_on_);
                }
                count_path_gen_fail_ = 0;
            } 
            if (goal_reach_)
            {
                ROS_INFO("Goal reach !!!");
                if (!approach_done_) approach_done_ = true;
                else docking_done_ = true;
                current_state_ = STOP;
                goal_reach_ = false;
                quintic_planner.resetPlanner();
                count_goal_failed_ = 0;
            } 
            if (goal_failed_)
            {
                count_goal_failed_++;
                if (count_goal_failed_ == 3)
                {
                    failure_code_ = 3;
                    current_state_ = FAILURE;
                    break;
                }
                ROS_INFO("Goal failed. Try to recover !!!");
                quintic_planner.resetPlanner();
                current_state_ = RECOVER;
            } 
            break;
        case STOP:
            docking_state.data = "STOP";
            // Stop the controller
            controller_on_.data = false;
            pub_controller_on_.publish(controller_on_);
            if (docking_done_) current_state_ = END;
            else 
            {
                // docking_state = quintic_planner.GET_PALLET_POSE;
                current_state_ = APPROACHING;
                ros::Duration(1.0).sleep();
            }
            break;
        case RECOVER:
            docking_state.data = "RECOVER";
            // Stop the controller
            controller_on_.data = false;
            pub_controller_on_.publish(controller_on_);
            quintic_planner.resetPlanner();
            goal_failed_ = false;

            if (move_reverse_) cmd_fw.linear.x = 0.0;
            else cmd_fw.linear.x = -0.0;
            starting_time_recover = ros::Time::now().toSec();
            if (count_cmd_back_vel <= 100)
            {
                count_cmd_back_vel++;
                pub_cmd_vel.publish(cmd_fw);
            } 
            else 
            {
                count_cmd_back_vel = 0;
                cmd_fw.linear.x = 0;
                pub_cmd_vel.publish(cmd_fw);
                // current_state_ = GEN_PATH_AND_PUB_CONTROL;
                current_state_ = END;
            }
            break;
        case END:
            docking_state.data = "END";
            docking_done.data = true;
            break;
        case FAILURE:
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