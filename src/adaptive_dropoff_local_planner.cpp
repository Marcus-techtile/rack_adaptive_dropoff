#include "adaptive_dropoff_local_planner.h"

using namespace techtile;

/*********** Constructor ***********/
AdaptiveDockingLocalPlanner::AdaptiveDockingLocalPlanner(ros::NodeHandle &nh) : nh_(nh)
{
    ROS_INFO("Adaptive Local Planner Constructor");
}

AdaptiveDockingLocalPlanner::~AdaptiveDockingLocalPlanner(){}

void AdaptiveDockingLocalPlanner::initialize(tf2_ros::Buffer &tf, double sec)
{
    docking_manager_ = std::make_unique<DockingManager>(nh_, tf, sec);
    docking_manager_->config();
    docking_manager_->initDocking();
    ROS_INFO("Init Adaptive Docking Local Planner");  
}

/************ Set Frame ************/
// Set Local Frame. Default: "base_link_p"
void AdaptiveDockingLocalPlanner::setLocalFrame(std::string local_frame)
{
    docking_manager_->setLocalFrame(local_frame);
}

// Set Global Frame. Default: initialize from the param "/move_base_flex/AD/global_frame_id"
void AdaptiveDockingLocalPlanner::setGlobalFrame(std::string global_frame)
{
    docking_manager_->setGLobalFrame(global_frame);
}

/************ Set Plan ************/
// currently not use "starting_pose" and "header". Path frame is global/local frame
bool AdaptiveDockingLocalPlanner::setPlan (const std_msgs::Header &header, 
                const geometry_msgs::PoseStamped &starting_pose,
                const std::vector<geometry_msgs::PoseStamped> &goal_poses)
{
    docking_manager_->initDocking();
    return docking_manager_->setupPoses(goal_poses);
}

bool AdaptiveDockingLocalPlanner::setPlan (const std_msgs::Header &header,
            const std::vector<geometry_msgs::PoseStamped> &goal_poses)
{
    docking_manager_->initDocking();
    return docking_manager_->setupPoses(goal_poses);
}

/************* Execute Control ************/
// Just need velocity. Currently not use the other args
uint32_t AdaptiveDockingLocalPlanner::ExecuteControlLoop(const geometry_msgs::PoseStamped &pose,
                                const geometry_msgs::TwistStamped &velocity, 
                                geometry_msgs::Twist &cmd_vel,
                                std::string &message)
{
    docking_manager_->setRobotSpeed(velocity.twist);
    docking_manager_->dockingFSM();
    if (docking_manager_->docking_failed.data)
        return mbf_msgs::ExePathResult::FAILURE;
    cmd_vel = docking_manager_->getCmdVel();
    return mbf_msgs::ExePathResult::SUCCESS;
}

/************* Set Tolerance ************/
// Default value:
// dd: 0.1 (m)
// Approaching distance: dx: 0.03 (m), dy: 0.03 (m), dyaw: 0.05 (rad)
// Just use the IsGoalReached and set final tolerances if don't want to care about the other args
void AdaptiveDockingLocalPlanner::setGoalRange(double dd)
{
    docking_manager_->setGoalRange(dd);
}

bool AdaptiveDockingLocalPlanner::IsApproachingReached(double dx, double dy, double dyaw)
{
    docking_manager_->setApproachingTolerance(dx, dy, dyaw);
    return docking_manager_->isApproachingReach();
}

bool AdaptiveDockingLocalPlanner::IsGoalReached(double dx, double dy, double dyaw)
{
    docking_manager_->setDockingTolerance(dx, dy, dyaw);
    return docking_manager_->isGoalReach();
}

uint8_t AdaptiveDockingLocalPlanner::getDockingResult()
{ 
    return docking_manager_->getDockingResult();
}

geometry_msgs::Vector3 AdaptiveDockingLocalPlanner::getDockingFinalError()
{
    return docking_manager_->getDockingFinalError();
}
