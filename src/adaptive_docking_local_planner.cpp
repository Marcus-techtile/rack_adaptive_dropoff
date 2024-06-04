#include "adaptive_docking_local_planner.h"

using namespace techtile;

AdaptiveDockingLocalPlanner::AdaptiveDockingLocalPlanner(ros::NodeHandle &nh) : nh_(nh)
{
    initialize();
}

AdaptiveDockingLocalPlanner::~AdaptiveDockingLocalPlanner(){}

void AdaptiveDockingLocalPlanner::initialize()
{
    // docking_manager = std::make_shared<DockingManager>(nh_);
    docking_manager.initDocking();
}

bool AdaptiveDockingLocalPlanner::setPlan (const std_msgs::Header &header, 
                const geometry_msgs::PoseStamped &starting_pose,
                const geometry_msgs::PoseStamped &approaching_pose, 
                const geometry_msgs::PoseStamped &docking_pose)
{
    return docking_manager.setupPoses(approaching_pose, docking_pose);
}

bool AdaptiveDockingLocalPlanner::setPlan (const std_msgs::Header &header,
            const geometry_msgs::PoseStamped &approaching_pose, 
            const geometry_msgs::PoseStamped &docking_pose)
{
    return docking_manager.setupPoses(approaching_pose, docking_pose);
}

// uint32_t AdaptiveDockingLocalPlanner::ExecuteControlLoop(const geometry_msgs::PoseStamped &pose,
//                                 const geometry_msgs::TwistStamped &velocity, 
//                                 geometry_msgs::Twist &cmd_vel,
//                                 std::string &message)
uint32_t AdaptiveDockingLocalPlanner::ExecuteControlLoop()
{
    docking_manager.dockingFSM();
    if (docking_manager.docking_failed.data)
        return mbf_msgs::ExePathResult::FAILURE;
    return mbf_msgs::ExePathResult::SUCCESS;
}

bool AdaptiveDockingLocalPlanner::IsGoalReached()
{
    if (docking_manager.approaching_done.data && docking_manager.docking_done.data) 
        return true;
    else return false;
}
