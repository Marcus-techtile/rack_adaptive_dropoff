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

void AdaptiveDockingLocalPlanner::setLocalFrame(std::string local_frame)
{
    docking_manager.setLocalFrame(local_frame);
}

bool AdaptiveDockingLocalPlanner::setPlan (const std_msgs::Header &header, 
                const geometry_msgs::PoseStamped &starting_pose,
                const geometry_msgs::PoseStamped &approaching_pose, 
                const geometry_msgs::PoseStamped &docking_pose)
{
    docking_manager.initDocking();
    return docking_manager.setupPoses(approaching_pose, docking_pose);
}

bool AdaptiveDockingLocalPlanner::setPlan (const std_msgs::Header &header,
            const geometry_msgs::PoseStamped &approaching_pose, 
            const geometry_msgs::PoseStamped &docking_pose)
{
    docking_manager.initDocking();
    return docking_manager.setupPoses(approaching_pose, docking_pose);
}

uint32_t AdaptiveDockingLocalPlanner::ExecuteControlLoop(const geometry_msgs::PoseStamped &pose,
                                const geometry_msgs::TwistStamped &velocity, 
                                geometry_msgs::Twist &cmd_vel,
                                std::string &message)
{
    docking_manager.setRobotSpeed(velocity.twist);
    docking_manager.dockingFSM();
    if (docking_manager.docking_failed.data)
        return mbf_msgs::ExePathResult::FAILURE;
    cmd_vel = docking_manager.getCmdVel();
    return mbf_msgs::ExePathResult::SUCCESS;
}

void AdaptiveDockingLocalPlanner::setGoalRange(double dd)
{
    docking_manager.setGoalRange(dd);
}

bool AdaptiveDockingLocalPlanner::IsApproachingReached(double dx, double dy, double dyaw)
{
    docking_manager.setApproachingTolerance(dx, dy, dyaw);
    return docking_manager.isApproachingReach();
}

bool AdaptiveDockingLocalPlanner::IsGoalReached(double dx, double dy, double dyaw)
{
    docking_manager.setDockingTolerance(dx, dy, dyaw);
    return docking_manager.isGoalReach();
}
