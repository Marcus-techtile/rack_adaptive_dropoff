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
// uint32_t AdaptiveDockingLocalPlanner::ExecuteControlLoop(geometry_msgs::Twist &cmd_vel)
{
    docking_manager.setRobotSpeed(velocity.twist);
    docking_manager.dockingFSM();
    if (docking_manager.docking_failed.data)
        return mbf_msgs::ExePathResult::FAILURE;
    cmd_vel = docking_manager.getCmdVel();
    return mbf_msgs::ExePathResult::SUCCESS;
}

bool AdaptiveDockingLocalPlanner::IsGoalReached(double app_tol_x, double app_tol_y, double app_tol_yaw,
                       double docking_tol_x, double docking_tol_y, double docking_tol_yaw,
                       double distance_tol)
{
    docking_manager.setGoalTolerance(app_tol_x, app_tol_y, app_tol_yaw,
                            docking_tol_x, docking_tol_y, docking_tol_yaw,
                            distance_tol);
    return docking_manager.isGoalReach();
}
