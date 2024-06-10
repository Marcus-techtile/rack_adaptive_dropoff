#ifndef ADAPTIVE_DOCKING_LOCAL_PLANNER_H
#define ADAPTIVE_DOCKING_LOCAL_PLANNER_H
#include <ros/ros.h>
#include "docking_planner.h"
#include <mbf_msgs/ExePathResult.h>

namespace techtile {
class AdaptiveDockingLocalPlanner
{
private:
    ros::NodeHandle nh_;

    DockingManager docking_manager{nh_};

public:
    AdaptiveDockingLocalPlanner(ros::NodeHandle &nh);
    ~AdaptiveDockingLocalPlanner();
       
    /**** MBF Interface ****/
    // 
    void initialize();
    bool setPlan (const std_msgs::Header &header, 
				const geometry_msgs::PoseStamped &starting_pose,
				const geometry_msgs::PoseStamped &approaching_pose, 
				const geometry_msgs::PoseStamped &docking_pose);
    bool setPlan (const std_msgs::Header &header,
                const geometry_msgs::PoseStamped &approaching_pose, 
                const geometry_msgs::PoseStamped &docking_pose);
    uint32_t ExecuteControlLoop(const geometry_msgs::PoseStamped &pose,
                                    const geometry_msgs::TwistStamped &velocity, 
                                    geometry_msgs::Twist &cmd_vel,
                                    std::string &message);
    uint32_t ExecuteControlLoop(geometry_msgs::Twist &cmd_vel);
    bool IsGoalReached(double app_tol_x, double app_tol_y, double app_tol_yaw,
                       double docking_tol_x, double docking_tol_y, double docking_tol_yaw,
                       double distance_tol);

    // Addition methods
    void SetDockingTolerance (const double dx, const double dy, const double dyaw);
    void SetApproachingTolerance (const double dx, const double dy, const double dyaw);
    void SetMaxSpeed(const double max_linear_speed);





};
} //namespace
#endif // ADAPTIVE_DOCKING_LOCAL_PLANNER_H