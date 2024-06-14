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

    // DockingManager docking_manager{nh_};
    std::unique_ptr<DockingManager> docking_manager_;

public:
    AdaptiveDockingLocalPlanner(ros::NodeHandle &nh);
    ~AdaptiveDockingLocalPlanner();
       
    /**** MBF Interface ****/
    // 
    void initialize(tf2_ros::Buffer &tf);
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
    bool IsApproachingReached(double dx, double dy, double dyaw);
    bool IsGoalReached(double dx, double dy, double dyaw);
    uint8_t getDockingResult();
    // Addition methods
    void setGoalRange(double dd);
    void setLocalFrame(std::string local_frame);
    void setGlobalFrame(std::string global_frame);
    void SetApproachingTolerance (const double dx, const double dy, const double dyaw);
    void SetMaxSpeed(const double max_linear_speed);





};
} //namespace
#endif // ADAPTIVE_DOCKING_LOCAL_PLANNER_H