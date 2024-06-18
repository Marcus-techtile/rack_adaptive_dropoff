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
       
/**
 * @brief Initialize the Adaptive Local Planner.
 * @param tf tf_buffer
 * @param sec tf transform timeout (second). Default: 1s
 */
    void initialize(tf2_ros::Buffer &tf, double sec);

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
/**
 * @brief Return Docking Result Code.
 * 
 * This function returns various result codes based on the outcome
 * of the docking process.
 *
 * @return The docking result code:
 * - 0: PROCESS (AD is still processing).
 * - 1: SUCCESS.
 * - 2: FAIL_DOCKING_PATH_IS_NOT_FEASIBLE.
 * - 3: FAIL_DOCKING_BAD_ACCURACY.
 * - 4: FAIL_TF_ERROR (Not implemented).
 * - 5: FAIL_INVALID_CONTROL_OUTPUT.
 */
    uint8_t getDockingResult();
    // Addition methods
    void setGoalRange(double dd);
    void setLocalFrame(std::string local_frame);
    void setGlobalFrame(std::string global_frame);
};
} //namespace
#endif // ADAPTIVE_DOCKING_LOCAL_PLANNER_H