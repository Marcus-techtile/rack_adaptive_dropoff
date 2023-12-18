#include "quintic_planner.h"

#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <string.h>
#include <map>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include "pallet_dock_msgs/PalletDockingAction.h"

class DockingManager
{
private:
    ros::NodeHandle nh_;

    /* Publisher */
    ros::Publisher pub_docking_state;
    ros::Publisher pub_docking_done;
    ros::Publisher pub_approaching_done;
    ros::Publisher pub_cmd_vel;
    ros::Publisher pub_controller_on_;
    ros::Publisher pub_fake_goal_pose_;
    ros::Publisher pub_goal_pose_;

    /* Subscriber */
    ros::Subscriber sub_cmd_vel;
    ros::Subscriber sub_pallet_pose_;
    ros::Subscriber sub_pallet_pose_ready_;
    
        // Pallet docking action server
    ros::Subscriber sub_docking_server_result_;
    pallet_dock_msgs::PalletDockingActionResult docking_server_result_;

        // Pallet pose
    geometry_msgs::PoseStamped pallet_pose_;
    std_msgs::Bool pallet_pose_ready_;
    ros::Time starting_detection_time_;
    double detection_timeout_;

    /* Docking Service*/
    ros::ServiceServer service_;
    
    /* Pallet detection client service */
    ros::ServiceClient pallet_detect_srv_client_;

    /* Subscriber variables */
    geometry_msgs::Twist cmd_vel_sub;

    /* Auxiliary class*/
    QuinticPlanner quintic_planner;

    double goal_distance;       // setup distance from the goal to the pallet pose

    std_msgs::Bool docking_done;        // Docking process done or not
    std_msgs::Bool approaching_done;    // Approaching process done or not

    geometry_msgs::Twist cmd_fw;

    std_msgs::Bool controller_on_;

    int count_cmd_back_vel{0};

    /* goal point */
    bool move_reverse_{false};
    double dis_approach_offset_ = 1.5; // Offset from pallet to goal
    double dis_docking_offset_ = 0.5;
    geometry_msgs::Vector3 goal_pose_;      // pose of the goal, vector3 type x:x; y:y, z:yaw
    geometry_msgs::PoseStamped global_goal_pose_;
    geometry_msgs::PoseStamped  local_static_goal_pose_;   //local goal to facilitate computing global goal
    geometry_msgs::PoseStamped  local_goal_pose_;
    double goal_sideshift_;

    double check_goal_distance_;
    double approaching_min_dis_; //minimum necessary distance (perpendicular distance) to start approaching. Less than it, the forklift will move backward to increase the distance

    /* Tolerance (in local frame) */
    double distance_tolerance_{0.05};       // absolute tolerance of distance = sqrt(x^2+y^2)
    double angle_tolerance_{5*180/M_PI};    // absolute tolerance of yaw
    double x_tolerance_{0.1}, y_tolerance_{0.1};              // absolute tolerance of x
    bool check_inside_goal_range_{false};
    int count_outside_goal_range_;

    int count_path_gen_fail_{0};
    int count_goal_failed_{0};

    /* Docking location */
    std::string docking_area_;

    /* Docking State Control */
    bool start_docking_FSM{false};
    enum docking_state {IDLE, GET_PALLET_POSE, APPROACHING, DOCKING, 
                        SET_GOAL, UPDATE_GOAL, SIDESHIFT_CONTROL, GEN_PATH_AND_PUB_CONTROL, STOP, RECOVER, END, FAILURE};
    bool start_fsm_{false};
    bool get_pallet_pose_{false}; // transition to start GET_PALLET_POSE
    bool pallet_pose_avai_{false};  // transition from GET_PALLET_POSE to APPROACH
    bool approach_done_{false};     // transition from APPROACHING to DOCKING, to SET_GOAL
    bool docking_done_{false};       // transition from DOCKING to SET_GOAL
    bool goal_setup_{false};        // transition from SET_GOAL to UPDATE_GOAL
    bool goal_avai_{false};         // transition from UPDATE_GOAL to GEN_PATH
    bool goal_reach_{false};         // transition from GEN_PATH to STOP
    bool goal_failed_{false};        // transition from GEN_PATH to RECOVER

    docking_state current_state_, old_state_;
    std_msgs::String docking_state, old_docking_state;  //publish to topic

    /* Failure cases */
    int failure_code_;
    std::vector<std::string> failure_cases_;
    std::map<int, std::string> failure_map_;

    double starting_time_recover;

    /* tf conversion */
    std::string path_frame_;
    tf2_ros::Buffer docking_tf_buffer;
    tf2_ros::TransformListener docking_listener{docking_tf_buffer};

    tf2_ros::Buffer pose_tf_buffer;
    tf2_ros::TransformListener pose_listener{pose_tf_buffer};

    /* Fake goal */
    double fake_goal_x_, fake_goal_y_, fake_goal_yaw_;
    bool use_fake_goal_;

    /* Callback function */
    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
    bool dockingServiceCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    void palletPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void dockingServerResultCallback(const pallet_dock_msgs::PalletDockingActionResult::ConstPtr& msg);
    

    void goalSetup(double distance_pallet, geometry_msgs::PoseStamped pallet_pose);
    void updateGoal();
    void checkGoalReach();
public:
    DockingManager(ros::NodeHandle &nh);
    ~ DockingManager();

    void initDocking();
    void resetPlanAndControl();
    void dockingFSM();

};