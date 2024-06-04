#include "quintic_planner.h"
#include "docking_control.h"

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
#include <mutex>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include "pallet_dock_msgs/PalletDockingAction.h"
#include "pallet_dock_msgs/LiftPositionAction.h"
#include "pallet_dock_msgs/LiftPositionActionGoal.h"
#include "forklift_msgs/CmdLiftMastActionGoal.h"
#include "forklift_msgs/CmdLiftMastAction.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

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
    ros::Publisher pub_docking_error_;
    ros::Publisher pub_global_goal_pose_;
    ros::Publisher pub_local_goal_pose_;
    ros::Publisher pub_goal_pose_array_;

    /* Subscriber */
    ros::Subscriber sub_cmd_vel;
    ros::Subscriber sub_move_back_;

    /* Docking Service*/
    ros::ServiceServer service_;
    
        // Pallet docking action server
    ros::Subscriber sub_docking_server_result_;
    ros::Subscriber sub_docking_server_goal_;
    pallet_dock_msgs::PalletDockingActionResult docking_server_result_;
    pallet_dock_msgs::PalletDockingActionGoal docking_server_goal_;
    bool docking_goal_avai_;

        // goal pose
    geometry_msgs::PoseStamped pallet_pose_;
    geometry_msgs::PoseStamped global_pallet_pose_;
    bool global_pallet_pose_setup_;

    geometry_msgs::PoseStamped approaching_goal_;
    geometry_msgs::PoseStamped docking_goal_;

        // Docking Mode
    bool docking_mode_;

    /* Auxiliary class*/
    QuinticPlanner quintic_planner{nh_};
    DockingControl docking_control{nh_};

    /* Turn on controller */
    std_msgs::Bool controller_on_;      // on/off signal for controller

    /* goal point */
    bool pose_setup_;
    geometry_msgs::PoseStamped approaching_goal_pose_;
    geometry_msgs::PoseStamped docking_goal_pose_;
    geometry_msgs::PoseStamped tf_approaching_goal_;
    geometry_msgs::PoseStamped tf_docking_goal_;
    bool norm_goal_frame_;
    
    double moveback_straight_distance_;
    
    geometry_msgs::PoseStamped  global_goal_pose_;         // goal pose in global_frame_ 
    geometry_msgs::PoseStamped  local_static_goal_pose_;   // goal pose in path_frame_
    geometry_msgs::PoseStamped  local_update_goal_pose_;   // goal pose in path_frame_. Updated each period

    

    double check_approaching_goal_distance_;                // distance from robot to approaching pose
    double approaching_min_dis_; //minimum necessary distance (perpendicular distance) to start approaching. Less than it, the forklift will move backward to increase the distance

    /* Tolerance (in local frame) */
    double distance_tolerance_;       // absolute tolerance of distance = sqrt(x^2+y^2)
    double angle_tolerance_, final_angle_tolerance_;    // absolute tolerance of yaw
    double x_tolerance_, final_x_tolerance_;
    double y_tolerance_, final_y_tolerance_;              // absolute tolerance of x
    bool check_inside_goal_range_{false};       // if the distance error < distance tol -> inside goal range
    int count_outside_goal_range_;

    /* Docking State Control */
    bool start_pallet_docking_, start_returning_;
    std_msgs::Bool returning_mode_;             // Mode returning back to the pre-docking position
    bool start_docking_FSM{false};
    enum docking_state {IDLE, APPROACHING, DOCKING, 
                        SET_GOAL, UPDATE_GOAL, SIDESHIFT_CONTROL, GEN_PATH_AND_PUB_CONTROL, STOP, RECOVER, END, FAILURE};
    bool pallet_pose_avai_{false};  // transition from GET_PALLET_POSE to APPROACH
    bool approach_done_{false};     // transition from APPROACHING to DOCKING, to SET_GOAL
    bool docking_done_{false};       // transition from DOCKING to SET_GOAL
    bool goal_setup_{false};        // transition from SET_GOAL to UPDATE_GOAL
    bool goal_avai_{false};         // transition from UPDATE_GOAL to GEN_PATH
    bool goal_reach_{false};         // transition from GEN_PATH to STOP
    bool goal_failed_{false};        // transition from GEN_PATH to RECOVER

    docking_state current_pallet_docking_state_, old_state_;
    std_msgs::String docking_state, old_docking_state;  //publish to topic

    /* Failure cases */
    int failure_code_;
    std::vector<std::string> failure_cases_;
    std::map<int, std::string> failure_map_;

    /* tf conversion */
    std::string path_frame_;
    std::string global_frame_;
    tf2_ros::Buffer docking_tf_buffer;
    tf2_ros::TransformListener docking_listener{docking_tf_buffer};

    /* Mutex */
    std::mutex mutex_;

    /* Callback function */
    bool dockingServiceCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    void dockingServerResultCallback(const pallet_dock_msgs::PalletDockingActionResult::ConstPtr& msg);
    void dockingServerGoalCallback(const pallet_dock_msgs::PalletDockingActionGoal::ConstPtr& msg);

    void goalSetup();
    void updateGoal();
    void checkGoalReach();
public:
    DockingManager(ros::NodeHandle &nh);
    ~ DockingManager();
    void setParam(ros::NodeHandle &nh);

    void initDocking();
    void resetPlanAndControl();

    bool setupPoses(geometry_msgs::PoseStamped approaching_pose,
                geometry_msgs::PoseStamped docking_pose);

    // Planner state transition
    void idleState();
    void approachingState();
    void dockingState();

    bool ExtendApproachingDistance();
    void setGoalState();

    void quinticPlannerSetup();
    void goalReachHandling();
    void goalFailHandling();
    void genPathAndPubControlState();

    void stopState();
    void recoverState();
    void endState();
    void failureState();

    bool startFSM();
    void dockingFSM();

    geometry_msgs::Twist getCmdVel();

    /* Docking state of each stage*/
    std_msgs::Bool docking_done;        // Docking process done or not
    std_msgs::Bool approaching_done;    // Approaching process done or not
    std_msgs::Bool docking_failed;
};