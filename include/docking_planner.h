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

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "rack_detection_msg/RackDeviation.h"
#include "rack_detection_msg/RackLine.h"

class DockingManager
{
private:
    ros::NodeHandle nh_;

    /* Publisher */
    ros::Publisher pub_docking_state;
    ros::Publisher pub_docking_done;
    ros::Publisher pub_approaching_done;
    ros::Publisher pub_cmd_vel;
    ros::Publisher pub_docking_error_;
    ros::Publisher pub_global_goal_pose_;
    ros::Publisher pub_local_goal_pose_;
    ros::Publisher pub_goal_pose_array_;
    ros::Publisher pub_approaching_error_;
    ros::Publisher pub_final_docking_error_;

    /* Subscriber */
    ros::Subscriber sub_cmd_vel;
    ros::Subscriber sub_move_back_;
    ros::Subscriber sub_rack_deviation_;
        std::string rack_deviation_topic_;
        rack_detection_msg::RackDeviation rack_deviation_;
        void rackDeviationCallback(const rack_detection_msg::RackDeviation::ConstPtr &msg);
        bool rack_deviation_avai_;
    ros::Subscriber sub_rack_line_;
        std::string rack_line_topic_;
        rack_detection_msg::RackLine rack_line_;
        void rackLineCallback(const rack_detection_msg::RackLine::ConstPtr &msg);
        bool rack_line_avai_;

        // Pallet docking action server
    ros::Subscriber sub_docking_server_result_;
    ros::Subscriber sub_docking_server_goal_;
    bool docking_goal_avai_;

        // goal pose
    geometry_msgs::PoseStamped pallet_pose_;
    geometry_msgs::PoseStamped global_pallet_pose_;
    bool global_pallet_pose_setup_;

    geometry_msgs::PoseStamped approaching_goal_;
    geometry_msgs::PoseStamped docking_goal_;

        // Docking Mode
    bool docking_mode_{0};

    /* Auxiliary class*/
    std::shared_ptr<QuinticPlanner> quintic_planner_;
    std::shared_ptr<DockingControl> docking_control_;

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
    double goal_range_{0.1};       // absolute distance = sqrt(x^2+y^2) is consider inside goal range
    double app_angle_tolerance_{0.05}, docking_angle_tolerance_{0.02};    // absolute tolerance of yaw
    double app_x_tolerance_{0.03}, docking_x_tolerance_{0.01};
    double app_y_tolerance_{0.03}, docking_y_tolerance_{0.025};              // absolute tolerance of x
    double x_tolerance_, y_tolerance_, angle_tolerance_;
    bool check_inside_goal_range_{false};       // if the distance error < distance tol -> inside goal range
    int count_outside_goal_range_;

    /* Docking error */
    geometry_msgs::Vector3 approaching_error_;
    geometry_msgs::Vector3 final_error_;

    // limit for final docking tolerance
    double limit_tol_x_{0.001}, limit_tol_y_{0.001}, limit_tol_angle_{0.01};

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
    std::string path_frame_{"base_link_p"};
    std::string global_frame_;

    tf2_ros::Buffer& tf_;
    double tf_time_out_{1.0};

    /* Mutex */
    std::mutex mutex_;

    void goalSetup();
    void updateGoal();
    void checkGoalReach();
    void getAndPubDockingErrors(double error_x, double error_y, double error_yaw);
public:
    DockingManager(ros::NodeHandle &nh, tf2_ros::Buffer &tf, double sec);
    ~ DockingManager();
    void config();

    void initDocking();
    void resetPlanAndControl();

    bool setupPoses(geometry_msgs::PoseStamped approaching_pose,
                geometry_msgs::PoseStamped docking_pose);
    void setGoalRange(double dd);
    void setApproachingTolerance(double dx, double dy, double dyaw);
    void setDockingTolerance(double &dx, double &dy, double &dyaw);
    void setLocalFrame(std::string local_frame);
    void setGLobalFrame(std::string global_frame);

    // Planner state transition
    void idleState();
    void approachingState();
    void dockingState();

    bool ExtendApproachingDistance();
    void setGoalState();

    void quinticPlannerSetup();
    void goalReachHandling();
    void goalFailHandling();
    void setRobotSpeed(geometry_msgs::Twist robot_speed);
    void genPathAndPubControlState();

    void stopState();
    void recoverState();
    void endState();
    void failureState();

    bool startFSM();
    void dockingFSM();

    // Additional method
    bool isApproachingReach();
    bool isGoalReach();
    geometry_msgs::Twist getCmdVel();

    uint8_t getDockingResult();
    geometry_msgs::Vector3 getDockingFinalError();

    /* Docking state of each stage*/
    std_msgs::Bool docking_done;        // Docking process done or not
    std_msgs::Bool approaching_done;    // Approaching process done or not
    std_msgs::Bool docking_failed;

    enum class dockingResult:uint8_t{PROCESS, SUCCESS, FAIL_DOCKING_PATH_IS_NOT_FEASIBLE, 
                                FAIL_DOCKING_BAD_ACCURACY, FAIL_TF_ERROR, FAIL_INVALID_CONTROL_OUTPUT};
    dockingResult docking_result; 
};