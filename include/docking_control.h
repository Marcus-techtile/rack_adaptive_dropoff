#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <nav_msgs/Path.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Twist.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <visualization_msgs/Marker.h>

#include "fuzzy_control.h"
#include "pure_pursuit_control.h"
#include <mutex>

class DockingControl
{
private:
    ros::NodeHandle nh_;

    /* Publisher */
    ros::Publisher pub_cmd_vel_;
    ros::Publisher pub_pp_steering_;
    ros::Publisher pub_local_path_;
    ros::Publisher pub_pp_lookahead_distance_;
    ros::Publisher pub_pp_lookahead_angle_;
    ros::Publisher pub_pp_lookahead_pose_, pub_nearest_pose_;
    ros::Publisher pub_pp_lookahead_curvature_;
    ros::Publisher marker_pub_;
    ros::Publisher pub_docking_local_path_;

    double docking_freq_;
    double dt_;

    bool use_cost_function_;

    /* Ref path */
    nav_msgs::Path ref_path_, local_ref_path_;
    bool ref_path_avai_{false};

    /* Local Control Path Output*/
    nav_msgs::Path local_control_path_;
    nav_msgs::Path predict_path_;

    /* Local Dynamic Goal */
    geometry_msgs::PoseStamped local_goal_;
    geometry_msgs::PoseStamped lookehead_pose_;

    /* Predict*/
    double predict_time_{5.0};

    /* Forklift parameters */
    double l_wheelbase_;
    geometry_msgs::Twist robot_speed_;

    /* control point index */
    int nearest_index_;
    int lk_index_vel_;

    /* PP tune parameters */
    double max_steering_;   // max steering wheel angle
    double min_steering_;   // min steering wheel angle
    double max_vel_;        // max linear velocity
    double min_vel_;        // min linear velocity
    double goal_correct_yaw_{0.3};

    /* PP varibales */
    PurePursuitController pure_pursuit_control;
    double steering_angle_, steering_;     // steering wheel
    double pp_look_ahead_time_;
    geometry_msgs::PoseStamped pp_lkh_pose_;

    /* Fuzzy control */
    FuzzyControl fuzzy_controller;
    double fuzzy_lookahead_dis_;
    double ref_velocity_, abs_ref_vel_, final_ref_vel_;       // reference velocity
    double backward_offset_;        //offset velocity used to switch to reverse movement
    double max_linear_acc_, min_linear_acc_;

    /* Limit docking velocity */
    double max_steering_speed_, min_steering_speed_;
    double max_pocket_dock_vel_;
    double max_pocket_dock_steering_;

    /* tf conversion */
    tf2_ros::Buffer &tf_buffer_c;
    double tf_time_out_{1.0};

    /* Limit angular rate */
    double max_angular_vel_;

    bool limit_sp_curve_;
    bool adaptive_ref_angle_;

    /* Mutex */
    std::mutex mutex_;

    void visualize();

public:
    DockingControl(ros::NodeHandle &nh, tf2_ros::Buffer &tf, double sec);
    ~DockingControl();

    /* Functions */
    void resetController();
    bool checkData();
    void setRefPath(nav_msgs::Path path);
    void setVel(geometry_msgs::Twist robot_speed);
    void setLocalGoal(geometry_msgs::PoseStamped local_goal);
    nav_msgs::Path convertPathtoLocalFrame(nav_msgs::Path global_path);
    int nearestPointIndexFind(nav_msgs::Path local_path);
    void steeringControl();
    void linearSpeedControl();
    void limitControlSignal();
    void controllerCal();
    nav_msgs::Path predictPath(geometry_msgs::Twist cmd_in);
    std::vector<geometry_msgs::Twist> generateControlSample(geometry_msgs::Twist current_cmd, geometry_msgs::Twist cal_cmd);
    geometry_msgs::Twist evaluateControlSampleAndOutputControl(std::vector<geometry_msgs::Twist> control_samp);

    std::string path_frame_{"base_link_p"};

    /* Output control command */
    double gain_heading_, gain_track_, gain_vel_;
    geometry_msgs::Twist cmd_vel_;   // command velocity
    geometry_msgs::Twist pre_cmd_vel_;

    std_msgs::Bool approaching_done_;
    std_msgs::Bool controller_on_;
    bool invalid_control_signal_;
    bool publish_cmd_{false};
};