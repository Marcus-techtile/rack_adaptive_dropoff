#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <nav_msgs/Path.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Twist.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <dynamic_reconfigure/server.h>
#include <pallet_docking_xsquare/purePursuitReconfigConfig.h>

#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

#include "utils.h"
#include "fuzzy_control.h"
#include "pure_pursuit_control.h"

class DockingControl
{
private:
    ros::NodeHandle nh_;

    /* Subscriber */
    ros::Subscriber sub_odom_, sub_steering_;
    ros::Subscriber sub_ref_path_;
    ros::Subscriber sub_goal_pose_;
    ros::Subscriber sub_controller_on_, sub_approaching_status_;
    ros::Subscriber joint_states_sub_;

    /* Publisher */
    ros::Publisher pub_cmd_vel_;
    ros::Publisher pub_pp_steering_;
    ros::Publisher pub_local_path_;
    ros::Publisher pub_debug_;
    ros::Publisher pub_pp_lookahead_distance_;
    ros::Publisher pub_pp_lookahead_angle_;
    ros::Publisher pub_pp_lookahead_pose_, pub_nearest_pose_;;
    ros::Publisher pub_pp_lookahead_curvature_;
    ros::Publisher marker_pub_;

    /* Odometry sub */
    nav_msgs::Odometry odom_sub_;
    bool odom_avai_{false};

    /* Steering sub */
    double steering_sub_;

    double docking_freq_;
    double dt_;

    /* Ref path */
    nav_msgs::Path ref_path_, local_ref_path_;
    bool ref_path_avai_{false};
    geometry_msgs::PoseWithCovarianceStamped goal_pose_;
    bool goal_avai_{false};

    /* Forklift parameters */
    double l_wheelbase_;

    /* Control general variable */
    std::string path_frame_;
    bool init_reconfig_{true};
    std_msgs::Bool approaching_done_;
    bool pub_stop_{false};

    /* PP tune parameters */
    double max_steering_;   // max steering wheel angle
    double min_steering_;   // min steering wheel angle
    double max_vel_;        // max linear velocity
    double min_vel_;        // min linear velocity
    double goal_correct_yaw_{0.3};

    /* PP varibales */
    PurePursuitController pure_pursuit_control;
    double steering_angle_, steering_;     // steering wheel
    double pp_look_ahead_time_, pp_look_ahead_time_straigh_line_;
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

    /* Limit PP lookahead distance */
    double pp_min_lk_distance_approaching_;
    double pp_min_lk_distance_docking_;

    /* Output control command */
    geometry_msgs::Twist cmd_vel_;   // command velocity

    /* tf conversion */
    tf2_ros::Buffer tf_buffer_c;
    tf2_ros::TransformListener listener{tf_buffer_c};

    /* Limit angular rate */
    double max_angular_vel_;

    bool limit_sp_curve_;
    bool adaptive_ref_angle_;

    /* Callback function */
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg_odom);
    void JointStateCallBack(const sensor_msgs::JointState::ConstPtr &msg);
    void refPathCallback(const nav_msgs::Path::ConstPtr& msg);
    void goalPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void controllerOnCallback(const std_msgs::Bool::ConstPtr& msg);
    void approachingStatusCallback(const std_msgs::Bool::ConstPtr& msg);

public:
    DockingControl();
    DockingControl(ros::NodeHandle &paramGet);
    ~DockingControl();

    /* Functions */
    void resetController();
    bool checkData();
    nav_msgs::Path convertPathtoLocalFrame(nav_msgs::Path global_path);
    int nearestPointIndexFind(nav_msgs::Path local_path);
    void controllerCal();

    std_msgs::Bool controller_on_;
};