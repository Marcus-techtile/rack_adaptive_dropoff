#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf2/utils.h>
#include <utils.h>

class PurePursuitController
{
private:
    ros::NodeHandle nh_;

    /* Control frequency */
    double freq_, dt_;
    /* Forklift parameters */
    double l_wheelbase_;

    /* input variables */
    nav_msgs::Odometry odom_;       
    nav_msgs::Path path_;
    double ref_vel_;    
    double cur_vel_, raw_cur_vel_;
    double lpf_output_{0.0};

    /* PP tune parameters */
    double min_look_ahead_dis_;
    double max_look_ahead_dis_;
    double max_steering_;   // max steering wheel angle
    double min_steering_;   // min steering wheel angle
    double goal_correct_yaw_;
    double k_lat_, k_angle_;

    // PID control
    double kp_, ki_;
    double pid_error_, pre_pid_error_;
    double i_sw_offset_;
    double p_part_, i_part_{0};
    bool use_track_path_pid_;
    double path_lateral_offset_;

    /* PP varibales */
    double distance_to_goal_;
    int closest_index_;
    double lk_time;
    geometry_msgs::Point point_lkh;
    bool use_point_interpolate_{true};
    bool use_ref_angle_from_path_;
    bool re_cal_lookahead_dis_;

public:
    PurePursuitController();
    PurePursuitController(ros::NodeHandle &paramGet);
    void resetPP();
    void setOdom(nav_msgs::Odometry odom);
    void setSpeed(geometry_msgs::Twist cur_speed);
    void setRefPath(nav_msgs::Path path);
    void setRefVel(double ref_vel);
    void setClosestPoint(int closest_index);
    void setLookaheadTime(double lk_t);
    void setRefAngleMode(bool use_angle_from_path);
    void limitLookaheadDistance(double min_dis, double max_dis);

    double calLookaheadDistance(double lk_t, double cur_spd);
    geometry_msgs::PoseStamped calLookaheadPoint(int nearest_index, double & lookahead_distance, nav_msgs::Path path);
    double calLookaheadCurvature(geometry_msgs::Point lookahead_point);
    geometry_msgs::Point interpolateLkhPoint(const geometry_msgs::Point & p1,
                                                    const geometry_msgs::Point & p2,
                                                    double r);
    void calControl();

    int point_index_;         // lookahead point index
    std_msgs::Float32 lateral_heading_error_;

    geometry_msgs::PoseStamped pp_lookahead_pose_;
    double look_ahead_distance_;
    double look_ahead_curvature_;
    double alpha_;              // angle between look ahead point and current pose
    double PP_steering_angle_;  // raw output from PP
    double steering_angle_;     // steering wheel output
};