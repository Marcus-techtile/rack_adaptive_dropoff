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

    double freq_, dt_;
    /* Forklift parameters */
    double l_wheelbase_;

    /* PP tune parameters */
    double look_ahead_time_;   // look ahead time
    double look_ahead_reverse_time_;
    double min_look_ahead_dis_;
    double max_look_ahead_dis_;
    
    double max_steering_;   // max steering wheel angle
    double min_steering_;   // min steering wheel angle
    double goal_correct_yaw_;

    double kp_, ki_;
    double sum_e_la{0};

    /* PP varibales */
    int closest_index_;
    geometry_msgs::Point point0, point1, point_lkh;
    bool use_point_interpolate_{true};
    bool use_ref_angle_from_path_;

    bool re_cal_lookahead_dis_;

    /* input variables */
    nav_msgs::Odometry odom_;
    nav_msgs::Path path_;
    double ref_vel_;

    double lpf_output_{0.0};

public:
    PurePursuitController();
    PurePursuitController(ros::NodeHandle &paramGet);
    void resetPP();
    void setOdom(nav_msgs::Odometry odom);
    void setRefPath(nav_msgs::Path path);
    void setRefVel(double ref_vel);
    void setClosestPoint(int closest_index);
    void setLookaheadTime(double lk_t);
    void limitLookaheadDistance(double min_dis, double max_dis);

    double calLookaheadDistance(double lk_t, double cur_spd);
    geometry_msgs::PoseStamped calLookaheadPoint(int nearest_index, double & lookahead_distance, nav_msgs::Path path);
    double calLookaheadCurvature(geometry_msgs::Point lookahead_point);
    //using the intersection between circle and 2 points of the line to interpolate the lkd point
    geometry_msgs::Point interpolateLkhPoint(const geometry_msgs::Point & p1,
                                                    const geometry_msgs::Point & p2,
                                                    double r);
    void calControl();
    double getSteeringAngle();

    int point_index_, pre_point_index_;         // public for visualization
    int max_lk_reverse_point_;
    std_msgs::Float32 lateral_heading_error_, lateral_error_;
    double cur_vel_, raw_cur_vel_;

    geometry_msgs::PoseStamped pp_lookahead_pose_;
    double look_ahead_distance_;
    double look_ahead_curvature_;
    double lk_time;
    double alpha_;          // angle between look ahead point and current pose

    double PP_steering_angle_;
    double steering_angle_;     // steering wheel
};