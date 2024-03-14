#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <utils.h>

class PurePursuitController
{
private:
    ros::NodeHandle nh_;

    /* Forklift parameters */
    double l_wheelbase_;

    /* PP tune parameters */
    double look_ahead_time_;   // look ahead time
    double look_ahead_reverse_time_;
    double min_look_ahead_dis_;
    double max_look_ahead_dis_;
    
    double max_steering_;   // max steering wheel angle
    double min_steering_;   // min steering wheel angle
    double goal_correct_yaw_{0.1};

    double kp_, ki_;
    double sum_e_la{0};

    /* PP varibales */

    double distance_;       // distance between look ahead point and current pose
    geometry_msgs::PoseStamped look_ahead_point_;  // pose of the look head point
    double rotational_radius_;  // rotational radius
    double steering_angle_;     // steering wheel
    bool correct_yaw_{false};
    // double cur_vel_, raw_cur_vel_;
    bool rotate_in_place_{false};
    int closest_point_;
    geometry_msgs::Point point0, point1, point_lkh;
    bool use_point_interpolate_{true};

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
    void setClosestPoint(int closest_point);
    void setGoalCorrectYaw(double dis_cor);
    void setLookaheadTime(double lk_t);

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

    double look_ahead_distance_;
    double lk_time;
    double alpha_;          // angle between look ahead point and current pose
};