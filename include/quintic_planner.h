#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "utils.h"

#include <std_srvs/SetBool.h>

class QuinticPolynominal
{
private:
    Eigen::MatrixXd a_matrix = Eigen::MatrixXd(3,3);
    Eigen::MatrixXd a_invese = Eigen::MatrixXd(3,3);
    Eigen::MatrixXd b_matrix = Eigen::MatrixXd(3,1);
    Eigen::MatrixXd x_matrix = Eigen::MatrixXd(3,1);

public:
    QuinticPolynominal(double x0, double v0, double a0, double xf, double yf, double af, double tf);
    double cal_point(double ti);
    double cal_vel(double ti);
    double cal_acc(double ti);
    double cal_jerk(double ti);

    std::vector<double> a_coef;
    double tf_2, tf_3, tf_4, tf_5;
    double xt;
    double dxt;
    double ddxt;
    double dddxt;
};

class QuinticPlanner
{
private:
    ros::NodeHandle nh_;

    /* Publisher */
    ros::Publisher pub_quintic_pose_;
    ros::Publisher pub_quintic_local_path_, pub_quintic_path_;
    ros::Publisher marker_pub_;

    /* Quintic parameters */
    double sx_, sy_, syaw_, sv_, sa_;
    double gx_, gy_, gyaw_, gv_, ga_;
    double max_acc_, min_highest_acc_, max_jerk_;
    double dt_;

    /* odometry and pallet subscribe */
    double odom_yaw_;

    /* tf conversion */
    tf2_ros::Buffer &tf_buffer;
    double tf_time_out_{1.0};
public:
    /* function */
    QuinticPlanner(ros::NodeHandle &nh, tf2_ros::Buffer &tf, double sec);

    /* Quintic planner */
    bool set_param_{false};
    std::string path_frame_{"base_link_p"};
    std::string global_frame_{"map"};
    geometry_msgs::PoseArray quintic_pose_, segment_quintic_pose_;
    nav_msgs::Path quintic_path_, global_quintic_path_, segment_quintic_path_; 

    double min_t_; //min time to goal
    double max_t_; //max time to goal

    /* starting constraint */
    double starting_vel_{0.0};
    double starting_acc_{0.0};
    double max_starting_vel_{0.1};

    /* ending constraint */
    double stopping_vel_{0.0};
    double stopping_acc_{0.0};
    double max_ending_vel_{0.3};
    double k_ending_lat_{0.8}, k_ending_angle_{0.5};

    /* path variables*/
    bool path_feasible_{false};
    bool segment_path_feasible_{false};
    bool path_avai_{false};
    std::vector<double> time_;                     // time index
    std::vector<double> rx_, ry_, ryaw_;
    std::vector<double> rv_, ra_, rax_, ray_, rj_;        //x, y, yaw

    void setFrame(std::string local_frame, std::string global_frame);
    void setParams(double sx, double sy, double syaw, double sv, double sa,
                    double gx, double gy, double gyaw, double gv, double ga);
    
    std::vector<geometry_msgs::PoseStamped> waypointUpdate(std::vector<geometry_msgs::PoseStamped> waypoints);
    
    void genPath(std::vector<geometry_msgs::PoseStamped> waypoints);
    void resetPlanner();
    void visualize(geometry_msgs::PoseStamped pallet_pose);
};
