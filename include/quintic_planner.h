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

#include <dynamic_reconfigure/server.h>
#include <pallet_docking_xsquare/quinticPlannerReconfigConfig.h>
#include <std_srvs/SetBool.h>

typedef pallet_docking_xsquare::quinticPlannerReconfigConfig config;

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

    /* Subscriber */
    ros::Subscriber sub_odom_, sub_pallet_pose_, sub_pallet_pose_ready_;

    /* Publisher */
    ros::Publisher pub_quintic_pose_, pub_quintic_path_, marker_pub_;

    boost::shared_ptr <dynamic_reconfigure::Server<config> > srv_;

    bool init_reconfig_{true};

    /* Quintic parameters */
    double sx_, sy_, syaw_, sv_, sa_;
    double gx_, gy_, gyaw_, gv_, ga_;
    double max_yaw_rate_, max_accel_, max_ax_, max_ay_, max_jerk_;
    double max_curv_;
    double dt_;

    /* odometry and pallet subscribe */
    double odom_yaw_;

    /* fake goal for test */
    double fake_goal_x_;
    double fake_goal_y_;
    double fake_goal_yaw_;

    /* tf conversion */
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener{tf_buffer};

public:
    /* function */
    QuinticPlanner(ros::NodeHandle &paramGet);

    /* Quintic planner */
    bool set_param_{false};
    std::string odom_topic_;
    std::string path_frame_;
    geometry_msgs::PoseArray quintic_pose_;
    nav_msgs::Path quintic_path_, local_quintic_path_; 

    double min_t_; //min time to goal
    double max_t_; //max time to goal

    /* starting constraint */
    double starting_vel_;
    double starting_acc_;

    /* ending constraint */
    double stopping_vel_;
    double stopping_acc_;

    /* odom */
    nav_msgs::Odometry odom_;
    double cur_vel_;
    bool odom_avai_{false};

    /* path variables*/
    bool path_feasible_{false};
    bool path_avai_{false};
    std::vector<double> time_;                     // time index
    std::vector<double> rx_, ry_, ryaw_, r_vyaw_;
    std::vector<double> rv_, ra_, rax_, ray_, rj_;        //x, y, yaw
    std::vector<double> curv_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg_odom);
    void reconfigCallback(pallet_docking_xsquare::quinticPlannerReconfigConfig &config, uint32_t level);

    void setParams(double sx, double sy, double syaw, double sv, double sa,
                    double gx, double gy, double gyaw, double gv, double ga);
    void genPath();
    void resetPlanner();
    void visualize(geometry_msgs::PoseStamped pallet_pose);
};
