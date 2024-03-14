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

typedef pallet_docking_xsquare::purePursuitReconfigConfig config;

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
    ros::Publisher pub_local_path_;
    ros::Publisher pub_debug_;
    ros::Publisher pub_pp_lookahead_distance_;
    ros::Publisher pub_pp_lookahead_angle_;
    ros::Publisher marker_pub_;
    boost::shared_ptr <dynamic_reconfigure::Server<config> > srv_;

    /* Odometry sub */
    nav_msgs::Odometry odom_sub_;
    bool odom_avai_{false};

    /* Steering sub */
    double steering_sub_;

    /* Ref path */
    nav_msgs::Path ref_path_, local_ref_path_;
    bool ref_path_avai_{false};
    geometry_msgs::PoseWithCovarianceStamped goal_pose_;
    bool goal_avai_{false};

    /* Forklift parameters */
    double l_wheelbase_;

    /* PP tune parameters */
    int look_ahead_;   // look ahead point of the ref path
    double alpha_offset_;   // offset for angle error
    double max_steering_;   // max steering wheel angle
    double min_steering_;   // min steering wheel angle
    double max_vel_;        // max linear velocity
    double min_vel_;        // min linear velocity
    double goal_correct_yaw_{0.3};

    /* PP varibales */
    double alpha_;          // angle between look ahead point and current pose
    double distance_;       // distance between look ahead point and current pose
    double goal_distance_;
    geometry_msgs::PoseStamped look_ahead_point_;  // pose of the look head point
    double rotational_radius_;  // rotational radius
    double steering_angle_, steering_;     // steering wheel
    double ref_velocity_, final_ref_vel_;       // reference velocity
    geometry_msgs::Twist cmd_vel_;   // command velocity
    bool correct_yaw_{false};

    double last_ref_vel_{0.0};
    double velocity_rate_limit_;

    bool init_reconfig_{true};
    std_msgs::Bool controller_on_, approaching_done_;
    bool pub_stop_{false};
    
    FuzzyControl fuzzy_controller;
    double fuzzy_lookahead_dis_;
    PurePursuitController pure_pursuit_control;

    /* tf conversion */
    tf2_ros::Buffer tf_buffer_c;
    tf2_ros::TransformListener listener{tf_buffer_c};

    /* Low pass filter */
    double lpf_output_s_{0.0}, lpf_output_v_{0.0}, lpf_output_d_{0.0};
    double e_pow_s_, e_pow_v_, e_pow_d_;

    /* Limit angular rate */
    double max_angular_vel_;

    /* Limit docking velocity */
    double max_pocket_dock_vel_;
    double max_pocket_dock_steering_;

    std::string path_frame_;
    
public:
    DockingControl(ros::NodeHandle &paramGet)
    {
        /* Get Param */
        paramGet.param<std::string>("path_frame", path_frame_, "base_link_p");
        paramGet.param("/forklift_params/wheel_base", l_wheelbase_, 1.311);
        // ROS_INFO("Robot model wheel base (%f)", l_wheelbase_);
        paramGet.param<int>("look_ahead", look_ahead_, 10);
        paramGet.param<double>("alpha_offset", alpha_offset_, 0.005);
        paramGet.param<double>("max_steering", max_steering_, 1.5);
        paramGet.param<double>("min_steering", min_steering_, -1.5);
        paramGet.param<double>("max_vel", max_vel_, 0.3);
        paramGet.param<double>("min_vel", min_vel_, 0.15);
        paramGet.param<double>("goal_correct_yaw", goal_correct_yaw_, 0.3);
        paramGet.param<double>("velocity_rate_limit", velocity_rate_limit_, 0.01);
        paramGet.param<double>("max_angular_vel", max_angular_vel_, 0.2);

        paramGet.param<double>("fuzzy_lookahead_dis", fuzzy_lookahead_dis_, 0.3);
        paramGet.param<double>("max_pocket_dock_vel", max_pocket_dock_vel_, 0.2);
        paramGet.param<double>("max_pocket_dock_steering", max_pocket_dock_steering_, 0.2);

        /* ROS Publisher */
        pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        pub_local_path_ = nh_.advertise<nav_msgs::Path>("pallet_docking/local_ref_path", 1);
        pub_debug_ = nh_.advertise<std_msgs::Float32>("pallet_docking/debug", 1);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/pallet_docking/marker", 1);
        pub_pp_lookahead_distance_ = nh_.advertise<std_msgs::Float32>("pallet_docking/purepursuit_lookahead_distance", 1);
        pub_pp_lookahead_angle_ = nh_.advertise<std_msgs::Float32>("pallet_docking/purepursuit_lookahead_angle", 1);

        /* ROS Subscriber */
        sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/gazebo/forklift_controllers/odom", 1, &DockingControl::odomCallback, this);
        sub_ref_path_ = nh_.subscribe<nav_msgs::Path>("/pallet_docking/quintic_path", 1, &DockingControl::refPathCallback, this);
        sub_goal_pose_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/pallet_docking/goal_pose", 1, &DockingControl::goalPoseCallback, this);
        sub_controller_on_ = nh_.subscribe<std_msgs::Bool>("/pallet_docking/controller_turn_on", 1, &DockingControl::controllerOnCallback, this);
        sub_approaching_status_ = nh_.subscribe<std_msgs::Bool>("/pallet_docking/pallet_approaching_done", 1, &DockingControl::approachingStatusCallback, this);
        joint_states_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &DockingControl::JointStateCallBack, this);

        // dynamic reconfigure server
        srv_ = boost::make_shared <dynamic_reconfigure::Server<config> > (paramGet);
        dynamic_reconfigure::Server<config>::CallbackType f;
        f = boost::bind(&DockingControl::reconfigCallback, this, _1, _2);
        srv_->setCallback(f);

        /* Initialize parameters */
        init_reconfig_ = true;
        ref_path_avai_ = false;
        goal_avai_ = false;
        controller_on_.data = false;
        approaching_done_.data = false;
        pub_stop_ = false;

        fuzzy_controller = FuzzyControl(paramGet);
        pure_pursuit_control = PurePursuitController(paramGet);
    
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg_odom)
    {
        odom_sub_.pose = msg_odom->pose;
        odom_sub_.twist = msg_odom->twist;
        odom_avai_ = true;
    }

    void JointStateCallBack(const sensor_msgs::JointState::ConstPtr &msg)
    {
        for (int i = 0; i < msg->name.size(); i++) {
            if (msg->name.at(i) == "drive_steer_joint" ) {
            steering_sub_ = msg->position.at(i);
            ROS_DEBUG("JointStateCallBack: steering angle: %f", steering_sub_);
            break;
            }
        }
    }

    void refPathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        ref_path_ = *msg;
        ref_path_avai_ = true;
    }

    void goalPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        goal_pose_ = *msg;
        goal_avai_ = true;
    }

    void controllerOnCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        controller_on_.data = msg->data;
        ROS_INFO("CONTROLLER IS TRIGGERED: %d !", controller_on_.data);
    }

    void approachingStatusCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        approaching_done_.data = msg->data;
    }

    void resetController()
    {
        pure_pursuit_control.resetPP();
        ref_path_.poses.clear();
        ref_path_avai_ = false;
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.angular.z = 0;
        pub_cmd_vel_.publish(cmd_vel_);
        lpf_output_s_ = 0;
        steering_ = 0;
        lpf_output_v_ = 0;
        final_ref_vel_ = 0;
    }

    void controllerCal()
    {
        if (!controller_on_.data)
        {
            if (!pub_stop_)
            {
                resetController();
                pub_stop_ = true;
            }
            return;
        }
        pub_stop_ = false;
        if(!odom_avai_)
        {
            ROS_WARN("No Odometry!!!");
            return; 
        }
        if (!ref_path_avai_)
        {
            ROS_WARN("No ref path!!!");
            return;
        }
        if (!goal_avai_)
        {
            ROS_WARN("No goal!!!");
            return;
        }
        if (ref_path_.poses.size() > 1)
        {
            /*********** CONVERT GLOBAL PATH TO BASE_LINK PATH ***********/ 
            local_ref_path_.poses.clear();
            for (int i = 0; i < ref_path_.poses.size(); i++)
            {
                ref_path_.poses.at(i).header.stamp = ros::Time(0);
                try
                {
                   local_ref_path_.poses.push_back(tf_buffer_c.transform(ref_path_.poses.at(i), path_frame_, ros::Duration(1)));
                }
                catch (tf::LookupException ex)
                {
                    ROS_ERROR("%s",ex.what());
                }
            }
            pub_local_path_.publish(local_ref_path_);

            /*********** NEAREST POINT FINDING **********/
            int closest_index = 0;
            double min_dist = sqrt(local_ref_path_.poses.at(closest_index).pose.position.x*local_ref_path_.poses.at(closest_index).pose.position.x +
                                    local_ref_path_.poses.at(closest_index).pose.position.y*local_ref_path_.poses.at(closest_index).pose.position.y);
            for (int i = 2; i < local_ref_path_.poses.size(); i++)
            {
                double dist = sqrt(local_ref_path_.poses.at(i).pose.position.x*local_ref_path_.poses.at(i).pose.position.x +
                                local_ref_path_.poses.at(i).pose.position.y*local_ref_path_.poses.at(i).pose.position.y);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    closest_index = i;
                }
            }
            if (ref_path_.poses.size() - 1 < closest_index) closest_index = ref_path_.poses.size() - 1;
            ROS_DEBUG ("Nearest index: %d", closest_index);

            /******** STEERING CONTROL *********/  
            std_msgs::Float32 debug_p;
            debug_p.data = pure_pursuit_control.lateral_error_.data;
            pub_debug_.publish(debug_p);
            pure_pursuit_control.setOdom(odom_sub_);
            pure_pursuit_control.setRefPath(local_ref_path_);
            pure_pursuit_control.setRefVel(final_ref_vel_);
            pure_pursuit_control.setClosestPoint(closest_index);
            pure_pursuit_control.calControl();
            steering_angle_ = pure_pursuit_control.getSteeringAngle();

            // Smooth the steering output. Lowpass filter
            double cutoff_frequency_steering = 0.5;
            e_pow_s_ = 1 - exp(-0.025 * 2 * M_PI * cutoff_frequency_steering);
            lpf_output_s_ += (steering_angle_ - lpf_output_s_) * e_pow_s_;

            steering_ = lpf_output_s_;

            // Visualize PP lookahead distance and lookahead angle
            std_msgs::Float32 pp_lkh_distance, pp_lkh_angle;
            pp_lkh_distance.data = pure_pursuit_control.look_ahead_distance_;
            pub_pp_lookahead_distance_.publish(pp_lkh_distance);

            pp_lkh_angle.data = pure_pursuit_control.alpha_;
            pub_pp_lookahead_angle_.publish(pp_lkh_angle);

            /******** VELOCITY CONTROL ****/
            int lk_index;
            double fuzzy_lk_dis = fuzzy_lookahead_dis_;
            double max_dist = sqrt(local_ref_path_.poses.at(local_ref_path_.poses.size()-1).pose.position.x*local_ref_path_.poses.at(local_ref_path_.poses.size()-1).pose.position.x
                                + local_ref_path_.poses.at(local_ref_path_.poses.size()-1).pose.position.y*local_ref_path_.poses.at(local_ref_path_.poses.size()-1).pose.position.y);
            
            if (fuzzy_lk_dis > max_dist) fuzzy_lk_dis = max_dist;
            for (lk_index = closest_index; lk_index < local_ref_path_.poses.size(); lk_index++)
            {
                if (abs(fuzzy_lk_dis) <= sqrt(local_ref_path_.poses.at(lk_index).pose.position.x*local_ref_path_.poses.at(lk_index).pose.position.x
                                            + local_ref_path_.poses.at(lk_index).pose.position.y*local_ref_path_.poses.at(lk_index).pose.position.y)) break;  
            }
            
            if (abs(local_ref_path_.poses.at(local_ref_path_.poses.size()-1).pose.position.x) < fuzzy_lk_dis)
                lk_index = local_ref_path_.poses.size()-1;
            
            fuzzy_controller.inputSolveGoal(abs(fuzzy_lk_dis));
            fuzzy_controller.inputsolveSteering(0.0);
            fuzzy_controller.inputResults();
            ref_velocity_ = fuzzy_controller.cal_fuzzy_output() * cos(steering_);
            
            if (local_ref_path_.poses.at(lk_index).pose.position.x < 0)
                if (ref_velocity_ > 0) ref_velocity_ = -ref_velocity_;
            
            // Smooth the velocity output
            double cutoff_frequency_v = 0.2;
            e_pow_v_ = 1 - exp(-0.025 * 2 * M_PI * cutoff_frequency_v);
            lpf_output_v_ += (ref_velocity_ - lpf_output_v_) * e_pow_v_;
            final_ref_vel_ = lpf_output_v_;

            /************ ANGULAR VELOCITY LIMIT ************/
            if (steering_ != 0)
            {
                double max_vel_limit = abs(max_angular_vel_ * l_wheelbase_ / tan(steering_));
                if (abs(final_ref_vel_) >= max_vel_limit) final_ref_vel_ = max_vel_limit * (final_ref_vel_/abs(final_ref_vel_));
            }
            if (abs(final_ref_vel_) < min_vel_) final_ref_vel_ = std::copysign(min_vel_, final_ref_vel_);


            /*********** LIMIT CONTROL SIGNAL WHEN DOCKING TO POCKET FOR SAFETY ************/
            if ((approaching_done_.data && final_ref_vel_ >= 0) ||
                (!approaching_done_.data && final_ref_vel_ < 0)) 
            {
                if (abs(steering_) >= max_pocket_dock_steering_) steering_ = max_pocket_dock_steering_*(abs(steering_)/steering_);
                if (abs(final_ref_vel_) >= max_pocket_dock_vel_) final_ref_vel_ = max_pocket_dock_vel_ * (final_ref_vel_/abs(final_ref_vel_));
            } 

            /*********** NAN OUTPUT HANDLE *************/
            if(isnan(steering_) || isnan(final_ref_vel_))
            {
                ROS_ERROR("NAN NUMBER ! RESET CONTROLLER");
                controller_on_.data = false;
                pub_stop_ = false;
                resetController();
                return;
            }

            cmd_vel_.linear.x = final_ref_vel_;
            cmd_vel_.angular.z = steering_;
            pub_cmd_vel_.publish(cmd_vel_);

            /********** VISUALIZE LOOKAHEAD POINT MARKER ************/
            visualization_msgs::Marker points;
            points.header.frame_id =  path_frame_;
            points.header.stamp = ros::Time::now();
            points.action = visualization_msgs::Marker::ADD;
            points.pose.orientation.w = 1.0;
            points.type = visualization_msgs::Marker::POINTS;
            points.color.b = 1.0f;
            points.color.a = 1.0;
            points.scale.x = 0.05;
            points.scale.y = 0.05;

            geometry_msgs::Point point_fuzzy;
            geometry_msgs::Point point_pp;
            geometry_msgs::Point closest_point;
            point_fuzzy.x = local_ref_path_.poses.at(lk_index).pose.position.x;
            point_fuzzy.y = local_ref_path_.poses.at(lk_index).pose.position.y;
            point_fuzzy.z = 0;
            point_pp.x = local_ref_path_.poses.at(pure_pursuit_control.point_index_).pose.position.x;
            point_pp.y = local_ref_path_.poses.at(pure_pursuit_control.point_index_).pose.position.y;
            point_pp.z = 0;
            closest_point.x = local_ref_path_.poses.at(closest_index).pose.position.x;
            closest_point.y = local_ref_path_.poses.at(closest_index).pose.position.y;
            closest_point.z = 0;

            std::vector<geometry_msgs::Point> my_points;
            my_points.push_back(point_fuzzy);
            my_points.push_back(point_pp);
            my_points.push_back(closest_point);

            for (int i = 0; i < my_points.size(); i++)
            {
                std_msgs::ColorRGBA c;
                if( i == 0)
                {
                    c.r = 1.0;
                    c.g = 1.0;
                }
                else if(i == 1)
                    c.b = 1.0;
                else
                    c.r = 1.0;
                c.a = 1.0;

                points.points.push_back(my_points[i]);
                points.colors.push_back(c);
            }
            marker_pub_.publish(points);
        }

    }

    void reconfigCallback(pallet_docking_xsquare::purePursuitReconfigConfig &config, uint32_t level)
    {
        if (init_reconfig_)
        {
            ROS_INFO("Get init param from launch file");
            init_reconfig_ = false;
        }
        else
        {
            ROS_INFO("Reconfigure Request");

            look_ahead_ = config.look_ahead;
            alpha_offset_ = config.alpha_offset;
            max_steering_ = config.max_steering;
            min_steering_ = config.min_steering;
            max_vel_ = config.max_vel;
            min_vel_ = config.min_vel;
            goal_correct_yaw_ = config.goal_correct_yaw;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "docking_control");
    ros::NodeHandle n ("~");
    ros::Rate loop_rate(50);

    DockingControl docking_controller(n);

    while(ros::ok())
    {
        docking_controller.controllerCal();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
