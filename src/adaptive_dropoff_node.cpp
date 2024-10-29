#include "adaptive_dropoff_local_planner.h"
#include "nav_msgs/Odometry.h"
#include "rack_detection_msg/RackDeviation.h"

using namespace techtile;

geometry_msgs::TwistStamped velocity;
nav_msgs::Odometry odom;

rack_detection_msg::RackDeviation rack_deviation;
bool rack_deviation_avai_{false};

void wheelOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom = *msg;
    velocity.twist.linear.x = -odom.twist.twist.linear.x;
}

void rackDeviationCallback(const rack_detection_msg::RackDeviation::ConstPtr &msg)
{
    rack_deviation = *msg;
    rack_deviation_avai_ =  true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "adaptive_dropoff");
    ros::NodeHandle n ("~");
    ros::Rate loop_rate(10);

    double approach_ref_x, approach_ref_y, approach_ref_angle;
    double docking_ref_x, docking_ref_y, docking_ref_angle;
    n.param<double>("approach_ref_x", approach_ref_x, 2.5);
    n.param<double>("approach_ref_y", approach_ref_y, 0.0);
    n.param<double>("approach_ref_angle", approach_ref_angle, 0.0);
    n.param<double>("docking_ref_x", docking_ref_x, 2.5);
    n.param<double>("docking_ref_y", docking_ref_y, 0.0);
    n.param<double>("docking_ref_angle", docking_ref_angle, 0.0);

    ros::Publisher pub_cmd;
    pub_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel_raw", 1);

    ros::Subscriber sub_wheel_odom = n.subscribe("/wheel_odom", 1, wheelOdomCallback); 
    ros::Subscriber sub_rack_deviation_ = n.subscribe("/rack_detection/fusioned_rack_deviation", 10, rackDeviationCallback);

    AdaptiveDockingLocalPlanner docking_local_planner(n);
    

    tf2_ros::Buffer tf_buffer_(ros::Duration(1.0));
    tf2_ros::TransformListener tf_listener_(tf_buffer_);
    ROS_INFO("Start initialize");
    docking_local_planner.initialize(tf_buffer_, 1.0);
    docking_local_planner.setLocalFrame("base_link_p");
    docking_local_planner.setGlobalFrame("odom");

    geometry_msgs::PoseStamped approach_pose;
    geometry_msgs::PoseStamped docking_pose;
    std_msgs::Header plan_header;
    

    geometry_msgs::Twist cmd_vel;

    int number_success = 0;
    int number_failed = 0;
    uint32_t control_exec;

    bool setup_goal_pose_{false};
    
    double depth_dsi;

    std::vector<geometry_msgs::PoseStamped> goal_poses;
    while(ros::ok())
    {  
        // if (!rack_deviation_avai_) {
        //     ROS_INFO("waiting for rack_deviation");
        //     ros::spinOnce();
        //     loop_rate.sleep();
        //     continue;
        // }

        double rack_angle = rack_deviation.orientation_deviation;
        rack_deviation_avai_ = false;
        if (!setup_goal_pose_)
        {
            
            approach_pose.header.frame_id = "base_link_p";
            approach_pose.header.stamp = ros::Time::now();
            approach_pose.pose.position.x = approach_ref_x;
            approach_pose.pose.position.y = approach_ref_y;
            approach_pose.pose.orientation = rpyToQuaternion(0, 0, approach_ref_angle);

            docking_pose.header.frame_id = "base_link_p";
            docking_pose.header.stamp = ros::Time::now();
            depth_dsi = docking_ref_x + number_success%6;
            // depth_dsi = docking_ref_x ;
            docking_pose.pose.position.x = depth_dsi;
            docking_pose.pose.position.y = docking_ref_y;
            docking_pose.pose.orientation = rpyToQuaternion(0, 0, docking_ref_angle);

            plan_header.frame_id = "map";

            goal_poses.clear();
            goal_poses.push_back(docking_pose);

            if (!docking_local_planner.setPlan(plan_header, goal_poses))
            {
                ROS_WARN("Cannot setup pose");
                return 0;
            }
            setup_goal_pose_ = true;
        }
        // ROS_INFO("lateral_deviation: %f", rack_deviation.lateral_deviation);
        // ROS_INFO("approach_pose lateral: %f", approach_pose.pose.position.y );
        // ROS_INFO("docking_pose lateral: %f", docking_pose.pose.position.y );
        // ROS_INFO("rack_angle: %f", rack_angle);

        geometry_msgs::PoseStamped pose;
        std::string msg;

        // ROS_INFO("Docking result: %d", docking_local_planner.getDockingResult());
        //bool approaching_reached = docking_local_planner.IsApproachingReached(0.02, 0.015, 0.02);
        bool goal_reached = docking_local_planner.IsGoalReached(0.01, 0.015, 0.02);
        // bool approaching_reached = docking_local_planner.IsApproachingReached(0.03, 0.015, 0.03);
        if (!goal_reached) 
            control_exec = docking_local_planner.ExecuteControlLoop(pose,
                                                                    velocity, 
                                                                    cmd_vel,
                                                                    msg);
        if (goal_reached || control_exec == mbf_msgs::ExePathResult::FAILURE)
        {
            if (goal_reached)
            {
                number_success++;
                ROS_INFO("DOCKING SUCCESS TIME: %d", number_success);
            }
            else 
            {
                number_failed++;
                ROS_INFO("DOCKING FAILURE TIME: %d", number_failed);
            }

            // move back to position
            geometry_msgs::Twist cmd_vel_back;
            double move_back_sp = -0.3;
            ros::Time start = ros::Time::now();
            while ((ros::Time::now().toSec() - start.toSec()) < (double)(depth_dsi/abs(move_back_sp) ))
            {
                cmd_vel_back.linear.x = move_back_sp;
                pub_cmd.publish(cmd_vel_back);
            }
            
            cmd_vel_back.linear.x = 0.0;
            pub_cmd.publish(cmd_vel_back);
            ros::Duration(1).sleep();

           if (!docking_local_planner.setPlan(plan_header, goal_poses))
           {
                ROS_WARN("Cannot setup pose");
                return 0;
           }
           setup_goal_pose_ = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
  
  return 0;
}
