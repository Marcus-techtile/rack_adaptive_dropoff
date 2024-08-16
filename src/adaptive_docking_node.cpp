#include "adaptive_docking_local_planner.h"
#include "nav_msgs/Odometry.h"

using namespace techtile;

geometry_msgs::TwistStamped velocity;
nav_msgs::Odometry odom;

void wheelOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom = *msg;
    velocity.twist.linear.x = -odom.twist.twist.linear.x;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "docking_local_planner");
    ros::NodeHandle n ("~");
    ros::Rate loop_rate(20);

    ros::Publisher pub_cmd;
    pub_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel_raw", 1);

    ros::Subscriber sub_wheel_odom = n.subscribe("/wheel_odom", 1, wheelOdomCallback);

    AdaptiveDockingLocalPlanner docking_local_planner(n);
    

    tf2_ros::Buffer tf_buffer_(ros::Duration(1.0));
    tf2_ros::TransformListener tf_listener_(tf_buffer_);
    ROS_INFO("Start initialize");
    docking_local_planner.initialize(tf_buffer_, 1.0);
    docking_local_planner.setLocalFrame("base_link");
    docking_local_planner.setGlobalFrame("odom");

    geometry_msgs::PoseStamped approach_pose;
    geometry_msgs::PoseStamped docking_pose;
    
    
    approach_pose.header.frame_id = "base_link";
    approach_pose.header.stamp = ros::Time::now();
    approach_pose.pose.position.x = 1.5;
    approach_pose.pose.position.y = 0.5;
    approach_pose.pose.orientation.x = 0.0;
    approach_pose.pose.orientation.y = 0.0;
    approach_pose.pose.orientation.z = 0.0;
    approach_pose.pose.orientation.w =  1.0;

    docking_pose.header.frame_id = "base_link";
    docking_pose.header.stamp = ros::Time::now();
    docking_pose.pose.position.x = 1.5;
    docking_pose.pose.position.y = 0.3;
    docking_pose.pose.orientation.x = 0.0;
    docking_pose.pose.orientation.y = 0.0;
    docking_pose.pose.orientation.z = 0.0;
    docking_pose.pose.orientation.w =  1.0;

    

    std_msgs::Header plan_header;
    plan_header.frame_id = "map";
    if (!docking_local_planner.setPlan(plan_header, approach_pose, docking_pose))
    {
        ROS_WARN("Cannot setup pose");
        return 0;
    }
    geometry_msgs::Twist cmd_vel;

    int number_success = 0;
    int number_failed = 0;
    uint32_t control_exec;
    while(ros::ok())
    {  
        geometry_msgs::PoseStamped pose;
        std::string msg;

        // ROS_INFO("Docking result: %d", docking_local_planner.getDockingResult());
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
            ros::Time start = ros::Time::now();
            while ((ros::Time::now() - start).toSec() < 5.0)
            {
                cmd_vel_back.linear.x = -0.3;
                pub_cmd.publish(cmd_vel_back);
            }
            
            cmd_vel_back.linear.x = 0.0;
            pub_cmd.publish(cmd_vel_back);
            ros::Duration(1).sleep();

            if (!docking_local_planner.setPlan(plan_header, approach_pose, docking_pose))
            {
                ROS_WARN("Cannot setup pose");
                return 0;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
  
  return 0;
}