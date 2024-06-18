#include "quintic_planner.h"

QuinticPolynominal::QuinticPolynominal(double x0, double v0, double a0, double xf, double vf, double af, double tf)
{
    a_coef = std::vector<double>(6, 0.0);
    tf_2 = tf*tf;
    tf_3 = tf*tf*tf;
    tf_4 = tf*tf*tf*tf;
    tf_5 = tf*tf*tf*tf*tf;

    a_matrix << tf_3, tf_4, tf_5,
                3*tf_2, 4*tf_3, 5*tf_4,
                6*tf, 12*tf_2, 20*tf_3;

    b_matrix << xf - x0 - v0*tf - a0*tf_2/2,
                vf - v0 - a0*tf,
                af - a0;

    a_invese = a_matrix.inverse();
    x_matrix = a_invese * b_matrix;

    a_coef[0] = x0;
    a_coef[1] = v0;
    a_coef[2] = ((1/2) * a0);
    a_coef[3] = (x_matrix(0));
    a_coef[4] = (x_matrix(1));
    a_coef[5] = (x_matrix(2));
}

double QuinticPolynominal::cal_point(double ti)
{
    xt = a_coef[0] + a_coef[1]*ti + a_coef[2]*ti*ti + a_coef[3]*ti*ti*ti 
            + a_coef[4]*ti*ti*ti*ti + a_coef[5]*ti*ti*ti*ti*ti;
    return xt;
}

double QuinticPolynominal::cal_vel(double ti)
{
    dxt = a_coef[1] + 2*a_coef[2]*ti + 3*a_coef[3]*ti*ti 
            + 4*a_coef[4]*ti*ti*ti + 5*a_coef[5]*ti*ti*ti*ti;    
    return dxt;
}

double QuinticPolynominal::cal_acc(double ti)
{
    ddxt = 2*a_coef[2] + 6*a_coef[3]*ti + 12*a_coef[4]*ti*ti + 20*a_coef[5]*ti*ti*ti;    
    return ddxt;   
}

double QuinticPolynominal::cal_jerk(double ti)
{
    dddxt = 6*a_coef[3] + 24*a_coef[4]*ti + 60*a_coef[5]*ti*ti;    
    return ddxt;   
}

/* Quintic Planner Class */
QuinticPlanner::QuinticPlanner(ros::NodeHandle &nh, tf2_ros::Buffer &tf, double sec):nh_(nh), tf_buffer(tf), tf_time_out_{1.0}
{
    /* Get Param */
    nh_.param<double>("starting_vel", starting_vel_, 0.01);
    nh_.param<double>("starting_acc", starting_acc_, 0.01);
    nh_.param<double>("stopping_vel", stopping_vel_, 0.01);
    nh_.param<double>("stopping_acc", stopping_acc_, -0.01);
    nh_.param<double>("max_yaw_rate", max_yaw_rate_, 0.1);
    nh_.param<double>("max_path_curvature", max_curv_, 2.0);
    nh_.param<double>("max_acc", max_accel_, 0.5);
    nh_.param<double>("max_ax", max_ax_, 0.5);
    nh_.param<double>("max_ay", max_ay_, 0.5);
    nh_.param<double>("max_jerk", max_jerk_, 0.5);
    nh_.param<double>("min_t", min_t_, 0.5);
    nh_.param<double>("max_t", max_t_, 10.0);
    nh_.param<double>("dt", dt_, 0.1);

    nh_.param<double>("fake_goal_x", fake_goal_x_, 3.5);
    nh_.param<double>("fake_goal_y", fake_goal_y_, 2.0);
    nh_.param<double>("fake_goal_yaw", fake_goal_yaw_, -0.087);

    /* ROS Publisher */
    pub_quintic_pose_ = nh_.advertise<geometry_msgs::PoseArray>("/pallet_docking/quintic_pose", 1);
    pub_quintic_path_ = nh_.advertise<nav_msgs::Path>("/pallet_docking/quintic_path", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/pallet_docking/visualization_marker", 10);

    /* Initialize parameters */
    quintic_pose_.header.frame_id = path_frame_;
    quintic_path_.header.frame_id = path_frame_;

    set_param_ = false;
    // controller_on_.data = false;

    resetPlanner();
}

void QuinticPlanner::setParams(double sx, double sy, double syaw, double sv, double sa,
                    double gx, double gy, double gyaw, double gv, double ga)
{
    sx_ = sx,   sy_ = sy,   syaw_ = syaw,   sv_ = sv,   sa_ = sa;
    gx_ = gx,   gy_ = gy,   gyaw_ = gyaw,   gv_ = gv,   ga_ = ga;
    set_param_ = true;
}

void QuinticPlanner::genPath()
{  
    time_.clear();
    rx_.clear(); ry_.clear(); ryaw_.clear(), r_vyaw_.clear();
    rv_.clear(); ra_.clear(); rax_.clear(); ray_.clear();
    rj_.clear();
    curv_.clear();

    double vxs = sv_ * cos(syaw_);
    double vys = sv_ * sin(syaw_);
    double vxg = gv_ * cos(gyaw_);
    double vyg = gv_ * sin(gyaw_);
    
    double axs = sa_* cos(syaw_);
    double ays = sa_* sin(syaw_);
    double axg = ga_* cos(gyaw_);
    double ayg = ga_* sin(gyaw_);

    // reinitialize path
    quintic_path_.header.frame_id = path_frame_;
    quintic_path_.header.stamp = ros::Time::now();
    quintic_path_.poses.clear();

    quintic_pose_.header.frame_id = path_frame_;
    quintic_pose_.header.stamp = ros::Time::now();
    quintic_pose_.poses.clear();

    geometry_msgs::Pose pose_tmp;
    geometry_msgs::PoseStamped pose_stamp_tmp;

    for (double t_to_goal = min_t_; t_to_goal <= max_t_; t_to_goal = t_to_goal + min_t_)
    {
        QuinticPolynominal xqp(sx_, vxs, axs, gx_, vxg, axg, t_to_goal);
        QuinticPolynominal yqp(sy_, vys, ays, gy_, vyg, ayg, t_to_goal);

        time_.clear();
        rx_.clear(); ry_.clear(); ryaw_.clear(), r_vyaw_.clear();
        rv_.clear(); ra_.clear(); rax_.clear(); ray_.clear();
        rj_.clear();
        curv_.clear();

        quintic_path_.poses.clear();
        quintic_pose_.poses.clear();
        // ROS_INFO(" ");
        // ROS_INFO("TIME TO GOAL: %f", t_to_goal);
        for (double t = 0.0; t <= t_to_goal; t = t + dt_)
        {
            time_.push_back(t);
            rx_.push_back(xqp.cal_point(t));
            ry_.push_back(yqp.cal_point(t));

            double vx = xqp.cal_vel(t);
            double vy = yqp.cal_vel(t);
            double v = sqrt(vx*vx + vy*vy);
            rv_.push_back(v);

            double yaw = atan2(vy, vx);
            if (vx < 0) yaw = atan2(-vy, abs(vx));

            ryaw_.push_back(yaw);

            // ROS_INFO("YAW: %f", yaw);
            
            if (t == 0) r_vyaw_.push_back(0.0);
            else
            {
                double pre_yaw = atan2(yqp.cal_vel(t-dt_), abs(xqp.cal_vel(t-dt_)));
                if (t == dt_) pre_yaw = 0;
                r_vyaw_.push_back(yaw - pre_yaw); 
                // if (abs(yaw - pre_yaw) > max_yaw_rate_) ROS_WARN("Yaw rate too fast: %f !!!", yaw - pre_yaw);
            } 

            double lk_dis = sqrt(xqp.cal_point(t)*xqp.cal_point(t) + yqp.cal_point(t)*yqp.cal_point(t));
            if (lk_dis == 0) curv_.push_back(0);
            else
            {   
                double cur_curv = (2*sin(yaw))/lk_dis;
                curv_.push_back(cur_curv);
                // ROS_INFO("Path curvature: %f", cur_curv);
                // if (cur_curv > max_curv_) ROS_WARN("High curvature: %f !!!", cur_curv);
            } 
            
            double ax = xqp.cal_acc(t);
            double ay = yqp.cal_acc(t);
            rax_.push_back(ax);
            ray_.push_back(ay);
            double a = sqrt(ax*ax + ay*ay);
            if (rv_.size() >= 2 && (rv_.at(rv_.size()-1) - rv_.at(rv_.size()-2) < 0)) a = -1*a;
            ra_.push_back(a);

            double jx = xqp.cal_jerk(t);
            double jy = yqp.cal_jerk(t);
            double j = sqrt(jx*jx + jy*jy);
            if (ra_.size() >= 2 && (ra_.at(ra_.size() - 1) - ra_.at(ra_.size() - 2))) j = -1*j;
            rj_.push_back(j);

            /* Add result to path */
            // Pose
            pose_tmp.position.x = xqp.cal_point(t);
            pose_tmp.position.y = yqp.cal_point(t);
            pose_tmp.orientation = rpyToQuaternion(0, 0, yaw);

            //path
            pose_stamp_tmp.header = quintic_path_.header;
            pose_stamp_tmp.pose.position.x = xqp.cal_point(t);
            pose_stamp_tmp.pose.position.y = yqp.cal_point(t);
            pose_stamp_tmp.pose.orientation = rpyToQuaternion(0, 0, yaw);

            quintic_pose_.poses.push_back(pose_tmp);
            quintic_path_.poses.push_back(pose_stamp_tmp);
        }

        // ROS_INFO("Time move %f", t_to_goal);

        if ((*std::max_element(curv_.begin(), curv_.end()) < max_curv_) &&
                (*std::min_element(curv_.begin(), curv_.end()) > -1*max_curv_) &&
            (*std::max_element(r_vyaw_.begin(), r_vyaw_.end()) < max_yaw_rate_) &&
                (*std::min_element(r_vyaw_.begin(), r_vyaw_.end()) > -1*max_yaw_rate_) &&
            (*std::max_element(ra_.begin(), ra_.end()) < max_accel_) &&
                (*std::min_element(ra_.begin(), ra_.end()) > -1*max_accel_) &&
            (*std::max_element(rj_.begin(), rj_.end()) < max_jerk_) &&
                (*std::min_element(rj_.begin(), rj_.end()) > -1*max_jerk_))
        {
            // ROS_INFO("Path found");
            path_feasible_ = true;
            break;
        } 
    }
    // ROS_INFO("PATH FEASIBLE: %d", path_feasible_);
    if (!path_feasible_) 
    {
        // ROS_WARN("PATH IS NOT FEASIBLE !!!");
        path_avai_ = false;
        path_feasible_ = false;
        return;
    }
    else
    {
        /* Convert path to global frame global_frame_ */ 
        global_quintic_path_.poses.clear();
        global_quintic_path_.header.frame_id = global_frame_;
        for (int i = 0; i < quintic_path_.poses.size(); i++)
        {
            quintic_path_.poses.at(i).header.stamp = ros::Time(0);
            try
            {
                global_quintic_path_.poses.push_back(tf_buffer.transform(quintic_path_.poses.at(i), global_frame_, ros::Duration(1)));
            }
            catch (tf2::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
            }
        }

        pub_quintic_path_.publish(global_quintic_path_);
        pub_quintic_pose_.publish(quintic_pose_);
        path_avai_ = true;
    } 
    

}

void QuinticPlanner::resetPlanner()
{
    set_param_ = false;
    path_feasible_ = false;
    path_avai_ = false;
    quintic_path_.poses.clear();
    quintic_pose_.poses.clear();
    // pub_quintic_path_.publish(quintic_path_);
    // pub_quintic_pose_.publish(quintic_pose_);
}

void QuinticPlanner::visualize(geometry_msgs::PoseStamped pallet_pose)
{
    geometry_msgs::PoseStamped local_pallet_pose_vs;
    /* Transform pallet pose to path_frame */
    pallet_pose.header.stamp = ros::Time(0);

    try
    {
        local_pallet_pose_vs = tf_buffer.transform(pallet_pose, path_frame_, ros::Duration(1));
    }
    catch (tf2::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    visualization_msgs::Marker points, line_list;
    points.id = 0;
    line_list.id = 1;
    points.header.frame_id = line_list.header.frame_id =  path_frame_;
    points.header.stamp = line_list.header.stamp = ros::Time::now();
    points.action  = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    points.type = visualization_msgs::Marker::POINTS;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line list is red
    line_list.color.b = 1.0;
    line_list.color.a = 1.0;

    points.scale.x = 0.05;
    points.scale.y = 0.05;

    line_list.scale.x = 0.05;

    geometry_msgs::Point point;
    geometry_msgs::Point point_sec;

    points.points.clear();
    line_list.points.clear();

        //visualize
    point.x = local_pallet_pose_vs.pose.position.x;
    point.y = local_pallet_pose_vs.pose.position.y;
    point.z = 0;
    points.points.push_back(point);
    line_list.points.push_back(point);

    point_sec.x = gx_;
    point_sec.y = gy_;
    point_sec.z = 0;
    points.points.push_back(point_sec);
    line_list.points.push_back(point_sec);

    marker_pub_.publish(points);
    marker_pub_.publish(line_list);
}