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
    nh_.param<double>("max_acc", max_acc_, 0.5);
    nh_.param<double>("min_highest_acc", min_highest_acc_, 0.1);
    nh_.param<double>("max_jerk", max_jerk_, 0.5);
    nh_.param<double>("min_t", min_t_, 0.5);
    nh_.param<double>("max_t", max_t_, 10.0);
    nh_.param<double>("dt", dt_, 0.1);

    nh_.param<double>("max_starting_vel", max_starting_vel_, 0.1);
    nh_.param<double>("max_ending_vel", max_ending_vel_, 0.3);
    nh_.param<double>("k_ending_lat", k_ending_lat_, 0.8);
    nh_.param<double>("k_ending_angle", k_ending_angle_, 0.5);

    /* ROS Publisher */
    pub_quintic_pose_ = nh_.advertise<geometry_msgs::PoseArray>("/pallet_docking/quintic_pose", 1);
    pub_quintic_path_ = nh_.advertise<nav_msgs::Path>("/pallet_docking/quintic_path", 1);
    pub_quintic_local_path_ = nh_.advertise<nav_msgs::Path>("/pallet_docking/quintic_local_path", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/pallet_docking/visualization_marker", 10);

    /* Initialize parameters */
    quintic_pose_.header.frame_id = path_frame_;
    quintic_path_.header.frame_id = path_frame_;

    set_param_ = false;
    resetPlanner();
}

void QuinticPlanner::setParams(double sx, double sy, double syaw, double sv, double sa,
                    double gx, double gy, double gyaw, double gv, double ga)
{
    sx_ = sx,   sy_ = sy,   syaw_ = syaw,   sv_ = sv,   sa_ = sa;
    gx_ = gx,   gy_ = gy,   gyaw_ = gyaw,   gv_ = gv,   ga_ = ga;
    set_param_ = true;
}

void QuinticPlanner::setFrame(std::string local_frame, std::string global_frame)
{
    global_frame_ = global_frame;
    path_frame_ = local_frame;
}

void QuinticPlanner::genPath(std::vector<geometry_msgs::PoseStamped> waypoints)
{  
    size_t n_segments = waypoints.size() - 1;

    quintic_path_.header.frame_id = path_frame_;
    quintic_path_.header.stamp = ros::Time::now();
    quintic_path_.poses.clear();

    quintic_pose_.header.frame_id = path_frame_;
    quintic_pose_.header.stamp = ros::Time::now();
    quintic_pose_.poses.clear();
    
    for (size_t i = 0; i < n_segments; i++)
    {
        sx_ = waypoints[i].pose.position.x;
        sy_ = waypoints[i].pose.position.y;
        gx_ = waypoints[i + 1].pose.position.x;
        gy_ = waypoints[i + 1].pose.position.y;
        syaw_ = tf::getYaw(waypoints[i].pose.orientation);
        gyaw_ = tf::getYaw(waypoints[i + 1].pose.orientation);

        time_.clear();
        rx_.clear(); ry_.clear(); ryaw_.clear();
        rv_.clear(); ra_.clear(); rax_.clear(); ray_.clear();
        rj_.clear();

        // Auto adjust the path derivative
        // gv_ = bilinearInterpolation(0.00, 0.00, 0.3, 0.3, 0, 0.3, 0.3, 0.3, abs(gy_), abs(gyaw_));
        // sv_ = gv_;
        // if(sv_ > 0.3) sv_ = 0.3;
        // if(gv_ > 0.3) gv_ = 0.3;

        
        sv_ = bilinearInterpolation(0.00, 0.00, 0.3, 0.3, 
                                    0, max_starting_vel_, max_starting_vel_, max_starting_vel_, 
                                    abs(gy_), abs(gyaw_));
        if (sv_ > max_starting_vel_) sv_ = max_starting_vel_;

        gv_ = std::min(max_ending_vel_, k_ending_lat_ * abs(gy_) + k_ending_angle_ * abs(gyaw_));
    
        // ROS_INFO("gv cal: %f, gv_: %f", k1 * abs(gy_) + k2 * abs(gyaw_), gv_);
        if (gx_ < 0)
        {
            sv_ = -sv_;
            gv_ = -gv_;
        }
        if (i > 0) sv_ = gv_;

        double vxs = sv_ * cos(syaw_);
        double vys = sv_ * sin(syaw_);
        double vxg = gv_ * cos(gyaw_);
        double vyg = gv_ * sin(gyaw_);
        
        double axs = sa_* cos(syaw_);
        double ays = sa_* sin(syaw_);
        double axg = ga_* cos(gyaw_);
        double ayg = ga_* sin(gyaw_);

        segment_quintic_path_.header.frame_id = path_frame_;
        segment_quintic_path_.header.stamp = ros::Time::now();
        segment_quintic_path_.poses.clear();
        segment_quintic_pose_.header.frame_id = path_frame_;
        segment_quintic_pose_.header.stamp = ros::Time::now();
        segment_quintic_pose_.poses.clear();

        geometry_msgs::Pose pose_tmp;
        geometry_msgs::PoseStamped pose_stamp_tmp;

        for (double t_to_goal = min_t_; t_to_goal <= max_t_; t_to_goal = t_to_goal + min_t_)
        {
            QuinticPolynominal xqp(sx_, vxs, axs, gx_, vxg, axg, t_to_goal);
            QuinticPolynominal yqp(sy_, vys, ays, gy_, vyg, ayg, t_to_goal);

            time_.clear();
            rx_.clear(); ry_.clear(); ryaw_.clear();
            rv_.clear(); ra_.clear(); rax_.clear(); ray_.clear();
            rj_.clear();

            segment_quintic_path_.poses.clear();
            segment_quintic_pose_.poses.clear();
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
                if (vx < 0) yaw = atan2(-vy,abs(vx));
                if ((i == n_segments - 1) && 
                    abs(abs(gx_) - abs(xqp.cal_point(t))) < 0.1) yaw = gyaw_;
                ryaw_.push_back(yaw);
                
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

                segment_quintic_pose_.poses.push_back(pose_tmp);
                segment_quintic_path_.poses.push_back(pose_stamp_tmp);
            }

            if ((*std::max_element(ra_.begin(), ra_.end()) < max_acc_) &&
                    (*std::min_element(ra_.begin(), ra_.end()) > -1*max_acc_) &&
                (*std::max_element(ra_.begin(), ra_.end()) > min_highest_acc_) &&
                (*std::min_element(ra_.begin(), ra_.end()) < -min_highest_acc_) &&
                (*std::max_element(rj_.begin(), rj_.end()) < max_jerk_) &&
                    (*std::min_element(rj_.begin(), rj_.end()) > -1*max_jerk_))
            {
                // ROS_INFO("Path found");
                //ROS_INFO("Time move %f", t_to_goal);
                // ROS_INFO(" Path: Max acc: %f", *std::max_element(ra_.begin(), ra_.end()));
                // ROS_INFO(" Path: Min acc: %f", *std::min_element(ra_.begin(), ra_.end()));
                segment_path_feasible_ = true;
                break;
            } 
        }
        if (segment_path_feasible_)
        {
            quintic_path_.poses.insert(quintic_path_.poses.end(), segment_quintic_path_.poses.begin(), segment_quintic_path_.poses.end());
            quintic_pose_.poses.insert(quintic_pose_.poses.end(), segment_quintic_pose_.poses.begin(), segment_quintic_pose_.poses.end());    
            segment_path_feasible_ = false;
            if (i == n_segments - 1)
            {
                path_feasible_ = true;
                break;
            }
            else continue;
        }
        if (!segment_path_feasible_)
        {
            path_feasible_ = false;
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

        pub_quintic_local_path_.publish(quintic_path_);
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