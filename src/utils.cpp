#include "utils.h"

geometry_msgs::Quaternion rpyToQuaternion(double roll, double pitch, double yaw)
{
    tf2::Quaternion tf_quaternion;
    tf_quaternion.setRPY(roll, pitch, yaw);
    tf_quaternion = tf_quaternion.normalize();

    geometry_msgs::Quaternion output;
    tf2::convert(tf_quaternion, output);
    return output;
}

void quaternionToRPY(geometry_msgs::Quaternion quat, double &roll, double &pitch, double &yaw)
{
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}

double normalize_M_PI(const double &angle) 
{
    double a = angle;
    while (a > M_PI/2.)
        a -= M_PI;
    while (a < -M_PI/2.)
        a += M_PI;
    return a;
}

std::vector<double> linspace(double start, double stop, int num, bool endpoint = true) 
{
    std::vector<double> result;
    if (num <= 0) return result; 
    
    if (num == 1) {
        result.push_back(start);
        return result;
    }

    double step = (stop - start) / (endpoint ? (num - 1) : num);

    for (int i = 0; i < num; ++i) {
        result.push_back(start + step * i);
    }

    if (endpoint) result.back() = stop; 
    
    return result;
}

double linearInterpolation(double x0, double y0, double x1, double y1, double x) {
    return y0 + ((y1 - y0) / (x1 - x0)) * (x - x0);
}

double bilinearInterpolation(double x1, double y1, double x2, double y2, 
                             double fQ11, double fQ21, double fQ12, double fQ22, 
                             double x, double y) {
    // Interpolate in the x direction
    double fR1 = fQ11 + (x - x1) * ((fQ21 - fQ11) / (x2 - x1));
    double fR2 = fQ12 + (x - x1) * ((fQ22 - fQ12) / (x2 - x1));
    
    // Interpolate in the y direction
    double fP = fR1 + (y - y1) * ((fR2 - fR1) / (y2 - y1));
    
    return fP;
}

std::vector<tf2::Transform> waypointsRelativeTransformsToGoal(std::vector<geometry_msgs::PoseStamped> poses) 
{
    std::vector<tf2::Transform> relative_transforms;

    geometry_msgs::PoseStamped last_pose = poses.back();    // goal is the last pose 
    
    // Convert the last pose (origin) to a tf2::Transform
    tf2::Transform origin_transform;
    tf2::fromMsg(last_pose.pose, origin_transform);

    for (size_t i = 0; i < poses.size() - 1; ++i) {
        // Convert each pose to a tf2::Transform
        tf2::Transform current_pose_transform;
        tf2::fromMsg(poses[i].pose, current_pose_transform);

        // Calculate the relative transform between the pose and the last pose (origin)
        tf2::Transform relative_transform = origin_transform.inverse() * current_pose_transform;
        relative_transforms.push_back(relative_transform);
    }
    return relative_transforms;
}

geometry_msgs::Pose transformToPose(const tf2::Transform& transform) {
    geometry_msgs::Pose pose;
    
    // Set translation
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();
    
    // Set rotation (as quaternion)
    tf2::Quaternion quat = transform.getRotation();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    
    return pose;
}

std::vector<geometry_msgs::PoseStamped> updateWaypointsWithGoal(std::vector<geometry_msgs::PoseStamped> poses,
                                        const geometry_msgs::PoseStamped& goal,
                                        std::vector<tf2::Transform> relative_transforms) {
    tf2::Transform new_origin_transform;
    tf2::fromMsg(goal.pose, new_origin_transform);

    for (size_t i = 1; i < poses.size() - 1; ++i) {
        // Calculate the new pose by applying the relative transform to the new origin
        tf2::Transform new_pose_transform = new_origin_transform * relative_transforms[i];

        // Convert the tf2::Transform to geometry_msgs::Pose using transformToPose function
        poses[i].pose = transformToPose(new_pose_transform);
    }

    // Update the last pose in the vector with the new last pose
    poses.back() = goal;
    return poses;
}
