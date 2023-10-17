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

