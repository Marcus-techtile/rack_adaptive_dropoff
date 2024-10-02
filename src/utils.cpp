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
