#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>


geometry_msgs::Quaternion rpyToQuaternion(double roll, double pitch, double yaw);
void quaternionToRPY(geometry_msgs::Quaternion quat, double &roll, double &pitch, double &yaw);
double normalize_M_PI(const double &angle);

std::vector<double> linspace(double start, double stop, int num, bool endpoint);
double linearInterpolation(double x0, double y0, double x1, double y1, double x);
double bilinearInterpolation(double x1, double y1, double x2, double y2, 
                             double fQ11, double fQ21, double fQ12, double fQ22, 
                             double x, double y);

std::vector<tf2::Transform> waypointsRelativeTransformsToGoal(std::vector<geometry_msgs::PoseStamped> poses);
geometry_msgs::Pose transformToPose(const tf2::Transform& transform);
std::vector<geometry_msgs::PoseStamped> updateWaypointsWithGoal(
                                        std::vector<geometry_msgs::PoseStamped> poses,
                                        const geometry_msgs::PoseStamped& goal,
                                        std::vector<tf2::Transform> relative_transforms);