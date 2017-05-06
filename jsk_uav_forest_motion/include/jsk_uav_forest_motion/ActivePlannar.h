#ifndef ACTIVE_PLANNAR_H_
#define ACTIVE_PLANNAR_H_

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <unistd.h>
#include <stdlib.h>
#include <eigen3/Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <math.h>

class ActivePlannar
{
public:
  ros::NodeHandle m_nh;

  bool m_active_plannar_start_flag;
  nav_msgs::Odometry m_uav_odom;
  std::vector<std::vector<geometry_msgs::Point32> > m_triangle_mesh;
  tf::Vector3 m_uav_pos;
  tf::Vector3 m_uav_ang;
  tf::Vector3 m_target_pos;
  tf::Vector3 m_start_pos;
  tf::Vector3 m_start_vel;
  tf::Vector3 m_target_vel;
  tf::Matrix3x3 m_uav_rot_mat;
  double m_acc_ub;
  double m_vel_ub;
  double m_control_period;

  /* Subscriber */
  ros::Subscriber m_sub_target_poses;
  ros::Subscriber m_sub_uav_odom;
  ros::Subscriber m_sub_start_flag;
  ros::Subscriber m_sub_scan_cluster;

  /* Publisher */
  ros::Publisher m_pub_control_points;
  ros::Publisher m_pub_triangle_mesh;

  void onInit();
  void startFlagCallback(const std_msgs::Empty msg);
  void uavOdomCallback(const nav_msgs::OdometryConstPtr& msg);
  void targetPosesCallback(const geometry_msgs::PolygonStampedConstPtr& msg);
  void scanClusterCallback(const sensor_msgs::LaserScanConstPtr& msg);
  void controlCallback(const ros::TimerEvent&);
  void generateTriangleFromPose(const geometry_msgs::PolygonStampedConstPtr& msg, std::vector<std::vector<geometry_msgs::Point32> >& origin_triangle);
  geometry_msgs::Point32 getGlobalPointFromLaser(double ang, double distance);
  geometry_msgs::Point32 getLocalPointFromLaser(double ang, double distance);
  void visualizeTriangleMesh();
  void point32ToPoint(geometry_msgs::Point32 pt32, geometry_msgs::Point& pt);
};

#endif
