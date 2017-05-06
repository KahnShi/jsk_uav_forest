#ifndef TRIANGLE_GENERATOR_H_
#define TRIANGLE_GENERATOR_H_

#include <ros/ros.h>
#include <jsk_uav_forest_motion/QuadrotorCommand.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>


class triangleGenerator
{
 public:
  ros::NodeHandle m_nh;

  /* Subscriber */
  ros::Subscriber m_sub_uav_odom;
  ros::Subscriber m_sub_uav_start_flag;
  ros::Subscriber m_sub_control_points;

  /* Publisher */
  ros::Publisher m_pub_uav_cmd;

  std::string m_uav_odom_sub_topic_name;
  std::string m_control_points_sub_topic_name;
  std::string m_spline_path_pub_topic_name;
  std::string m_uav_cmd_pub_topic_name;

  /* bspline generator */
  bsplineGenerate m_bspline_generator;
  int m_spline_degree;
  double m_spline_segment_time;
  std::vector<geometry_msgs::Point32> m_control_point_vec;

  /* uav */
  nav_msgs::Odometry m_uav_odom;
  QuadrotorCommand m_uav;
  bool m_yaw_mode;

  /* functions */
  void onInit();
  inline void vector3dConvertToPoint32(Vector3d point3, geometry_msgs::Point32& point32);
  inline void point32ConvertToVector3d(geometry_msgs::Point32 point32, Vector3d& point3);
  void splineInputParam();
  void uavOdomCallback(const nav_msgs::OdometryConstPtr& msg);
  void uavStartFlagCallback(const std_msgs::Empty msg);
  void controlPointsCallback(const geometry_msgs::PolygonStampedConstPtr& msg);
};

#endif
