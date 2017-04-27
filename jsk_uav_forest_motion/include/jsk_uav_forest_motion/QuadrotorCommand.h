
#ifndef QUADROTOR_COMMAND_H_
#define QUADROTOR_COMMAND_H_

/* ros */
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

/* linear algebra */
#include <math.h>
#include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/Dense>
// #include <eigen3/Eigen/LU>
// #include <eigen3/Eigen/Geometry>
// #include <eigen3/Eigen/Eigenvalues>
#include <tf/transform_broadcaster.h>

/* local class */
#include <bspline_ros/bsplineGenerate.h>

/* general header file */
#include <iostream>

using namespace Eigen;

class QuadrotorCommand {
public:
  QuadrotorCommand();
  virtual ~QuadrotorCommand();

  ros::NodeHandle m_nh;
  bool m_dji_mode;
  bool m_global_coordinate_control_mode;
  double m_uav_vel_ub;
  double m_uav_vel_lb;
  double m_uav_acc_ub;
  double m_uav_acc_lb;
  tf::Vector3 m_uav_world_pos;
  tf::Vector3 m_uav_world_vel;
  tf::Vector3 m_uav_world_acc;
  tf::Quaternion m_uav_q;
  /* state: 0, still; 1, take off; 2, start tracking; 3, finished */
  int m_uav_state;
  nav_msgs::Odometry m_uav_odom;
  geometry_msgs::Twist m_uav_cmd;
  bool m_uav_arrive_gps_point_flag;

  // pid
  double m_traj_track_p_gain;
  double m_traj_track_i_gain;
  double m_traj_track_d_gain;
  tf::Vector3 m_traj_track_i_term_accumulation;
  double m_traj_track_p_term_max;
  double m_traj_track_i_term_max;
  double m_traj_track_d_term_max;
  double m_uav_yaw_i_term_accumulation;
  double m_uav_odom_freq;

  /* bspline generator */
  bsplineGenerate *m_bspline_traj_ptr;
  bool m_traj_updated;
  bool m_traj_first_updated;
  double m_traj_start_time;

  /* Publisher */
  std::string m_uav_cmd_pub_topic_name;

  void onInit();
  void getUavOdom(const nav_msgs::OdometryConstPtr& uav_odom_msg);
  void trackGlobalTrajectory();
  bool uavMovingToPresetHeight(double height);
  inline tf::Vector3 vectorToVector3(std::vector<double> vec);
  inline tf::Vector3 vector3dToVector3(Vector3d vec);
};

#endif
