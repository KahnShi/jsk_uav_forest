
#include <jsk_uav_forest_motion/QuadrotorCommand.h>

QuadrotorCommand::QuadrotorCommand() {}
void QuadrotorCommand::onInit()
{
  ros::NodeHandle private_nh("~");
  private_nh.param("dji_mode", m_dji_mode, true);
  private_nh.param("global_coordinate_control", m_global_coordinate_control_mode, false);
  private_nh.param("uav_vel_upper_bound", m_uav_vel_ub, 7.0);
  private_nh.param("uav_vel_lower_bound", m_uav_vel_lb, -7.0);
  private_nh.param("uav_acc_upper_bound", m_uav_acc_ub, 2.0);
  private_nh.param("uav_acc_lower_bound", m_uav_acc_lb, -2.0);
  private_nh.param("uav_cmd_traj_track_p_gain", m_traj_track_p_gain, 0.3);
  private_nh.param("uav_cmd_traj_track_i_gain", m_traj_track_i_gain, 0.03);
  private_nh.param("uav_cmd_traj_track_d_gain", m_traj_track_d_gain, 0.0);
  private_nh.param("uav_cmd_traj_yaw_p_gain", m_traj_yaw_p_gain, 1.0);
  private_nh.param("uav_cmd_traj_track_p_term_max", m_traj_track_p_term_max, 6.0);
  private_nh.param("uav_cmd_traj_track_i_term_max", m_traj_track_i_term_max, 4.0);
  private_nh.param("uav_cmd_traj_track_d_term_max", m_traj_track_d_term_max, 0.0);
  private_nh.param("uav_odom_freq", m_uav_odom_freq, 50.0);

  m_traj_track_i_term_accumulation.setValue(0.0, 0.0, 0.0);
  m_traj_updated = false;
  m_traj_first_updated = false;
  m_uav_arrive_gps_point_flag = false;

  m_uav_cmd.linear.x = 0.0; m_uav_cmd.linear.y = 0.0; m_uav_cmd.linear.z = 0.0;
  m_uav_cmd.angular.x = 0.0; m_uav_cmd.angular.y = 0.0; m_uav_cmd.angular.z = 0.0;

  m_uav_state = 0;

  sleep(0.2); //To collect initial values for target and uav odom, which will be used in following functions


}

QuadrotorCommand::~QuadrotorCommand() {}

void QuadrotorCommand::getUavOdom(const nav_msgs::OdometryConstPtr& uav_odom_msg)
{
  m_uav_odom = *uav_odom_msg;
  m_uav_world_pos.setValue(uav_odom_msg->pose.pose.position.x,
                           uav_odom_msg->pose.pose.position.y,
                           uav_odom_msg->pose.pose.position.z);
  m_uav_q = tf::Quaternion(uav_odom_msg->pose.pose.orientation.x,
                           uav_odom_msg->pose.pose.orientation.y,
                           uav_odom_msg->pose.pose.orientation.z,
                           uav_odom_msg->pose.pose.orientation.w);
  m_uav_world_vel.setValue(uav_odom_msg->twist.twist.linear.x,
                           uav_odom_msg->twist.twist.linear.y,
                           uav_odom_msg->twist.twist.linear.z);
}

void QuadrotorCommand::trackGlobalTrajectory()
{
  if (m_traj_updated){
    m_uav_state = 2;
    m_traj_updated = false;
    m_traj_start_time = m_uav_odom.header.stamp.toSec();
  }
  else if (m_uav_state == 1){
    m_uav_cmd.linear.x = 0.0;
    m_uav_cmd.linear.y = 0.0;
    m_uav_cmd.linear.z = 0.0;
    m_uav_cmd.angular.z = 0.0;
    return;
  }

  double uav_current_traj_time = m_uav_odom.header.stamp.toSec() - m_traj_start_time;
  if (uav_current_traj_time >= m_bspline_traj_ptr->m_tn){
    m_uav_state = 1;
    ROS_INFO("\nArrived at last control point. \n");
    return;
  }
  tf::Vector3 uav_des_world_vel = vectorToVector3(m_bspline_traj_ptr->evaluateDerive(uav_current_traj_time));
  tf::Vector3 uav_des_world_pos = vectorToVector3(m_bspline_traj_ptr->evaluate(uav_current_traj_time));
  tf::Vector3 uav_real_world_pos;
  uav_real_world_pos = m_uav_world_pos;
  std::vector<double> uav_des_yaw = m_bspline_traj_ptr->evaluateYaw(uav_current_traj_time);

  tf::Matrix3x3  uav_rot_mat(m_uav_q);
  tfScalar uav_roll, uav_pitch, uav_yaw;
  uav_rot_mat.getRPY(uav_roll, uav_pitch, uav_yaw);
  tf::Matrix3x3 r_z; r_z.setRPY(0, 0, uav_yaw);

  /* pid control in trajectory tracking */
  tf::Vector3 traj_track_p_term =  (uav_des_world_pos - uav_real_world_pos) * m_traj_track_p_gain;
  double p_term_absolute_value = traj_track_p_term.distance(tf::Vector3(0.0, 0.0, 0.0));
  if (p_term_absolute_value > m_traj_track_p_term_max)
    traj_track_p_term = traj_track_p_term * m_traj_track_p_term_max / p_term_absolute_value;
  m_traj_track_i_term_accumulation += (uav_des_world_pos - uav_real_world_pos) / m_uav_odom_freq;
  double i_term_accumulation_absolute_value = m_traj_track_i_term_accumulation.distance(tf::Vector3(0.0, 0.0, 0.0));
  if (i_term_accumulation_absolute_value > m_traj_track_i_term_max)
    m_traj_track_i_term_accumulation = m_traj_track_i_term_accumulation * m_traj_track_i_term_max / i_term_accumulation_absolute_value;
  tf::Vector3 traj_track_i_term = m_traj_track_i_term_accumulation * m_traj_track_i_gain;

  /* feedforward */
  tf::Vector3 uav_vel = uav_des_world_vel + traj_track_p_term + traj_track_i_term;
  double uav_vel_absolute_value = uav_vel.distance(tf::Vector3(0.0, 0.0, 0.0));
  if (uav_vel_absolute_value > m_uav_vel_ub)
    uav_vel = uav_vel * m_uav_vel_ub / uav_vel_absolute_value;

  /* yaw */
  double uav_yaw_p_term = uav_des_yaw[1] - uav_yaw;
  if (uav_yaw_p_term > 1.57)
    uav_yaw_p_term -= 3.14;
  else if (uav_yaw_p_term < -1.57)
    uav_yaw_p_term += 3.14;
  double uav_yaw_vel = uav_des_yaw[0] + uav_yaw_p_term * m_traj_yaw_p_gain;

  /* Judge if the controller is locally based on uav coordinate. */
  if (!m_global_coordinate_control_mode){
    uav_vel = r_z.inverse() * uav_vel;
  }

  m_uav_cmd.linear.x = uav_vel.getX();
  m_uav_cmd.linear.y = uav_vel.getY();
  m_uav_cmd.linear.z = uav_vel.getZ();
  if (m_yaw_mode)
    m_uav_cmd.angular.z = uav_yaw_p_term;
  else
    m_uav_cmd.angular.z = 0.0;
}

bool QuadrotorCommand::uavMovingToPresetHeight(double height)
{
  m_uav_state = 0;
  m_uav_cmd.linear.x = 0.0; m_uav_cmd.linear.y = 0.0; m_uav_cmd.linear.z = 0.0;
  if (m_uav_world_pos.getZ() < height){
    m_uav_cmd.linear.z = 0.5;
  }
  else if (m_uav_world_pos.getZ() > height + 1.0){
    m_uav_cmd.linear.z = -0.5;
  }
  else{
    m_uav_state = 1;
    ROS_INFO("\n\n[Change to state] UAV reached specific height.\n\n");
    return true;
  }
  return false;
}


inline tf::Vector3 QuadrotorCommand::vectorToVector3(std::vector<double> vec)
{
  tf::Vector3 vec3(vec[0], vec[1], vec[2]);
  return vec3;
}

inline tf::Vector3 QuadrotorCommand::vector3dToVector3(Vector3d vec)
{
  tf::Vector3 vec3(vec[0], vec[1], vec[2]);
  return vec3;
}
