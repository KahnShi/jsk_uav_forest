#include <jsk_uav_forest_motion/TriangleGenerator.h>
void triangleGenerator::onInit()
{
  ros::NodeHandle private_nh("~");
  private_nh.param("uav_odom_sub_topic_name", m_uav_odom_sub_topic_name, (std::string)"ground_truth/state");
  private_nh.param("spline_path_pub_topic_name", m_spline_path_pub_topic_name, (std::string)"spline_path");
  private_nh.param("uav_cmd_pub_topic_name", m_uav_cmd_pub_topic_name, (std::string)"cmd_vel");
  private_nh.param("spline_degree", m_spline_degree, 2);
  private_nh.param("spline_segment_time", m_spline_segment_time, 1.0);

  /* bspline */
  m_bspline_generator.onInit(m_spline_degree, true, m_spline_path_pub_topic_name);

  /* Subscriber */
  m_sub_uav_odom = m_nh.subscribe<nav_msgs::Odometry>(m_uav_odom_sub_topic_name, 1, &triangleGenerator::uavOdomCallback, this);
  m_sub_uav_start_flag = m_nh.subscribe<std_msgs::Empty>("/simulator_uav_start_flag", 1, &triangleGenerator::uavStartFlagCallback, this);

  /* Publisher */
  m_pub_uav_cmd  = m_nh.advertise<geometry_msgs::Twist>(m_uav_cmd_pub_topic_name, 1);

  /* uav */
  m_uav.onInit();

  ROS_INFO("triangleGenerator init finished.");
}

void triangleGenerator::splineInputParam()
{
  geometry_msgs::PolygonStamped control_polygon_points;
  for (int i = 0; i < m_control_point_vec.size(); ++i){
    geometry_msgs::Point32 control_point, time_point;
    time_point.x = m_spline_segment_time * i;
    control_polygon_points.polygon.points.push_back(time_point);
    vector3dConvertToPoint32(m_control_point_vec[i], control_point);
    control_polygon_points.polygon.points.push_back(control_point);
  }
  m_bspline_generator.bsplineParamInput(&control_polygon_points);
  m_bspline_generator.getDerive();
}

inline void triangleGenerator::vector3dConvertToPoint32(Vector3d point3, geometry_msgs::Point32& point32)
{
  point32.x = point3.x();
  point32.y = point3.y();
  point32.z = point3.z();
}

void triangleGenerator::uavOdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  m_uav_odom = *msg;
  m_uav.getUavOdom(msg);
  if (m_uav.m_uav_state == 0){
    m_uav.uavMovingToPresetHeight(1.5);
    m_pub_uav_cmd.publish(m_uav.m_uav_cmd);
    return;
  }
}

void triangleGenerator::uavStartFlagCallback(const std_msgs::Empty msg)
{
  m_uav.m_uav_state = 2;
}
