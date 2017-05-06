#include <jsk_uav_forest_motion/ActivePlannar.h>

void ActivePlannar::onInit(){
  ros::NodeHandle private_nh("~");
  private_nh.param("control_period", m_control_period, 0.9);
  private_nh.param("velocity_upper_bound", m_vel_ub, 7.0);
  private_nh.param("acceleration_upper_bound", m_acc_ub, 2.0);

  m_sub_start_flag = m_nh.subscribe<std_msgs::Empty>("active_plannar_start_flag", 1, &ActivePlannar::startFlagCallback, this);
  m_sub_uav_odom = m_nh.subscribe<nav_msgs::Odometry>("ground_truth/state", 1, &ActivePlannar::uavOdomCallback, this);
  m_sub_target_poses = m_nh.subscribe<geometry_msgs::PolygonStamped>("target_tree_poses", 1, &ActivePlannar::targetPosesCallback, this);
  m_sub_scan_cluster = m_nh.subscribe<sensor_msgs::LaserScan>("scan_clustered", 1, &ActivePlannar::scanClusterCallback, this);
  ros::Timer timer = m_nh.createTimer(ros::Duration(m_control_period), &ActivePlannar::controlCallback, this);

  m_pub_control_points = m_nh.advertise<geometry_msgs::PolygonStamped>(std::string("control_points"), 1);
  m_pub_triangle_mesh = m_nh.advertise<visualization_msgs::Marker>(std::string("triangle_mesh"), 1);

  m_active_plannar_start_flag = false;
}

void ActivePlannar::controlCallback(const ros::TimerEvent&)
{
  
}

void ActivePlannar::startFlagCallback(const std_msgs::Empty msg){
  m_active_plannar_start_flag = true;
}

void ActivePlannar::uavOdomCallback(const nav_msgs::OdometryConstPtr& uav_msg){
  m_uav_odom = *uav_msg;
  tf::Quaternion uav_q(uav_msg->pose.pose.orientation.x,
                       uav_msg->pose.pose.orientation.y,
                       uav_msg->pose.pose.orientation.z,
                       uav_msg->pose.pose.orientation.w);
  m_uav_rot_mat.setRotation(uav_q);
  tfScalar r,p,y;
  m_uav_rot_mat.getRPY(r, p, y);
  m_uav_ang.setValue(r, p, y);
  m_uav_pos.setValue(uav_msg->pose.pose.position.x,
                     uav_msg->pose.pose.position.y,
                     uav_msg->pose.pose.position.z);
}

void ActivePlannar::targetPosesCallback(const geometry_msgs::PolygonStampedConstPtr& msg){
  // get triangle mesh directly from obstacle's ground truth position
  //generateTriangleFromPose(msg, m_triangle_mesh);
  //visualizeTriangleMesh();
}

void ActivePlannar::visualizeTriangleMesh(){
  visualization_msgs::Marker triangle_list_marker;
  triangle_list_marker.header.frame_id = "world";
  triangle_list_marker.header.stamp = ros::Time::now();
  triangle_list_marker.ns = std::string("triangle_mesh");
  triangle_list_marker.action = visualization_msgs::Marker::ADD;
  triangle_list_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

  geometry_msgs::Point pt;
  /* triangle list */
  triangle_list_marker.scale.x = 1.0;
  triangle_list_marker.scale.y = 1.0;
  triangle_list_marker.scale.z = 0.0;
  triangle_list_marker.color.a = 1.0;
  triangle_list_marker.id = 0;
  std_msgs::ColorRGBA color_r;
  color_r.r = 0.0; color_r.g = 0.0; color_r.b = 1.0; color_r.a = 1.0;
  srand (time(NULL));
  for (int i = 0; i < m_triangle_mesh.size(); ++i){
    point32ToPoint(m_triangle_mesh[i][0], pt);
    triangle_list_marker.points.push_back(pt);
    point32ToPoint(m_triangle_mesh[i][1], pt);
    triangle_list_marker.points.push_back(pt);
    point32ToPoint(m_triangle_mesh[i][2], pt);
    triangle_list_marker.points.push_back(pt);
    color_r.b = rand() / (double)RAND_MAX * 1.0;
    triangle_list_marker.colors.push_back(color_r);
  }
  m_pub_triangle_mesh.publish(triangle_list_marker);
}

void ActivePlannar::scanClusterCallback(const sensor_msgs::LaserScanConstPtr& laser_msg){
  m_triangle_mesh.clear();
  std::vector<geometry_msgs::Point32> p;
  int start_id, end_id, mid_id;
  int triangle_num = 0;
  for (size_t i = 0; i < laser_msg->ranges.size(); i++){
    if (!std::isnan(laser_msg->ranges[i])){
      p.push_back(getGlobalPointFromLaser(i*laser_msg->angle_increment+laser_msg->angle_min, fabs(laser_msg->ranges[i])));
      start_id = i;
      ++i;
      while (i < laser_msg->ranges.size()){
        if (std::isnan(laser_msg->ranges[i])){
          p.push_back(getGlobalPointFromLaser((i-1)*laser_msg->angle_increment+laser_msg->angle_min, fabs(laser_msg->ranges[i-1])));
          end_id = i - 1;
          if (start_id + 2 <= end_id)
            mid_id = (start_id + end_id) / 2;
          else
            mid_id = start_id;
          p.push_back(getGlobalPointFromLaser((mid_id-1)*laser_msg->angle_increment+laser_msg->angle_min, fabs(laser_msg->ranges[mid_id])));
          // judge whether cluster is the ground instead of obstacle
          // todo: if laser point's height < 0.2, consider it as the ground
          if (p[2].z > 0.2){
            triangle_num += 1;
            m_triangle_mesh.push_back(p);
          }
          p.clear();
          break;
        }
        ++i;
      }
    }
  }
  // todo: currently from ground truth, there are 3 triangles. If more than 3, means having wrong detection.
  // todo: sometimes one tree is detected into several parts
  // if (triangle_num > 3){
  //   ROS_WARN("Wrong obstacles are detected: %d", triangle_num - 3);
  //   for (int i = 0; i < triangle_num; ++i){
  //     std::cout << m_triangle_mesh[i][2].x << ", " << m_triangle_mesh[i][2].y << "\n";
  //   }
  // }
  visualizeTriangleMesh();
}

geometry_msgs::Point32 ActivePlannar::getGlobalPointFromLaser(double ang, double distance){
  geometry_msgs::Point32 pos_ground_ref = getLocalPointFromLaser(ang, distance);
  geometry_msgs::Point32 pt32;
  pt32.x = pos_ground_ref.x + m_uav_pos.x();
  pt32.y = pos_ground_ref.y + m_uav_pos.y();
  pt32.z = pos_ground_ref.z + m_uav_pos.z();
  return pt32;
}

geometry_msgs::Point32 ActivePlannar::getLocalPointFromLaser(double ang, double distance){
  tf::Vector3 pos_laser_ref(distance * cos(ang), distance * sin(ang), 0.0);
  tf::Vector3 pos_ground_ref = m_uav_rot_mat * pos_laser_ref;
  geometry_msgs::Point32 pt32;
  pt32.x = pos_ground_ref.getX();
  pt32.y = pos_ground_ref.getY();
  pt32.z = pos_ground_ref.getZ();
  return pt32;
}

void ActivePlannar::generateTriangleFromPose(const geometry_msgs::PolygonStampedConstPtr& msg, std::vector<std::vector<geometry_msgs::Point32> >& origin_triangle){
  // generate a 2d triangle based on 3d data, and assume target is a 0.1m circle
  double obstacle_radius = 0.15; // 0.1 * sqrt(2)
  for (int i = 0; i < msg->polygon.points.size(); ++i){
    std::vector<geometry_msgs::Point32> t1, t2;
    geometry_msgs::Point32 pt;
    pt = msg->polygon.points[i]; pt.x += obstacle_radius; t1.push_back(pt);
    pt = msg->polygon.points[i]; pt.x -= obstacle_radius; t1.push_back(pt);
    pt = msg->polygon.points[i]; pt.y += obstacle_radius; t1.push_back(pt);

    pt = msg->polygon.points[i]; pt.x += obstacle_radius; t2.push_back(pt);
    pt = msg->polygon.points[i]; pt.x -= obstacle_radius; t2.push_back(pt);
    pt = msg->polygon.points[i]; pt.y -= obstacle_radius; t2.push_back(pt);

    origin_triangle.push_back(t1);
    origin_triangle.push_back(t2);
  }
}

void ActivePlannar::point32ToPoint(geometry_msgs::Point32 pt32, geometry_msgs::Point& pt){
  pt.x = pt32.x;
  pt.y = pt32.y;
  pt.z = pt32.z;
}
