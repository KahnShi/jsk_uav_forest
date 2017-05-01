#include <jsk_uav_forest_motion/ActivePlannar.h>

void ActivePlannar::onInit(){
  m_sub_start_flag = m_nh.subscribe<std_msgs::Empty>("active_plannar_start_flag", 1, &ActivePlannar::startFlagCallback, this);
  m_sub_uav_odom = m_nh.subscribe<nav_msgs::Odometry>("ground_truth/state", 1, &ActivePlannar::uavOdomCallback, this);
  m_sub_target_poses = m_nh.subscribe<geometry_msgs::PolygonStamped>("target_tree_poses", 1, &ActivePlannar::targetPosesCallback, this);

  m_pub_control_points = m_nh.advertise<geometry_msgs::PolygonStamped>(std::string("control_points"), 1);
  m_pub_triangle_mesh = m_nh.advertise<visualization_msgs::Marker>(std::string("triangle_mesh"), 1);

  m_active_plannar_start_flag = false;
}

void ActivePlannar::startFlagCallback(const std_msgs::Empty msg){
  m_active_plannar_start_flag = true;
}

void ActivePlannar::uavOdomCallback(const nav_msgs::OdometryConstPtr& msg){
  m_uav_odom = *msg;
}

void ActivePlannar::targetPosesCallback(const geometry_msgs::PolygonStampedConstPtr& msg){
  std::vector<std::vector<geometry_msgs::Point32> > origin_triangle;
  generateTriangleFromPose(msg, origin_triangle);

  visualization_msgs::Marker triangle_list_marker;
  triangle_list_marker.header = msg->header;
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
  color_r.r = 1.0; color_r.g = 0.0; color_r.b = 0.0; color_r.a = 1.0;
  srand (time(NULL));
  for (int i = 0; i < origin_triangle.size(); ++i){
    point32ToPoint(origin_triangle[i][0], pt);
    triangle_list_marker.points.push_back(pt);
    point32ToPoint(origin_triangle[i][1], pt);
    triangle_list_marker.points.push_back(pt);
    point32ToPoint(origin_triangle[i][2], pt);
    triangle_list_marker.points.push_back(pt);
    color_r.r = rand() / (double)RAND_MAX * 1.0;
    triangle_list_marker.colors.push_back(color_r);
  }
  m_pub_triangle_mesh.publish(triangle_list_marker);
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
