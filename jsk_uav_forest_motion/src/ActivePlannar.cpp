#include <jsk_uav_forest_motion/ActivePlannar.h>

ActivePlannar::ActivePlannar(ros::NodeHandle nh, ros::NodeHandle nhp): m_nh(nh), m_nhp(nhp)
{
  m_nhp.param("control_period", m_control_period, 0.95);
  m_nhp.param("plan_period", m_plan_period, 1.0);
  m_nhp.param("velocity_upper_bound", m_vel_ub, 4.0);
  m_nhp.param("acceleration_upper_bound", m_acc_ub, 2.0);
  m_nhp.param("goal_x", m_goal_pos[0], 10.0);
  m_nhp.param("goal_y", m_goal_pos[1], 0.0);
  m_nhp.param("goal_z", m_goal_pos[2], 5.0);
  m_nhp.param("motion_directions", m_n_acc_scope, 3);
  m_nhp.param("motion_directions", m_n_motion_directions, 8);
  m_nhp.param("safety_radius", m_safety_radius, 0.6);
  m_nhp.param("transfer_cost_weight", m_transfer_cost_weight, 0.0);
  m_nhp.param("velocity_differ_cost_weight", m_velocity_differ_cost_weight, 0.3);

  m_sub_start_flag = m_nh.subscribe<std_msgs::Empty>("active_plannar_start_flag", 1, &ActivePlannar::startFlagCallback, this);
  m_sub_uav_odom = m_nh.subscribe<nav_msgs::Odometry>("ground_truth/state", 1, &ActivePlannar::uavOdomCallback, this);
  m_sub_target_poses = m_nh.subscribe<geometry_msgs::PolygonStamped>("target_tree_poses", 1, &ActivePlannar::targetPosesCallback, this);
  m_sub_scan_cluster = m_nh.subscribe<sensor_msgs::LaserScan>("scan_clustered", 1, &ActivePlannar::scanClusterCallback, this);
  m_timer = m_nh.createTimer(ros::Duration(0.1), &ActivePlannar::controlCallback, this);

  m_pub_control_points = m_nh.advertise<geometry_msgs::PolygonStamped>(std::string("control_points"), 1);
  m_pub_triangle_mesh = m_nh.advertise<visualization_msgs::Marker>(std::string("triangle_mesh"), 1);

  m_active_plannar_start_flag = false;

  m_acc_prev.setValue(0.0, 0.0, 0.0);
  m_acc_next.setValue(0.0, 0.0, 0.0);
}

void ActivePlannar::controlCallback(const ros::TimerEvent& e)
{
  if (!m_active_plannar_start_flag)
    return;
  std::vector<geometry_msgs::Point32> control_pts;
  geometry_msgs::Point32 control_pt;
  control_pt.x = m_uav_pos.x();
  control_pt.y = m_uav_pos.y();
  // todo: 2d plan
  control_pt.z = m_uav_pos.z();
  control_pts.push_back(control_pt);
  control_pt.x += m_uav_vel.x() / 2 * m_plan_period;
  control_pt.y += m_uav_vel.y() / 2 * m_plan_period;
  // todo: 2d plan
  control_pt.z += 0.0;
  control_pts.push_back(control_pt);

  geometry_msgs::Point32 new_pt_center;
  new_pt_center.x = control_pts[1].x + 2 * (control_pts[1].x - control_pts[0].x);
  new_pt_center.y = control_pts[1].y + 2 * (control_pts[1].y - control_pts[0].y);
  // todo: 2d plan
  new_pt_center.z = control_pt.z;

  control_pts.push_back(new_pt_center);
  std::vector<double> scores;
  // todo: add when acceleration = 0
  int max_score_id = 0; //-1
  // assign initial value to m_acc_next
  m_acc_next.setValue(0.0, 0.0, 0.0);
  for (int i = 0; i < m_n_acc_scope * m_n_motion_directions; ++i){ // i = -1
    tf::Vector3 acc_next(0.0, 0.0, 0.0);
    if (i == -1){
      acc_next.setValue(0.0, 0.0, 0.0);
    }
    else{
      int acc_id = i % m_n_acc_scope + 1;
      int direction_id = i % m_n_motion_directions;
      double acc = m_acc_ub * acc_id / m_n_acc_scope;
      double ang = m_uav_ang.z() + 2 * 3.14 * direction_id / m_n_motion_directions;
      // todo: currently only 2d planning
      acc_next.setValue(acc * sin(ang), acc * cos(ang), 0.0);
    }
    control_pts[2].x = new_pt_center.x + acc_next.x() * m_plan_period;
    control_pts[2].y = new_pt_center.y + acc_next.y() * m_plan_period;
    control_pts[2].z = new_pt_center.z + acc_next.z() * m_plan_period;
    control_pts[2].x = (control_pts[2].x + control_pts[1].x) / 2;
    control_pts[2].y = (control_pts[2].y + control_pts[1].y) / 2;
    control_pts[2].z = (control_pts[2].z + control_pts[1].z) / 2;
    if (isControlTriangleFeasible(control_pts)){
      double cur_score = getControlTriangleScore(control_pts, acc_next);
      scores.push_back(cur_score);
      if (cur_score >= scores[max_score_id]){
        max_score_id = i;
        m_acc_next.setX(acc_next.getX());
        m_acc_next.setY(acc_next.getY());
        m_acc_next.setZ(acc_next.getZ());
      }
    }
    else
      scores.push_back(-10000.0);
  }
  geometry_msgs::PolygonStamped control_polygon;
  // judge whether all the situations are unavailable
  if (scores[max_score_id] < -10000.0 + 0.1){
    ROS_WARN("No available path.");
    // e-stop with maximum acceleration
    // todo
    m_acc_next.setValue(-m_acc_ub, 0.0, 0.0);
    control_pts[2].x = new_pt_center.x + m_acc_next.x() * m_plan_period;
    control_pts[2].y = new_pt_center.y + m_acc_next.y() * m_plan_period;
    control_pts[2].z = new_pt_center.z + m_acc_next.z() * m_plan_period;
  }
  else{
    control_pts[2].x = new_pt_center.x + m_acc_next.x() * m_plan_period;
    control_pts[2].y = new_pt_center.y + m_acc_next.y() * m_plan_period;
    control_pts[2].z = new_pt_center.z + m_acc_next.z() * m_plan_period;
  }
  control_pts[2].x = (control_pts[2].x + control_pts[1].x) / 2;
  control_pts[2].y = (control_pts[2].y + control_pts[1].y) / 2;
  control_pts[2].z = (control_pts[2].z + control_pts[1].z) / 2;
  for (int i = 0; i < 3; ++i)
    control_polygon.polygon.points.push_back(control_pts[i]);
  m_pub_control_points.publish(control_polygon);
  std::cout << "acc: " << m_acc_next.x() << ", " << m_acc_next.y() << "\n\n";
  m_acc_prev = m_acc_next;
}

bool ActivePlannar::isControlTriangleFeasible(std::vector<geometry_msgs::Point32>& control_pts){
  /* velocity feasible detection */
  // todo: 2d plan
  double vel = sqrt(pow(0.0 - 0.0, 2) + pow(control_pts[2].y - control_pts[1].y, 2) + pow(control_pts[2].x - control_pts[1].x, 2)) / m_plan_period;
  if (vel > m_vel_ub)
    return false;
  /* collision free detection */
  bool is_no_collision = true;
  for (int i = 0; i < m_triangle_mesh_buf.size(); ++i){
    if (doTriangleInteresect2(control_pts, m_triangle_mesh_buf[i])){
      is_no_collision = false;
      break;
    }
  }
  if (is_no_collision)
    return true;
  else
    return false;
}

double ActivePlannar::getControlTriangleScore(std::vector<geometry_msgs::Point32>& control_pts, tf::Vector3 acc_next){
  /* contribution to trip - transfer cost */
  double score, trip_contribution, transfer_cost, velocity_score;
  // todo: 2d plan
  trip_contribution = sqrt(pow(0.0 - 0.0, 2) + pow(control_pts[0].y - m_goal_pos[1], 2) + pow(control_pts[0].x - m_goal_pos[0], 2)) - sqrt(pow(0.0 - 0.0, 2) + pow(control_pts[2].y - m_goal_pos[1], 2) + pow(control_pts[2].x - m_goal_pos[0], 2));

  transfer_cost = acc_next.distance(m_acc_prev);

  double goal_velocity[3] = {0.0, 0.0, 0.0};
  velocity_score = sqrt(pow(0.0, 2) + pow((control_pts[2].y - control_pts[1].y) / m_plan_period - goal_velocity[1], 2) + pow((control_pts[2].x - control_pts[1].x) / m_plan_period - goal_velocity[0], 2));

  score = trip_contribution - m_transfer_cost_weight * transfer_cost - m_velocity_differ_cost_weight * velocity_score;
  return score;
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
  m_uav_vel.setValue(uav_msg->twist.twist.linear.x,
                     uav_msg->twist.twist.linear.y,
                     uav_msg->twist.twist.linear.z);
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
  // for (int i = 0; i < m_triangle_mesh.size(); ++i){
  //   point32ToPoint(m_triangle_mesh[i][0], pt);
  //   triangle_list_marker.points.push_back(pt);
  //   point32ToPoint(m_triangle_mesh[i][1], pt);
  //   triangle_list_marker.points.push_back(pt);
  //   point32ToPoint(m_triangle_mesh[i][2], pt);
  //   triangle_list_marker.points.push_back(pt);
  //   color_r.b = rand() / (double)RAND_MAX * 1.0;
  //   triangle_list_marker.colors.push_back(color_r);
  // }
  for (int i = 0; i < m_triangle_mesh_buf.size(); ++i){
    point32ToPoint(m_triangle_mesh_buf[i][0], pt);
    triangle_list_marker.points.push_back(pt);
    point32ToPoint(m_triangle_mesh_buf[i][1], pt);
    triangle_list_marker.points.push_back(pt);
    point32ToPoint(m_triangle_mesh_buf[i][2], pt);
    triangle_list_marker.points.push_back(pt);
    color_r.b = rand() / (double)RAND_MAX * 1.0;
    triangle_list_marker.colors.push_back(color_r);
  }
  m_pub_triangle_mesh.publish(triangle_list_marker);
}

void ActivePlannar::scanClusterCallback(const sensor_msgs::LaserScanConstPtr& laser_msg){
  m_triangle_mesh.clear();
  m_triangle_mesh_buf.clear();
  std::vector<geometry_msgs::Point32> p;
  int start_id, end_id, mid_id;
  int triangle_num = 0;
  // To generate triangle for representing unknown region
  geometry_msgs::Point32 pt_extend_1, pt_extend_2;
  for (size_t i = 0; i < laser_msg->ranges.size(); i++){
    if (!std::isnan(laser_msg->ranges[i])){
      p.push_back(getGlobalPointFromLaser(i*laser_msg->angle_increment+laser_msg->angle_min, fabs(laser_msg->ranges[i])));
      // triangle unknown region, with an edge with 10.0(as long as possible) length
      pt_extend_1 = getGlobalPointFromLaser(i*laser_msg->angle_increment+laser_msg->angle_min, fabs(laser_msg->ranges[i]) + 10.0);
      start_id = i;
      ++i;
      while (1){
        if (i == laser_msg->ranges.size() || std::isnan(laser_msg->ranges[i])){
          p.push_back(getGlobalPointFromLaser((i-1)*laser_msg->angle_increment+laser_msg->angle_min, fabs(laser_msg->ranges[i-1])));
          pt_extend_2 = getGlobalPointFromLaser((i-1)*laser_msg->angle_increment+laser_msg->angle_min, fabs(laser_msg->ranges[i-1]) + 20.0);
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
            // Add two unknown region whenever an obstalce is found into triangle mesh
            // todo: 2d plan
            for (int j = 0; j < 3; ++j)
              p[j].z = m_goal_pos[2];
            std::vector<geometry_msgs::Point32> p_buf = p;
            m_triangle_mesh.push_back(p);
            extendTriangleMargin(p, p_buf, m_safety_radius);
            m_triangle_mesh_buf.push_back(p_buf);

            p[2] = pt_extend_1;
            p[2].z = m_goal_pos[2];
            m_triangle_mesh.push_back(p);
            extendTriangleMargin(p, p_buf, m_safety_radius);
            m_triangle_mesh_buf.push_back(p_buf);

            p[2] = pt_extend_2;
            p[2].z = m_goal_pos[2];
            m_triangle_mesh.push_back(p);
            extendTriangleMargin(p, p_buf, m_safety_radius);
            m_triangle_mesh_buf.push_back(p_buf);
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
