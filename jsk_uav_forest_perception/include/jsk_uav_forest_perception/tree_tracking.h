// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef TREE_TRACKING_H_
#define TREE_TRACKING_H_

/* ros */
#include <ros/ros.h>

/* message filter */
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

/* ros msg/srv */
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>

using namespace std;

class TreeTracking
{
public:
  TreeTracking(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~TreeTracking(){}

  typedef message_filters::TimeSynchronizer<sensor_msgs::LaserScan, geometry_msgs::Vector3Stamped> SyncPolicy;
private:
  ros::NodeHandle nh_, nhp_;

  boost::shared_ptr<SyncPolicy> sync_;
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub_sync_vision_detection_;
  message_filters::Subscriber<sensor_msgs::LaserScan> sub_sync_scan_;

  ros::Subscriber sub_uav_odom_;
  ros::Subscriber sub_laser_scan_;

  ros::Publisher pub_tree_location_;
  ros::Publisher pub_tree_global_location_;

  string uav_odom_topic_name_;
  string laser_scan_topic_name_;
  string vision_detection_topic_name_;
  string tree_location_topic_name_;
  string tree_global_location_topic_name_;
  string tree_cluster_topic_name_;

  double target_tree_drift_thre_;
  double uav_tilt_thre_;
  bool verbose_;

  double target_theta_, target_dist_;
  tf::Vector3 uav_odom_;
  float uav_roll_, uav_pitch_, uav_yaw_;
  tf::Vector3 target_tree_global_location_;

  void subscribe();
  void unsubscribe();

  void visionDetectionCallback(const sensor_msgs::LaserScanConstPtr& laser_msg, const geometry_msgs::Vector3StampedConstPtr& vision_detection_msg);
  void laserScanCallback(const sensor_msgs::LaserScanConstPtr& laser_msg);
  void uavOdomCallback(const nav_msgs::OdometryConstPtr& uav_msg);


};

#endif
