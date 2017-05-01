
/*
 * Copyright (c) 2016, JSK Robotics Laboratory, The University of Tokyo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef JSK_GAZEBO_MOVING_OBJECT_PLUGIN_H
#define JSK_GAZEBO_MOVING_OBJECT_PLUGIN_H

#include <boost/bind.hpp>
#include <boost/random.hpp>
#include <boost/random/random_device.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/RayShape.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/common.hh>

#include <stdio.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>

namespace gazebo
{

class GazeboMovingObject : public ModelPlugin
{
public:
  GazeboMovingObject();
  virtual ~GazeboMovingObject();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();
  virtual void Reset();

private:
  physics::WorldPtr world_;
  physics::LinkPtr link_;
  physics::ModelPtr model_;

  std::string link_name_;
  std::string namespace_;

  ros::NodeHandle* node_handle_;
  ros::Publisher pub_tree_pose_;
  ros::Time state_stamp_;

  event::ConnectionPtr update_connection_;

  double vel_x, vel_y, vel_yaw;
  common::Time last_time_;
  bool terminated_;
  bool static_object_;
  std::string pose_str_;
  std::vector<double> pose_;
  bool go_up_flag_;
};

}  // namespace gazebo

#endif  // JSK_MBZIRC_COMMON_MBZIRC_GAZEBO_TREASURE_PLUGIN_H