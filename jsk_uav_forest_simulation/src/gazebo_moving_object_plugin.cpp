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

#include <jsk_uav_forest_simulation/gazebo_moving_object_plugin.h>
#include <string>

namespace gazebo
{

GazeboMovingObject::GazeboMovingObject()
{
}

GazeboMovingObject::~GazeboMovingObject()
{
  event::Events::DisconnectWorldUpdateBegin(update_connection_);

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboMovingObject::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  world_ = _model->GetWorld();
  link_ = _model->GetLink();
  link_name_ = link_->GetName();
  namespace_.clear();

  vel_x = vel_y = vel_yaw = 0;
  static_object_ = false;
  last_time_ = world_->GetSimTime();
  terminated_ = false;
  go_up_flag_ = true;

  // load parameters from sdf
  if (_sdf->HasElement("robotNamespace")) namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();

  if (_sdf->HasElement("bodyName") && _sdf->GetElement("bodyName")->GetValue())
    {
      link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
      link_ = _model->GetLink(link_name_);
    }

  if (_sdf->HasElement("staticObject") && _sdf->GetElement("staticObject")->GetValue())
    {
      static_object_ = _sdf->GetElement("staticObject")->Get<std::string>() == "true"?true:false;
    }

  if (_sdf->HasElement("pose") && _sdf->GetElement("pose")->GetValue())
    {
      pose_str_ = _sdf->GetElement("pose")->Get<std::string>();
      std::istringstream iss(pose_str_);

      pose_ = std::vector<double>{
        std::istream_iterator<double>(iss),
        std::istream_iterator<double>()
      };
      model_->SetLinkWorldPose(math::Pose(pose_[0], pose_[1], pose_[2], pose_[3], pose_[4], pose_[5]), link_);
    }

  if (!link)
  {
    ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  //pub_score_ = node_handle_->advertise<std_msgs::String>("score", 1, true);  // set latch true
  ros::NodeHandle param_handle(*node_handle_, "controller");



  update_connection_ = event::Events::ConnectWorldUpdateBegin(
                                                              boost::bind(&GazeboMovingObject::Update, this));
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboMovingObject::Update()
{
  if ( static_object_ || terminated_ )
    {
      return;
    }
  math::Pose pose = link_->GetWorldPose();
  // Constant speed to go upper boundary and then lower boundary
  double max_y = 3.0;
  if (pose.pos.y > max_y)
    go_up_flag_ = false;
  else if (pose.pos.y < -max_y)
    go_up_flag_ = true;

  if (go_up_flag_)
    vel_y = 1.0;
  else
    vel_y = -1.0;

  vel_x = 0.0;
  vel_yaw = 0.0;

  model_->SetLinearVel(math::Vector3(vel_x, vel_y, 0));
  model_->SetAngularVel(math::Vector3(0, 0, vel_yaw));
  last_time_ = world_->GetSimTime();

  // check score
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboMovingObject::Reset()
{
  state_stamp_ = ros::Time();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboMovingObject)

}  // namespace gazebo
