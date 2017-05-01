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

/* This node is to convert from Mono16 to the recommended format for processing in ROS (float depth image in m) */

/* ros */
#include <ros/ros.h>

/* ros msg */
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

/* cv */
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

/* standard */
#include <iostream>
#include <vector>

class monoConvert
{
public:
  monoConvert();
  ros::NodeHandle m_nh;
  std::string m_mono_topic_name;
  std::string m_cvt_topic_name;
  ros::Subscriber m_sub_mono_image;
  ros::Publisher m_pub_cvt_image;
  void monoCallback(const sensor_msgs::ImageConstPtr& msg);
};

monoConvert::monoConvert(){
  ros::NodeHandle private_nh("~");
  private_nh.param("mono_topic_name", m_mono_topic_name, std::string("/guidance/depth_image"));
  private_nh.param("cvt_topic_name", m_cvt_topic_name, std::string("/guidance/depth/image_raw"));
  m_sub_mono_image = m_nh.subscribe<sensor_msgs::Image>(m_mono_topic_name, 1, &monoConvert::monoCallback, this);
  m_pub_cvt_image  = m_nh.advertise<sensor_msgs::Image>(m_cvt_topic_name, 1);
}

void monoConvert::monoCallback(const sensor_msgs::ImageConstPtr& raw_msg){
  sensor_msgs::ImagePtr depth_msg( new sensor_msgs::Image );
  depth_msg->header   = raw_msg->header;
  depth_msg->height   = raw_msg->height;
  depth_msg->width    = raw_msg->width;
  depth_msg->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  depth_msg->step     = raw_msg->width * (sensor_msgs::image_encodings::bitDepth(depth_msg->encoding) / 8);
  depth_msg->data.resize(depth_msg->height * depth_msg->step);
  // Fill in the depth image data, converting mm to m
  uint16_t bad_point = 0;
  const uint16_t* raw_data = reinterpret_cast<const uint16_t*>(&raw_msg->data[0]);
  uint16_t* depth_data = reinterpret_cast<uint16_t*>(&depth_msg->data[0]);
  for (unsigned index = 0; index < depth_msg->height * depth_msg->width; ++index)
    {
      uint16_t raw = raw_data[index];
      depth_data[index] = (raw == 0) ? bad_point : (uint16_t)(raw / pow(2.0, 7) * 1000);
    }

  m_pub_cvt_image.publish(depth_msg);
}

int main (int argc, char **argv){
  ros::init (argc, argv, "mono_convert");
  monoConvert converter;
  ros::spin ();
}
