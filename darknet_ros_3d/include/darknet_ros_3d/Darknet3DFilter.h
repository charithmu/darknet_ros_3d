/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2019, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

#ifndef DARKNET_ROS_3D_DARKNET3DFILTER_H
#define DARKNET_ROS_3D_DARKNET3DFILTER_H

#include <ros/ros.h>

#include <darknet_ros_3d_msgs/BoundingBoxes3d.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <list>
#include <vector>
#include <map>

namespace darknet_ros_3d
{

struct DetectedObject
{
  int obj_id;
  std::string class_id;
  double probability;
  double size_x;
  double size_y;
  double size_z;
  tf2::Vector3 central_point;
  std::list<tf2::Stamped<tf2::Vector3>> history; // future use
};

struct ObjectConfiguration
{
  std::string class_id;
  ros::Duration life_time; // future use
  double min_probability;

  double min_x;
  double min_y;
  double min_z;

  double max_x;
  double max_y;
  double max_z;

  double min_size_x;
  double min_size_z;
  double min_size_y;

  double max_size_x;
  double max_size_y;
  double max_size_z;
};

class Darknet3DFilter
{
public:
  explicit Darknet3DFilter();

  void reset();

  void objectsCallback(const darknet_ros_3d_msgs::BoundingBoxes3d::ConstPtr& msg);
  void timer_callback(const ros::TimerEvent& ev);

  void set_active() {active_ = true;}
  void set_inactive() {active_ = false;}

  void add_class(const std::string& class_id, const ObjectConfiguration& conf);

private:
  bool is_already_detected(const DetectedObject& object);
  bool is_interested_class(const std::string& class_id);
  bool is_valid_object(const DetectedObject& object);

  void add_object(const DetectedObject& object);
  bool same_object(const DetectedObject& obj1, const DetectedObject& obj2);
  void merge_objects(DetectedObject& existing_object, const DetectedObject& new_object);

  void publishMarker(const DetectedObject &object);

  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber object_sub_;
  ros::Publisher marker_pub_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Timer timer_;
  
  // Params
  std::string output_bbx3d_topic_, output_filtered_markers_topic_, map_frame_;

  std::map<std::string, ObjectConfiguration> classes_conf_;
  std::vector<DetectedObject> objects_;
  std::vector<visualization_msgs::Marker> markers_;
  int id_itr_;
  bool active_;

};

};  // namespace darknet_ros_3d

#endif  // DARKNET_ROS_3D_DARKNET3DFILTER_H
