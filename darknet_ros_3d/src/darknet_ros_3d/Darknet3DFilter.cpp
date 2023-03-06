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

#include "darknet_ros_3d/Darknet3DFilter.h"

#include <ros/ros.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <list>
#include <algorithm>

namespace darknet_ros_3d
{

  Darknet3DFilter::Darknet3DFilter() : nh_("~"),
                                       tf_listener_(tfBuffer_),
                                       active_(true)
  {
    output_bbx3d_topic_ = "/darknet_ros_3d/bounding_boxes";
    output_filtered_markers_topic_ = "/darknet_ros_3d/filtered_markers";

    map_frame_ = "summit_xl_map";
    id_itr_ = 0;

    nh_.param("output_bbx3d_topic", output_bbx3d_topic_, output_bbx3d_topic_);
    nh_.param("output_filtered_markers_topic", output_filtered_markers_topic_, output_filtered_markers_topic_);
    nh_.param("map_frame", map_frame_, map_frame_);

    object_sub_ = nh_.subscribe(output_bbx3d_topic_, 10, &Darknet3DFilter::objectsCallback, this);

    // timer_ = nh_.createTimer(ros::Duration(0.5), &Darknet3DFilter::timer_callback, this);

    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(output_filtered_markers_topic_, 10);
  }

  void
  Darknet3DFilter::objectsCallback(const darknet_ros_3d_msgs::BoundingBoxes3d::ConstPtr &msg)
  {
    if (!active_)
      return;

    for (const auto &bb : msg->bounding_boxes)
    {
      if (!is_interested_class(bb.Class))
        continue;

      geometry_msgs::TransformStamped any2wf_msg;
      tf2::Transform any2wf;

      std::string error;
      if (tfBuffer_.canTransform(msg->header.frame_id, map_frame_, msg->header.stamp, ros::Duration(1), &error))
        any2wf_msg = tfBuffer_.lookupTransform(msg->header.frame_id, map_frame_, msg->header.stamp);
      else
      {
        ROS_ERROR("Can't transform %s", error.c_str());
        return;
      }

      tf2::Stamped<tf2::Transform> aux;
      tf2::convert(any2wf_msg, aux);

      any2wf = aux;

      DetectedObject object;

      object.central_point.setX((bb.xmax + bb.xmin) / 2.0);
      object.central_point.setY((bb.ymax + bb.ymin) / 2.0);
      object.central_point.setZ((bb.zmax + bb.zmin) / 2.0);

      object.central_point = any2wf.inverse() * object.central_point;

      object.size_x = bb.xmax - bb.xmin;
      object.size_y = bb.ymax - bb.ymin;
      object.size_z = bb.zmax - bb.zmin;
      object.class_id = bb.Class;
      object.probability = bb.probability;

      if (is_valid_object(object) && !is_already_detected(object))
      {
        ROS_INFO("Adding a new object");
        object.obj_id = ++id_itr_;
        objects_.push_back(object);
        publishMarker(object);
      }
    }
  }

  void
  Darknet3DFilter::timer_callback(const ros::TimerEvent &ev)
  {
    return;
  }

  bool
  Darknet3DFilter::same_object(const DetectedObject &obj1, const DetectedObject &obj2)
  {
    return obj1.class_id == obj2.class_id &&
           (fabs(obj1.central_point.x() - obj2.central_point.x()) < (obj1.size_x / 2.0 + obj2.size_x / 2.0)) &&
           (fabs(obj1.central_point.y() - obj2.central_point.y()) < (obj1.size_y / 2.0 + obj2.size_y / 2.0)) &&
           (fabs(obj1.central_point.z() - obj2.central_point.z()) < (obj1.size_z / 2.0 + obj2.size_z / 2.0));
  }

  void
  Darknet3DFilter::reset()
  {
    objects_.clear();
  }

  void
  Darknet3DFilter::add_class(const std::string &class_id, const ObjectConfiguration &conf)
  {
    classes_conf_[class_id] = conf;
  }

  bool
  Darknet3DFilter::is_interested_class(const std::string &class_id)
  {
    return classes_conf_.find(class_id) != classes_conf_.end();
  }

  bool
  Darknet3DFilter::is_already_detected(const DetectedObject &object)
  {
    for (const auto &test_obj : objects_)
    {
      if (same_object(object, test_obj))
      {
        return true;
      }
    }
    return false;
  }

  bool
  Darknet3DFilter::is_valid_object(const DetectedObject &object)
  {
    const ObjectConfiguration &conf = classes_conf_[object.class_id];

    return object.probability > conf.min_probability &&
           object.size_x > conf.min_size_x && object.size_x < conf.max_size_x &&
           object.size_y > conf.min_size_y && object.size_y < conf.max_size_y &&
           object.size_z > conf.min_size_z && object.size_z < conf.max_size_z;
  }

  void
  Darknet3DFilter::publishMarker(const DetectedObject &object)
  {
    visualization_msgs::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = ros::Time();
    m.ns = "darknet_ros_3d";
    m.id = object.obj_id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = object.central_point.x();
    m.pose.position.y = object.central_point.y();
    m.pose.position.z = object.central_point.z();
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.scale.x = 0.5;
    m.scale.y = 0.5;
    m.scale.z = 0.5;

    // ObjectConfiguration oc = classes_conf_.find(object.class_id)->second;
    // double min_p = oc.min_probability;
    // double p = object.probability;
    // m.color.a = (p - min_p) / (100 - min_p) * 0.9 + 0.1; // Normalize the p value to 0.1(when min_p) to 1.0(when p=1.0)
    m.color.a = 1.0;

    m.lifetime = ros::Duration(0);
    // m.frame_locked = true;
    // m.mesh_resource = "file:///home/charith/dev/rosws/detection/src/darknet_ros_3d/darknet_ros_3d/models/person_standing/meshes/standing.dae"; // only if using a MESH_RESOURCE marker type.
    markers_.push_back(m);

    visualization_msgs::MarkerArray markers_msg;
    markers_msg.markers = markers_;

    marker_pub_.publish(markers_msg);
  }

}; // namespace darknet_ros_3d
