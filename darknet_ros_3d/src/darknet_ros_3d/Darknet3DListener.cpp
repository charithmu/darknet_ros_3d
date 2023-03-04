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

#include "darknet_ros_3d/Darknet3DListener.h"

#include <ros/ros.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <list>
#include <algorithm>

namespace darknet_ros_3d
{

  Darknet3DListener::Darknet3DListener() : nh_("~"),
                                           tf_listener_(tfBuffer_),
                                           active_(true)
  {
    output_bbx3d_topic_ = "/darknet_ros_3d/bounding_boxes";
    output_filtered_markers_topic_ = "/darknet_ros_3d/filtered_markers";

    map_frame_ = "summit_xl_map";
    // obj_id = 0;

    nh_.param("output_bbx3d_topic", output_bbx3d_topic_, output_bbx3d_topic_);
    nh_.param("output_filtered_markers_topic", output_filtered_markers_topic_, output_filtered_markers_topic_);
    nh_.param("map_frame", map_frame_, map_frame_);

    object_sub_ = nh_.subscribe(output_bbx3d_topic_, 10, &Darknet3DListener::objectsCallback, this);

    timer_ = nh_.createTimer(ros::Duration(0.5), &Darknet3DListener::timer_callback, this);
    
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(output_filtered_markers_topic_, 10);
  }

  void
  Darknet3DListener::objectsCallback(const darknet_ros_3d_msgs::BoundingBoxes3d::ConstPtr &msg)
  {
    if (!active_)
      return;

    int counter = 0;
    for (const auto &bb : msg->bounding_boxes)
    {
      // ROS_INFO("Object received [%s]", bb.Class.c_str());

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

      // ROS_INFO("-----------> %d prob=%f  [%s] coords=(%lf %lf, %lf) sizes=[%lf, %lf, %lf]", counter++,
      //          object.probability, object.class_id.c_str(),
      //          object.central_point.x(), object.central_point.y(), object.central_point.z(),
      //          object.size_x, object.size_y, object.size_z);

      if (is_valid_object(object))
      {
        // ROS_INFO("Detected object is valid");
        add_object(object);
      }
      else
      {
        // ROS_INFO("Detected object is not valid");
      }
    }

    // check_objects_history();
  }

  /// @brief timer callback to update markers.
  /// @param ev
  void
  Darknet3DListener::timer_callback(const ros::TimerEvent &ev)
  {
    publishMarkers();
  }

  /// @brief timer callback to check object history, update speeds, and remove object if history is empty.
  /// @param ev
  // void
  // Darknet3DListener::timer_callback(const ros::TimerEvent &ev)
  // {
  //   publishMarker(object);

  //   check_objects_history();

  //   for (auto &object : objects_)
  //   {
  //     update_speed(object);
  //   }

  //   auto it = objects_.begin();
  //   while (it != objects_.end())
  //   {
  //     const ObjectConfiguration &conf = classes_conf_[it->class_id];
  //     if (conf.dynamic && it->history.empty())
  //       it = objects_.erase(it);
  //     else
  //       ++it;
  //   }
  // }

  /// @brief Check and remove if history point of a object is older than max life time.
  // void
  // Darknet3DListener::check_objects_history()
  // {
  //   for (auto &object : objects_)
  //   {
  //     const ObjectConfiguration &conf = classes_conf_[object.class_id];
  //     if (conf.dynamic)
  //     {
  //       while (!object.history.empty() && (ros::Time::now() - object.history.front().stamp_) > conf.max_seconds)
  //       {
  //         object.history.pop_front();
  //       }
  //     }
  //   }
  // }

  /// @brief Check for duplicate object detection and add if new.
  /// @param object
  void
  Darknet3DListener::add_object(const DetectedObject &object)
  {
    bool new_object = true;
    for (auto &existing_object : objects_)
    {
      if (same_object(existing_object, object))
      {
        // ROS_INFO("Merging with one existing");
        merge_objects(existing_object, object);
        new_object = false;
      }
      else if (!other_object(existing_object, object))
      {
        // ROS_INFO("Not merging, but it is not other object");
        new_object = false;
      }
    }

    if (new_object)
    {
      ROS_INFO("Adding a new object");
      objects_.push_back(object);
    }
  }

  bool
  Darknet3DListener::other_object(const DetectedObject &obj1, const DetectedObject &obj2)
  {
    return obj1.class_id != obj2.class_id ||
           (fabs(obj1.central_point.x() - obj2.central_point.x()) > (obj1.size_x / 1.9 + obj2.size_x / 1.9)) ||
           (fabs(obj1.central_point.y() - obj2.central_point.y()) > (obj1.size_y / 1.9 + obj2.size_y / 1.9)) ||
           (fabs(obj1.central_point.z() - obj2.central_point.z()) > (obj1.size_z / 1.9 + obj2.size_z / 1.9));
  }

  bool
  Darknet3DListener::same_object(const DetectedObject &obj1, const DetectedObject &obj2)
  {
    return obj1.class_id == obj2.class_id &&
           (fabs(obj1.central_point.x() - obj2.central_point.x()) < (obj1.size_x / 2.0 + obj2.size_x / 2.0)) &&
           (fabs(obj1.central_point.y() - obj2.central_point.y()) < (obj1.size_y / 2.0 + obj2.size_y / 2.0)) &&
           (fabs(obj1.central_point.z() - obj2.central_point.z()) < (obj1.size_z / 2.0 + obj2.size_z / 2.0));
  }

  void
  Darknet3DListener::merge_objects(DetectedObject &existing_object, const DetectedObject &new_object)
  {
    double e_min_x = existing_object.central_point.x() - existing_object.size_x / 2.0;
    double e_min_y = existing_object.central_point.y() - existing_object.size_y / 2.0;
    double e_min_z = existing_object.central_point.z() - existing_object.size_z / 2.0;
    double e_max_x = existing_object.central_point.x() + existing_object.size_x / 2.0;
    double e_max_y = existing_object.central_point.y() + existing_object.size_y / 2.0;
    double e_max_z = existing_object.central_point.z() + existing_object.size_z / 2.0;

    double n_min_x = new_object.central_point.x() - new_object.size_x / 2.0;
    double n_min_y = new_object.central_point.y() - new_object.size_y / 2.0;
    double n_min_z = new_object.central_point.z() - new_object.size_z / 2.0;
    double n_max_x = new_object.central_point.x() + new_object.size_x / 2.0;
    double n_max_y = new_object.central_point.y() + new_object.size_y / 2.0;
    double n_max_z = new_object.central_point.z() + new_object.size_z / 2.0;

    double min_x = std::min(e_min_x, n_min_x);
    double min_y = std::min(e_min_y, n_min_y);
    double min_z = std::min(e_min_z, n_min_z);
    double max_x = std::max(e_max_x, n_max_x);
    double max_y = std::max(e_max_y, n_max_y);
    double max_z = std::max(e_max_z, n_max_z);

    const ObjectConfiguration &conf = classes_conf_[existing_object.class_id];

    double x = (max_x + min_x) / 2.0;
    double y = (max_y + min_y) / 2.0;
    double z = (max_z + min_z) / 2.0;
    x = std::max(std::min(x, conf.max_x), conf.min_x);
    y = std::max(std::min(y, conf.max_y), conf.min_y);
    z = std::max(std::min(z, conf.max_z), conf.min_z);

    // if (conf.dynamic)
    // {
    //   tf2::Stamped<tf2::Vector3> point;
    //   point.stamp_ = ros::Time::now();
    //   point.frame_id_ = working_frame_;
    //   point.setX(x);
    //   point.setY(y);
    //   point.setZ(z);
    //   existing_object.history.push_back(point);

    //   update_speed(existing_object);
    // }

    existing_object.central_point.setX(x);
    existing_object.central_point.setY(y);
    existing_object.central_point.setZ(z);

    existing_object.size_x = std::max(std::min(max_x - min_x, conf.max_size_x), conf.min_size_x);
    existing_object.size_y = std::max(std::min(max_y - min_y, conf.max_size_y), conf.min_size_y);
    existing_object.size_z = std::max(std::min(max_z - min_z, conf.max_size_z), conf.min_size_z);

    existing_object.probability = (existing_object.probability + new_object.probability) / 2.0;
  }

  void
  Darknet3DListener::reset()
  {
    objects_.clear();
  }

  void
  Darknet3DListener::add_class(const std::string &class_id, const ObjectConfiguration &conf)
  {
    classes_conf_[class_id] = conf;
  }

  bool
  Darknet3DListener::is_interested_class(const std::string &class_id)
  {
    return classes_conf_.find(class_id) != classes_conf_.end();
  }

  bool
  Darknet3DListener::is_already_detected(const DetectedObject &object)
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
  Darknet3DListener::is_valid_object(const DetectedObject &object)
  {
    const ObjectConfiguration &conf = classes_conf_[object.class_id];

    return object.probability > conf.min_probability &&
           object.size_x > conf.min_size_x && object.size_x < conf.max_size_x &&
           object.size_y > conf.min_size_y && object.size_y < conf.max_size_y &&
           object.size_z > conf.min_size_z && object.size_z < conf.max_size_z;
  }

  /// @brief Update speed using history
  /// @param object
  // void
  // Darknet3DListener::update_speed(DetectedObject &object)
  // {
  //   if (object.history.size() > 2)
  //   {
  //     // calculate speed from object history
  //     double diff_time = (object.history.back().stamp_ - object.history.front().stamp_).toSec();
  //     double vx = (object.history.back().x() - object.history.front().x()) / diff_time;
  //     double vy = (object.history.back().y() - object.history.front().y()) / diff_time;
  //     double vz = (object.history.back().z() - object.history.front().z()) / diff_time;

  //     if (fabs(vx) < 2.0 && fabs(vy) < 2.0 && fabs(vz) < 2.0)
  //     {
  //       object.speed.setX(vx);
  //       object.speed.setY(vy);
  //       object.speed.setZ(vz);
  //     }
  //   }
  //   else
  //   {
  //     object.speed.setX(0.0);
  //     object.speed.setY(0.0);
  //     object.speed.setZ(0.0);
  //   }
  // }

  /// @brief Print details on current object list
  // void
  // Darknet3DListener::print()
  // {
  //   int counter = 0;
  //   ROS_INFO("============> Number of ojects %zu", objects_.size());

  //   for (const auto &obj : objects_)
  //   {
  //     ROS_INFO("============> %d prob=%f  [%s] coords=(%lf %lf, %lf) sizes=[%lf, %lf, %lf] speed={%lf, %lf, %lf}",
  //              counter++,
  //              obj.probability,
  //              obj.class_id.c_str(),
  //              obj.central_point.x(), obj.central_point.y(), obj.central_point.z(),
  //              obj.size_x, obj.size_y, obj.size_z,
  //              obj.speed.x(), obj.speed.y(), obj.speed.z());

  //     const ObjectConfiguration &conf = classes_conf_[obj.class_id];
  //     if (conf.dynamic)
  //     {
  //       for (const auto &point : obj.history)
  //       {
  //         ROS_INFO("\t[%f] (%lf, %lf, %lf)", point.stamp_.toSec(), point.x(), point.y(), point.z());
  //       }
  //     }
  //   }
  // }

  void
  Darknet3DListener::publishMarker()
  {
      visualization_msgs::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = ros::Time();
      m.ns = "darknet_ros_3d";
      // m.id = object.obj_id;
      m.type = visualization_msgs::Marker::MESH_RESOURCE;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.position.x = 0; // object.central_point.x();
      m.pose.position.y = 0; // object.central_point.y();
      m.pose.position.z = 0; // object.central_point.z();
      m.pose.orientation.x = 0.0;
      m.pose.orientation.y = 0.0;
      m.pose.orientation.z = 0.0;
      m.pose.orientation.w = 1.0;
      m.scale.x = 1.0;
      m.scale.y = 1.0;
      m.scale.z = 1.0;
      m.color.a = 1.0; // Don't forget to set the alpha!
      // m.color.r = 0.0;
      // m.color.g = 1.0;
      // m.color.b = 0.0;
      m.lifetime = ros::Duration(0);
      // m.frame_locked = true;
      m.mesh_resource = "file:///home/charith/dev/rosws/detection/src/darknet_ros_3d/darknet_ros_3d/models/person_standing/meshes/standing.dae"; // only if using a MESH_RESOURCE marker type.
      markers_.push_back(m);

      visualization_msgs::MarkerArray markers_msg;
      markers_msg.markers = &markers_;

      marker_pub_.publish(markers_msg);

  }

}; // namespace darknet_ros_3d
