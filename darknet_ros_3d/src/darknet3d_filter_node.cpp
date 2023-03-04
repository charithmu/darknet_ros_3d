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
*    * Neither the name of Intelligent Robotics nor the names of itsdarknet3dfilter
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
/* Author: Fernando González fergonzaramos@yahoo.es  */

#include <ros/ros.h>

#include "darknet_ros_3d/Darknet3DFilter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "darknet_3d_filter");
  darknet_ros_3d::Darknet3DFilter darknet3dfilter;

  darknet_ros_3d::ObjectConfiguration person;
  
  person.class_id = "person";
  person.min_probability = 0.95;
  person.life_time = ros::Duration(30.0); // not used

  person.min_x = 1.0;
  person.max_x = 4.0;
  person.min_y = -2.0;

  person.max_y = 2.0;
  person.min_z = 0.0;
  person.max_z = 2.0;

  person.min_size_x = 0.3;
  person.min_size_y = 0.3;
  person.min_size_z = 0.6;

  person.max_size_x = 1.0;
  person.max_size_y = 1.0;
  person.max_size_z = 2.5;

  darknet3dfilter.add_class("person", person);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
