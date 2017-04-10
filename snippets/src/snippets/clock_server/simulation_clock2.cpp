/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Heriot-Watt University, UK.
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
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Heriot-Watt University nor the names of
*     its contributors may be used to endorse or promote products
*     derived from this software without specific prior written
*     permission.
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
*
*  Author: Joel Cartwright
*
*********************************************************************/

#include <iostream>
#include <sstream>
using namespace std;

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64.h>

//#include <osl_core/ModuleCore.h>

// ---------------------------------------------------------------------------

double time_scale = 2.0;

void onTimeScale(const typename std_msgs::Float64::ConstPtr& msg)
{
	time_scale = msg->data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "clock_sim");
	ros::NodeHandle node;

	ros::Subscriber positionSub = node.subscribe<std_msgs::Float64>("time_scale", 1, onTimeScale);
	//double time_scale = 2.0;
	double clock_publish_rate = 100.0;

	ROS_INFO_STREAM("Setting ROS param use_time_time to true.");
	ros::param::set("/use_sim_time", true);

	if (!ros::param::getCached("/time_scale", time_scale)) {
		ROS_INFO_STREAM("clock_time_scale not set, using default: " << time_scale);
	}
	else {
		ROS_INFO_STREAM("time_scale: " << time_scale);
	}

	if (!ros::param::getCached("clock_publish_rate", clock_publish_rate)) {
		ROS_INFO_STREAM("clock_publish_rate not set, using default: " << clock_publish_rate);
	}
	else {
		ROS_INFO_STREAM("clock_publish_rate: " << clock_publish_rate);
	}

	ros::Publisher clock_pub = node.advertise<rosgraph_msgs::Clock>("/clock", 1);

	ROS_INFO_STREAM("Sending simulation clock at wall rate " << clock_publish_rate << " Hz.");

	ros::WallTime real_start_time = ros::WallTime::now();
	ros::Time sim_start_time;
	// There is no assignment operator between WallTime and Time, so do it manually.
	sim_start_time.sec = real_start_time.sec;
	sim_start_time.nsec = real_start_time.nsec;

	ros::WallTime real_time;
	ros::WallDuration real_duration;
	ros::Duration sim_duration;

	ros::WallRate loop_rate(clock_publish_rate);

	ros::WallDuration check_scale_period(0.1);
	ros::WallTime check_scale_time = real_start_time + check_scale_period;

	rosgraph_msgs::Clock clock_msg;

	while (ros::ok())
	{
		real_time = ros::WallTime::now();
		real_duration = real_time - real_start_time;
		sim_duration.fromSec(real_duration.toSec() * time_scale);
		clock_msg.clock = sim_start_time + sim_duration;

		if (check_scale_time <= real_time) {
			check_scale_time = real_time + check_scale_period;
			double new_time_scale;
			if (ros::param::getCached("/time_scale", new_time_scale) && new_time_scale != time_scale) {
				ROS_INFO_STREAM("time_scale parameter has changed, applying new value: " << new_time_scale);
				real_start_time += real_duration;
				sim_start_time += sim_duration;
				time_scale = new_time_scale;
			}
		}

		clock_pub.publish(clock_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	ROS_INFO_STREAM("Setting ROS param use_time_time to false.");
	ros::param::set("/use_sim_time", false);

	return 0;
}
