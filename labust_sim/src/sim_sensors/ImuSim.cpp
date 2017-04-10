/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, LABUST, UNIZG-FER
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
 *   * Neither the name of the LABUST nor the names of its
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
 *
 *  Author: Dula Nad
 *  Created: 01.02.2013.
 *********************************************************************/
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <labust/tools/conversions.hpp>

#include <boost/thread/mutex.hpp>

struct ImuSim
{
	ImuSim()
	{
		ros::NodeHandle nh;
		odom = nh.subscribe<nav_msgs::Odometry>("meas_odom",1,&ImuSim::onOdom, this);
		acc = nh.subscribe<geometry_msgs::Vector3>("nuacc_ideal",1,&ImuSim::onAcc, this);

		imu_pub = nh.advertise<sensor_msgs::Imu>("imu",1);
		depth_pub = nh.advertise<std_msgs::Float32>("depth",1);
	}

	void onOdom(const typename nav_msgs::Odometry::ConstPtr& msg)
	{
		sensor_msgs::Imu::Ptr imu(new sensor_msgs::Imu());
		imu->header.stamp = ros::Time::now();
		imu->header.frame_id = "base_link";
		imu->angular_velocity = msg->twist.twist.angular;
		imu->orientation = msg->pose.pose.orientation;

		{
			boost::mutex::scoped_lock l(acc_mux);
			imu->linear_acceleration = nuacc;
		}

		imu_pub.publish(imu);

		std_msgs::Float32::Ptr depth(new std_msgs::Float32());
		depth->data = msg->pose.pose.position.z;
		depth_pub.publish(depth);

		//Publish the imu_frame location
		///\todo Replace this with the available static transform publisher node
		geometry_msgs::TransformStamped transform;
		transform.transform.translation.x = 0;
		transform.transform.translation.y = 0;
		transform.transform.translation.z = 0;
		labust::tools::quaternionFromEulerZYX(0, 0, 0,
				transform.transform.rotation);
		transform.child_frame_id = "imu_frame";
		transform.header.frame_id = "base_link";
		transform.header.stamp = ros::Time::now();
		broadcaster.sendTransform(transform);
	}

	void onAcc(const typename geometry_msgs::Vector3::ConstPtr& msg)
	{
		boost::mutex::scoped_lock l(acc_mux);
		nuacc = *msg;
	}

private:
	ros::Subscriber odom, acc;
	ros::Publisher imu_pub, depth_pub;
	boost::mutex acc_mux;
	geometry_msgs::Vector3 nuacc;
	tf2_ros::TransformBroadcaster broadcaster;
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"imu_sim");
	ros::NodeHandle nh;
	ImuSim imu;
	ros::spin();
	return 0;
}


