/*********************************************************************
 * USBLSim.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: Filip Mandic
 *
 ********************************************************************/

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, LABUST, UNIZG-FER
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
 *********************************************************************/
#include <Eigen/Dense>
#include <ros/ros.h>
#include <auv_msgs/NavSts.h>
#include <std_msgs/Float32.h>
#include <underwater_msgs/USBLFix.h>
#include <labust/math/NumberManipulation.hpp>

class USBLSim{

public:

	USBLSim(){

		ros::NodeHandle nh;

		subVehiclePos = nh.subscribe<auv_msgs::NavSts>("pos_first",1,&USBLSim::onVehiclePos,this);
		subTargetPos = nh.subscribe<auv_msgs::NavSts>("pos_second",1,&USBLSim::onTargetPos,this);
		pubUSBLFix = nh.advertise<underwater_msgs::USBLFix>("usbl_fix",1);
		pubRange = nh.advertise<std_msgs::Float32>("range",1);

	}

	~USBLSim(){}

	void start(){

		ros::NodeHandle ph("~");
		double Ts(1.0);
		ph.param("Trange",Ts,Ts);
		ros::Rate rate(1/Ts);

		int i = 0;

		while (ros::ok()){


			if(i++==0) continue;
			range = (vehPos-tarPos).norm();
			bearing = labust::math::wrapRad(std::atan2(double(vehPos(1)-tarPos(1)),double(vehPos(0)-tarPos(0)))-0*vehYaw);
			elevation = std::asin((double(vehPos(2)-tarPos(2)))/range);


			usbl.header.stamp = ros::Time::now();
			usbl.range = range;
			usbl.bearing = bearing;
			usbl.elevation = elevation;

			std_msgs::Float32 range;
			range.data = usbl_past.range;
			pubRange.publish(range);

			pubUSBLFix.publish(usbl_past);

			usbl_past = usbl;


			rate.sleep();
			ros::spinOnce();
		}
	}

	void onVehiclePos(const auv_msgs::NavSts::ConstPtr& data){

		vehPos << data->position.north, data->position.east, data->position.depth;
		vehYaw = data->orientation.yaw;
	}

	void onTargetPos(const auv_msgs::NavSts::ConstPtr& data){

		tarPos << data->position.north, data->position.east, data->position.depth;
	}

	/*
	 * Class variables
	 */

	ros::Subscriber subVehiclePos, subTargetPos;
	ros::Publisher pubUSBLFix, pubRange;

	Eigen::Vector3d vehPos, tarPos;
	double vehYaw;
	//std_msgs::Float32 range, bearing, elevation;
	double range, bearing, elevation;



	underwater_msgs::USBLFix usbl, usbl_past;
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"range_sim");
	USBLSim usblSim;
	usblSim.start();
	return 0;
}












