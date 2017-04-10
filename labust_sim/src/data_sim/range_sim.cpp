/*********************************************************************
 * range_sim.cpp
 *
 *  Created on: Aug 18, 2014
 *      Author: Filip Mandic
 *
 ********************************************************************/

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
 *********************************************************************/
#include <Eigen/Dense>
#include <ros/ros.h>
#include <auv_msgs/NavSts.h>
#include <std_msgs/Float32.h>

class RangeSim{

public:

	RangeSim(){

		ros::NodeHandle nh;

		vehiclePosSub = nh.subscribe<auv_msgs::NavSts>("meas_ideal",1,&RangeSim::onVehiclePos,this);
		targetPosSub = nh.subscribe<auv_msgs::NavSts>("target_pos",1,&RangeSim::onTargetPos,this);
		rangeMeas = nh.advertise<std_msgs::Float32>("rangeMeas",1);
	}

	~RangeSim(){}

	void start(){

		ros::NodeHandle ph("~");
		double Ts(1.0);
		ph.param("Trange",Ts,Ts);
		ros::Rate rate(1/Ts);

		while (ros::ok())
		{

			range.data = (vehPos-tarPos).norm();
			rangeMeas.publish(range);

			rate.sleep();
			ros::spinOnce();

		}
	}

	void onVehiclePos(const auv_msgs::NavSts::ConstPtr& data){

		vehPos << data->position.north, data->position.east, data->position.depth;
	}

	void onTargetPos(const auv_msgs::NavSts::ConstPtr& data){

		tarPos << data->position.north, data->position.east, data->position.depth;
	}

	/*
	 * Class variables
	 */

	ros::Subscriber vehiclePosSub, targetPosSub;
	ros::Publisher rangeMeas;

	Eigen::Vector3d vehPos, tarPos;
	std_msgs::Float32 range;
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"range_sim");
	RangeSim rs;
	rs.start();
	return 0;
}







