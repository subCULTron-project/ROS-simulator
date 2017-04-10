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
 *  Created on: 06.05.2013.
 *  Author: Dula Nad
 *********************************************************************/
#include <labust/control/AsyncMerger.hpp>
#include <std_msgs/Float32.h>

#include <algorithm>

using namespace labust::control;


AsyncMergerBase::AsyncMergerBase():
		Ts(0.3)
{
	this->reset_axes();
	ros::NodeHandle nh,ph("~");
	ph.param("sampling_time",Ts,Ts);
	timer = nh.createTimer(ros::Duration(Ts), &AsyncMergerBase::checkArrived, this);
	loop_time = nh.advertise<std_msgs::Float32>("loop_time",1);
}

void AsyncMergerBase::reset_axes()
{
	for(int i=0; i<6; ++i)
	{
		outaxis[i] = true;
		outval[i] = 0;
	}
}

void AsyncMergerBase::copyToVector(const auv_msgs::BodyForceReq::ConstPtr& topic, bool* axis, double* val)
{
	labust::tools::disableAxisToVector(topic->disable_axis, axis);
	labust::tools::pointToVector(topic->wrench.force,val);
	labust::tools::pointToVector(topic->wrench.torque,val,3);
}

void AsyncMergerBase::copyFromVector(const auv_msgs::BodyForceReq::Ptr& topic, bool* axis, double* val)
{
	labust::tools::vectorToDisableAxis(axis,topic->disable_axis);
	labust::tools::vectorToPoint(val,topic->wrench.force);
	labust::tools::vectorToPoint(val,topic->wrench.torque,3);
}

void AsyncMergerBase::copyToVector(const auv_msgs::BodyVelocityReq::ConstPtr& topic, bool* axis, double* val)
{
	labust::tools::disableAxisToVector(topic->disable_axis, axis);
	labust::tools::pointToVector(topic->twist.linear,val);
	labust::tools::pointToVector(topic->twist.angular,val,3);
}

void AsyncMergerBase::copyFromVector(const auv_msgs::BodyVelocityReq::Ptr& topic, bool* axis, double* val)
{
	labust::tools::vectorToDisableAxis(axis, topic->disable_axis);
	labust::tools::vectorToPoint(val,topic->twist.linear);
	labust::tools::vectorToPoint(val,topic->twist.angular,3);
}

bool AsyncMergerBase::allArrived()
{
	bool all(true);
	for(ControllerMap::const_iterator it=controllers.begin();
			it!=controllers.end(); ++it)
	{
		ROS_INFO("Status: %s : %d",it->first.c_str(), it->second);
		all = all & it->second;
	}
	return all;
}

void AsyncMergerBase::clearAll()
{
	std::vector <std::string> toremove;

	//Reset arrived flag
	for(ControllerMap::iterator it=controllers.begin();
			it!=controllers.end(); ++it)
	{
		if (!it->second) toremove.push_back(it->first);
		it->second = false;
	}
	//Reset axes
	this->reset_axes();

	for (int i=0; i<toremove.size(); ++i)
	{
		ROS_WARN("AsyncMerge: removing unresponsive controller %s", toremove[i].c_str());
		controllers.erase(toremove[i]);
	}
}

/**
 * Method to handle new and old requesters.
 */
void AsyncMergerBase::handleRequester(const std::string& requester)
{
	controllers[requester] = true;
	if (allArrived())
	{
		ROS_INFO("All controller inputs arrived in time.");
		this->publishFinal();
		lastAll = ros::Time::now();
		clearAll();
	}
}

void AsyncMergerBase::checkArrived(const ros::TimerEvent& event)
{
	boost::mutex::scoped_lock l(cnt_mux);
	double dT = (ros::Time::now() - lastAll).toSec();
	if ((dT > Ts) && controllers.size())
	{
		ROS_WARN("Not all controller inputs arrived in time, dT=%f",dT);
		this->publishFinal();
		this->clearAll();
	}
}


int main(int argc, char* argv[])
{
	ros::init(argc,argv,"merger");

	ros::NodeHandle ph("~");
	bool merge_tau(false);
	bool merge_nu(false);
	ph.param("merge_tau", merge_tau, false);
	ph.param("merge_nu", merge_nu, false);

	AsyncMergerBase::Ptr m;
	if (merge_tau)	m.reset(new AsyncMerger<auv_msgs::BodyForceReq>());
	if (merge_nu) m.reset(new AsyncMerger<auv_msgs::BodyVelocityReq>());

	ros::spin();

	return 0;
}
