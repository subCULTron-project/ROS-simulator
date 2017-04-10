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
#include <labust/control/manual_control.h>
#include <labust/tools/conversions.hpp>
#include <labust/math/NumberManipulation.hpp>

#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/NavSts.h>

#include <boost/bind.hpp>

using namespace labust::control;

ManualControl::ManualControl():
		nav_valid(false),
		navstate(DOF+1,0),
		mapped(DOF,0)
{
	generators.assign(SRequest::DISABLED);
	this->onInit();
}

void ManualControl::onInit()
{
	ros::NodeHandle nh, ph("~");

	this->setDefaultConfig();
	if (!this->setupConfig())
	{
		ROS_ERROR("ManualControl: Wrong configuration. Check all "
				"vector sizes are %d and sampling size is >0.", DOF);
		this->setDefaultConfig();
	}

	//Setup publishers
	tauref = nh.advertise<auv_msgs::BodyForceReq>("manual_tau",1);
	nuref = nh.advertise<auv_msgs::BodyVelocityReq>("manual_nu",1);
	etaref = nh.advertise<auv_msgs::NavSts>("manual_eta",1);
	//Setup subscribers
	joyin = nh.subscribe(config.topic, 1, &ManualControl::onJoystick, this);
	navsts = nh.subscribe("navsts", 1, &ManualControl::onNavSts, this);
	//Setup service
	config_server = nh.advertiseService("manual_configure", &ManualControl::onConfiguration, this);
	select_server = nh.advertiseService("manual_select", &ManualControl::onSelect, this);
}

bool ManualControl::setupConfig()
{
	ros::NodeHandle nh, ph("~");
	//Configure remapping
	ph.param("axes_map", config.axes_map, config.axes_map);
	ph.param("scale_map", config.scale_map, config.scale_map);
	ph.param("integrated", config.integrated, config.integrated);
	//Configure
	ph.param("maximum_effort", config.maximum_effort, config.maximum_effort);
	ph.param("maximum_nu", config.maximum_nu, config.maximum_nu);
	ph.param("maximum_speed", config.maximum_speed, config.maximum_speed);
	std::vector<int> ug(SRequest::DISABLED,0);
	ph.param("use_generators", ug, ug);
	for (int i=0; i<ug.size(); ++i)
	{
		if (i >= generators.size()) break;
		generators[i] = ug[i];
	}
	//Test first for simulation time, runtime and then the private override
	nh.param("simulation/sampling_time", config.sampling_time, config.sampling_time);
	nh.param("runtime/sampling_time", config.sampling_time, config.sampling_time);
	ph.param("sampling_time", config.sampling_time, config.sampling_time);
	bool use_ff(false);
	ph.param("use_feedforward", use_ff, use_ff);
	config.use_ff = use_ff;

	//Perform some sanity check on the configurations
	bool flag(config.sampling_time > 0);
	flag = flag && (config.axes_map.size() == DOF);
	flag = flag && (config.scale_map.size() == DOF);
	flag = flag && (config.maximum_effort.size() == DOF);
	flag = flag && (config.maximum_nu.size() == DOF);
	flag = flag && (config.maximum_speed.size() == DOF);

	return flag;
}

void ManualControl::setDefaultConfig()
{
	//Set default 1on1 mapping, no scaling
	for(int i=0; i<DOF; ++i) config.axes_map.push_back(i);
	config.scale_map.resize(6,1);
	//By default no axes are integrated.
	config.integrated.resize(6,0);

	config.maximum_effort.resize(6,1);

	config.maximum_nu.resize(6,1);

	config.maximum_speed.resize(6,1);
	config.sampling_time = 0.1;
	config.use_ff = false;

	config.topic = "joy";
}

bool ManualControl::onConfiguration(CRequest& req, CResponse& resp)
{
	boost::mutex::scoped_lock l(cfg_mutex);
	navcon_msgs::ManualConfiguration& cfg(req.configuration);
	if (cfg.axes_map.size() == DOF)
	{
		this->config.axes_map = cfg.axes_map;
	}
	if (cfg.scale_map.size() == DOF) this->config.scale_map = cfg.scale_map;
	if (cfg.integrated.size() == DOF) this->config.integrated = cfg.integrated;
	if (cfg.maximum_effort.size() == DOF) this->config.maximum_effort = cfg.maximum_effort;
	if (cfg.maximum_nu.size() == DOF) this->config.maximum_nu = cfg.maximum_nu;
	if (cfg.maximum_speed.size() == DOF)
	{
		this->config.maximum_speed = cfg.maximum_speed;
		this->config.use_ff = cfg.use_ff;
		this->config.sampling_time = cfg.sampling_time;
	}

	if (!req.configuration.topic.empty())
	{
		this->config.topic = req.configuration.topic;
		ros::NodeHandle nh;
		joyin = nh.subscribe(config.topic, 1, &ManualControl::onJoystick, this);
	}

	resp.current_configuration = this->config;
	return true;
}

bool ManualControl::onSelect(SRequest& req, SResponse& resp)
{
	boost::mutex::scoped_lock l(cfg_mutex);
	for (int i=0; i<req.use_generator.size(); ++i)
	{
		if ((req.use_generator[i]>=SRequest::DISABLED) &&
				(req.use_generator[i]<=SRequest::POSITION))
		{
			this->generators[i] = req.use_generator[i];
		}
	}

	return true;
}

void ManualControl::onJoystick(const sensor_msgs::Joy::ConstPtr& joy)
{
	this->remap(joy);

	bool pubEffort(false), pubSpeed(false), pubPos(false);

	std::vector<double> tauv(DOF,0),nuv(DOF,0),etaff(DOF,0);

	boost::mutex::scoped_lock l(cfg_mutex);
	for(int i=0; i<DOF; ++i)
	{
		if (config.axes_map[i] == CRequest::DISABLED) continue;

		switch (generators[i])
		{
		case SRequest::EFFORT:
			pubEffort = true;
			tauv[i] = mapped[i] * config.maximum_effort[i];
			break;
		case SRequest::SPEED:
			pubSpeed = true;
			nuv[i] = mapped[i] * config.maximum_speed[i];
			break;
		case SRequest::POSITION:
			if (!nav_valid) continue;
			pubPos = true;
			etaff[i] = mapped[i] * config.maximum_speed[i];
			break;
		}
	}
	l.unlock();

	if (pubEffort) pubTau(tauv);
	if (pubSpeed) pubNu(nuv);
	if (pubPos) pubEta(etaff);
};

void ManualControl::onNavSts(const auv_msgs::NavSts::ConstPtr& state)
{
	boost::mutex::scoped_lock l(nav_mutex);
	//Copy the data
	DataType pos(DOF+1,0);
	labust::tools::nedToVector(state->position,pos);
	labust::tools::rpyToVector(state->orientation,pos,3);
	pos[A] = state->altitude;

	if (nav_valid)
	{
		//Update only the data that is not used
		for(int i=0; i<DOF; ++i)
		{
			if (generators[i] != SRequest::POSITION) navstate[i] = pos[i];
		}
	}
	else
	{
		//Copy all for the first time
		navstate = pos;
		nav_valid = true;
	}
}

void ManualControl::pubEta(const DataType& etaff)
{
	boost::mutex::scoped_lock l(nav_mutex);
	if (!nav_valid) return;

	//Compensate for rotations
	Eigen::Vector3d compff, ff;
	Eigen::Quaternion<double> quat;
	ff<<etaff[X],etaff[Y],etaff[Z];
	labust::tools::quaternionFromEulerZYX(navstate[K], navstate[M], navstate[N],quat);
	compff = quat.toRotationMatrix()*ff;
	//Copy the new X,Y,Z speeds
	DataType etadot(DOF,0);
	for (int i=X; i<compff.size(); ++i) etadot[i] = compff(i);
	for (int i=K; i<DOF; ++i) etadot[i] = etaff[i];

	boost::mutex::scoped_lock lcfg(cfg_mutex);
	for (int i=X; i<DOF; ++i)
	{
		navstate[i] += etadot[i]*config.sampling_time;
		//Wrap angles
		if (i>(DOF/2-1)) navstate[i] = labust::math::wrapRad(navstate[i]);
	}
	//Special handling for altitude
	navstate[A] += etadot[Z]*config.sampling_time;

	auv_msgs::NavSts eta;
	labust::tools::vectorToNED(navstate, eta.position);
	labust::tools::vectorToRPY(navstate, eta.orientation, 3);
	if (config.use_ff)
	{
		labust::tools::vectorToPoint(etaff, eta.body_velocity);
		labust::tools::vectorToRPY(etaff, eta.orientation_rate, 3);
	}
	lcfg.unlock();
	l.unlock();

	eta.header.stamp = ros::Time::now();
	eta.header.frame_id = "local";
	etaref.publish(eta);
};

void ManualControl::pubTau(const DataType& tauv)
{
	auv_msgs::BodyForceReq tau;
	labust::tools::vectorToPoint(tauv, tau.wrench.force);
	labust::tools::vectorToPoint(tauv, tau.wrench.torque, 3);
	tau.header.stamp = ros::Time::now();
	tau.header.frame_id = "base_link";
	tauref.publish(tau);
}

void ManualControl::pubNu(const DataType& nuv)
{
	auv_msgs::BodyVelocityReq nu;
	labust::tools::vectorToPoint(nuv, nu.twist.linear);
	labust::tools::vectorToPoint(nuv, nu.twist.angular, 3);
	nu.header.stamp = ros::Time::now();
	nu.header.frame_id = "base_link";
	nuref.publish(nu);
}

void ManualControl::remap(const sensor_msgs::Joy::ConstPtr& joy)
{
	boost::mutex::scoped_lock l(cfg_mutex);
	for (int i=0;i<DOF;++i)
	{
		//Track disabling of axes
		if (config.axes_map[i] == CRequest::DISABLED)
		{
			mapped[i] = 0;
			continue;
		}
		//Check if joystick has enough axes
		if (joy->axes.size() <= i) continue;

		if (config.integrated[i] > 0)
		{
			mapped[i] += joy->axes[config.axes_map[i]]*config.scale_map[i]*config.integrated[i];
			mapped[i] = labust::math::coerce(mapped[i], -1, 1);
		}
		else
		{
			mapped[i] = joy->axes[config.axes_map[i]]*config.scale_map[i];
		}
	}
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"manual");

	labust::control::ManualControl controller;
	ros::spin();

	return 0;
}



