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
#include <labust/ros/diver_sim.h>
#include <labust/math/NumberManipulation.hpp>

#include <auv_msgs/NavSts.h>
#include <sensor_msgs/JointState.h>

using namespace labust::simulation;

DiverSim::DiverSim():
		jnames(20,""),
		jdefaults(60,0),
		move_head(false),
		runSim(false),
		Ts(0.1)
{
	this->onInit();
}

void DiverSim::onInit()
{
	ros::NodeHandle nh,ph("~");

	//Look first in the simulation namespace
	nh.param("simulation/sampling_time",Ts,Ts);
	//Look in the private namespace for override
	ph.param("sampling_time", Ts, Ts);
	//Configure model
	model.setTs(Ts);

	//Load joint names and defaults
	ph.param("joint_names",jnames,jnames);
	ph.param("joint_defaults",jdefaults,jdefaults);
	ph.param("move_head", move_head, move_head);

	if (jdefaults.size() != 3*jnames.size())
	{
		ROS_ERROR("Parameter joint_defaults needs to have 3x the size of parameter joint_names.");
		throw std::runtime_error("Invalid size of parameter joint_defaults");
	}

	//Setup the default joint position
	for(int i=0; i<jnames.size(); ++i)
	{
		jstate.name.push_back(jnames[i] + "_x");
		jstate.name.push_back(jnames[i] + "_y");
		jstate.name.push_back(jnames[i] + "_z");
        jstate.position.push_back(jdefaults[3*i]);
        jstate.position.push_back(jdefaults[3*i+1]);
        jstate.position.push_back(jdefaults[3*i+2]);
	}

	navsts = nh.advertise<auv_msgs::NavSts>("state",1);
	joints = nh.advertise<sensor_msgs::JointState>("joint_states",1);
	nusub = nh.subscribe<auv_msgs::BodyVelocityReq>("diver_nu",1,&DiverSim::onNu,this);

	runner = boost::thread(boost::bind(&DiverSim::start, this));
}

void DiverSim::start()
{
	runSim = true;
	ros::Rate rate(1/Ts);
	while (ros::ok() && runSim)
	{
		this->step();
		rate.sleep();
	}
}

void DiverSim::step()
{
	double hpan(0),htilt(0);
	auv_msgs::NavSts::Ptr nav(new auv_msgs::NavSts());
	{
		boost::mutex::scoped_lock l(nu_mux);
		labust::tools::vectorToPoint(nu, nav->body_velocity);
		labust::tools::vectorToRPY(nu, nav->orientation_rate,3);
		if (move_head)
		{
			enum {p = 3, q =4};
			hpan = nu(p);
			htilt = nu(q);
			//Reset rates to avoid influence on body
			nu(p) = 0;
			nu(q) = 0;
		}
		//Simulate
		model.step(nu);
		if (move_head)
		{
			enum {p = 3, q =4};
			//Recover speeds for future use
			nu(p) = hpan;
			nu(q) = htilt;
		}
	}

	labust::tools::vectorToNED(model.position(), nav->position);
	labust::tools::vectorToRPY(model.orientation(), nav->orientation);
	//Clear the roll/pitch axes to allow for head control
	nav->orientation.roll = 0;
	nav->orientation.pitch = 0;
	nav->header.stamp = ros::Time::now();
	navsts.publish(nav);

	//Assume the interval of oscillations is proportional to speed (it's not, but anyways)
	this->stepJoints(nav->body_velocity.x*4, hpan, htilt);
	jstate.header.stamp = ros::Time::now();
	joints.publish(jstate);
}

void DiverSim::stepJoints(double w, double hpan, double htilt)
{
	const double FOOT_LAG(M_PI/2-M_PI/12);
	const double THIGH_LAG(-M_PI/12);
	static double t=0;
	static double w0=0;
	static double T=0.4;
	///\todo Remove hard-coded positions and find index by name
	enum {LCALF=3*12+1, RCALF=3*17+1,
		LFOOT=3*11+1, RFOOT=3*18+1,
		LTHIGH=3*13+1, RTHIGH=3*16+1,
		LBACKZ=3*14+2, LBACKX=3*14,
		HEADZ = 3*3+2, HEADY = 3*3+1};
	double C = w0*t;
	w0 = (w*Ts + w0*T)/(Ts+T);
	t = ((fabs(w0) > 0.01)?C/w0:0);
	t += Ts;

	jstate.position[LCALF] = jdefaults[LCALF] + M_PI/12*sin(w0*t);
	jstate.position[LFOOT] = jdefaults[LFOOT] - M_PI/12*sin(w0*t+FOOT_LAG);
	jstate.position[LTHIGH] = jdefaults[LTHIGH] + M_PI/36*sin(w0*t+THIGH_LAG);
	jstate.position[RCALF] = jdefaults[RCALF] - M_PI/12*sin(w0*t);
	jstate.position[RFOOT] = jdefaults[RFOOT] + M_PI/12*sin(w0*t+FOOT_LAG);
	jstate.position[RTHIGH] = jdefaults[RTHIGH] - M_PI/36*sin(w0*t+THIGH_LAG);

	//jstate.position[LBACKZ] = labust::math::wrapRad(jdefaults[LBACKZ] - M_PI/96*sin(w0*t));
	//jstate.position[LBACKX] = jdefaults[LBACKX] - M_PI/96*sin(w0*t);

	//Head state
	jstate.position[HEADZ] = labust::math::wrapRad(jdefaults[HEADZ] + hpan);
	jstate.position[HEADY] = labust::math::wrapRad(jdefaults[HEADY] + htilt);
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"diver_sim");
	ros::NodeHandle nh;

	labust::simulation::DiverSim diver_simulator;

	ros::spin();
	return 0;
}
