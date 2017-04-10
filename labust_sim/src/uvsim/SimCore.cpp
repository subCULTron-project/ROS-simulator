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
#include <labust/ros/SimCore.hpp>
#include <labust/tools/DynamicsLoader.hpp>
#include <labust/tools/conversions.hpp>
#include <geometry_msgs/TransformStamped.h>

#include <sstream>

using namespace labust::simulation;

void labust::simulation::configureModel(const ros::NodeHandle& nh, RBModel& model)
{
	labust::tools::loadDynamicsParams(nh, model);

	nh.param("sampling_time",model.dT, model.dT);
	nh.param("coupled",model.isCoupled,model.isCoupled);
	Eigen::Vector3d bdg;
	labust::tools::getMatrixParam(nh,"bounding_ellipsoid",bdg);
	model.ae = bdg(0);
	model.be = bdg(1);
	model.ce = bdg(2);
	labust::tools::getMatrixParam(nh,"eta0",model.eta0);
	labust::tools::getMatrixParam(nh,"nu0",model.nu0);
	labust::tools::getMatrixParam(nh,"current",model.current);

	typedef Eigen::Matrix<double,6,1> NoiseVec;
	NoiseVec pn(NoiseVec::Zero()),mn(NoiseVec::Zero());
	labust::tools::getMatrixParam(nh,"process_noise",pn);
	labust::tools::getMatrixParam(nh,"measurement_noise",mn);
	model.noise.setNoiseParams(pn,mn);

	//Populate allocation
	int alloc_type(-1);
	Eigen::MatrixXd alloc;
	Eigen::MatrixXi dofs, groups;
	nh.param("allocation_type",alloc_type,-1);
    labust::tools::getMatrixParam(nh,"allocation_matrix",alloc);
	labust::tools::getMatrixParam(nh,"allocation_dofs",dofs);
	labust::tools::getMatrixParam(nh,"allocation_thruster_groups",groups);

	model.allocator.init(alloc_type, alloc, dofs, groups);
	model.init();
}

SimCore::SimCore():
		tau(vector::Zero()),
		rate(10),
		wrap(1),
		enablePublishWorld(true),
		enablePublishSimBaseLink(true),
		originLat(0),
		originLon(0)
{
	this->onInit();
}

void SimCore::onInit()
{
	ros::NodeHandle nh,ph("~");
	configureModel(nh, model);
	modelReport();

	//Subscribers
	//auv_msgs
	tauIn = nh.subscribe<auv_msgs::BodyForceReq>("tauIn", 1, &SimCore::onTau<auv_msgs::BodyForceReq>, this);
	//odometry
	tauInWrench = nh.subscribe<geometry_msgs::WrenchStamped>("tauInWrench", 1, &SimCore::onTau<geometry_msgs::WrenchStamped>, this);
	//general
	currentsSub = nh.subscribe<geometry_msgs::TwistStamped>("currents", 1, &SimCore::onCurrents, this);
	positionSub = nh.subscribe<auv_msgs::NavSts>("position_static", 1, &SimCore::onPositionStatic, this);
	calcState = true;

	//Publishers
	//Auv_msgs
	meas = nh.advertise<auv_msgs::NavSts>("meas_ideal",1);
	measn = nh.advertise<auv_msgs::NavSts>("meas_noisy",1);
	tauAch = nh.advertise<auv_msgs::BodyForceReq>("tauAch",1);
	//odometry
	odom = nh.advertise<nav_msgs::Odometry>("meas_odom",1);
	odomn = nh.advertise<nav_msgs::Odometry>("meas_odom_noisy",1);
	tauAchWrench = nh.advertise<geometry_msgs::WrenchStamped>("tauAchWrench",1);
	//general
	acc = nh.advertise<geometry_msgs::Vector3>("nuacc_ideal",1);

	double fs(10);
	double maxThrust(1), minThrust(-1);
	ph.param("maxThrust",maxThrust,maxThrust);
	ph.param("minThrust",minThrust,minThrust);
	model.allocator.setThrusterMinMax(minThrust, maxThrust);
	ph.param("Rate",fs,fs);
	ph.param("ModelWrap", wrap,wrap);
	model.dT = 1/(fs*wrap);
	rate = ros::Rate(fs);
	ph.param("publish_world", enablePublishWorld, enablePublishWorld);
	ph.param("publish_sim_base", enablePublishSimBaseLink, enablePublishSimBaseLink);
	ph.param("originLat", originLat, originLat);
	ph.param("originLon", originLon, originLon);
	runner = boost::thread(boost::bind(&SimCore::start, this));
}

void SimCore::onCurrents(const geometry_msgs::TwistStamped::ConstPtr& currents)
{
	boost::mutex::scoped_lock l(model_mux);
	labust::tools::pointToVector(currents->twist.linear, model.current);
}

void SimCore::onPositionStatic(const auv_msgs::NavSts::ConstPtr& msg)
{
	vector pos = vector::Zero(6);
	pos[0] = msg->position.north;
	pos[1] = msg->position.east;
	pos[2] = msg->position.depth;
	pos[3] = msg->orientation.roll;
	pos[4] = msg->orientation.pitch;
	pos[5] = msg->orientation.yaw;
	model.setNuAndEta(model.Nu(), pos);

	// end of the message flag
	if (msg->status == 1) calcState = true;
	else calcState = false;
}

void SimCore::etaNuToNavSts(const vector& eta, const vector& nu, auv_msgs::NavSts& state)
{
	state.position.north = eta(RBModel::x);
	state.position.east = eta(RBModel::y);
	state.position.depth = eta(RBModel::z);
	state.orientation.roll = eta(RBModel::phi);
	state.orientation.pitch = eta(RBModel::theta);
	state.orientation.yaw = eta(RBModel::psi);

	state.body_velocity.x = nu(RBModel::u);
	state.body_velocity.y = nu(RBModel::v);
	state.body_velocity.z = nu(RBModel::w);
	state.orientation_rate.roll = nu(RBModel::p);
	state.orientation_rate.pitch = nu(RBModel::q);
	state.orientation_rate.yaw = nu(RBModel::r);
}

void SimCore::publishNavSts()
{
	auv_msgs::NavStsPtr state(new auv_msgs::NavSts()),
			nstate(new auv_msgs::NavSts());

	etaNuToNavSts(model.Eta(),model.Nu(),*state);
	etaNuToNavSts(model.EtaNoisy(),model.NuNoisy(),*nstate);

	//Handle LAT-LON additions and frame publishing
	//Or make them wholly external to the simulator

	//Publish messages
	state->header.stamp = nstate->header.stamp = ros::Time::now();
	state->header.frame_id = nstate->header.frame_id = "local";
	meas.publish(state);
	measn.publish(nstate);
}

void SimCore::publishOdom()
{
	nav_msgs::OdometryPtr state(new nav_msgs::Odometry()),
			nstate(new nav_msgs::Odometry());

	etaNuToOdom(model.Eta(),model.Nu(),*state);
	etaNuToOdom(model.EtaNoisy(),model.NuNoisy(),*nstate);

	//Handle LAT-LON additions and frame publishing
	//Handle covariance additions

	//Publish messages
	state->header.stamp = nstate->header.stamp = ros::Time::now();
	state->header.frame_id = nstate->header.frame_id = "local";
	state->child_frame_id = nstate->child_frame_id = "base_link";
	odom.publish(state);
	odomn.publish(nstate);
}

void SimCore::etaNuToOdom(const vector& eta, const vector& nu, nav_msgs::Odometry& state)
{
	labust::tools::vectorToPoint(eta,state.pose.pose.position);

	Eigen::Quaternion<double> quat;
	labust::tools::quaternionFromEulerZYX(eta(RBModel::phi),
			eta(RBModel::theta),
			eta(RBModel::psi), quat);
	state.pose.pose.orientation.x = quat.x();
	state.pose.pose.orientation.y = quat.y();
	state.pose.pose.orientation.z = quat.z();
	state.pose.pose.orientation.w = quat.w();

	labust::tools::vectorToPoint(nu,state.twist.twist.linear);
	labust::tools::vectorToPoint(nu,state.twist.twist.angular,3);
}

void SimCore::publishWorld()
{
	geometry_msgs::TransformStamped transform;
	transform.transform.translation.x = originLon;
	transform.transform.translation.y = originLat;
	transform.transform.translation.z = 0;
	labust::tools::quaternionFromEulerZYX(0, 0, 0,
			transform.transform.rotation);
	transform.child_frame_id = "world";
	transform.header.frame_id = "worldLatLon";
	transform.header.stamp = ros::Time::now();
	broadcast.sendTransform(transform);

	transform.transform.translation.x = 0;
	transform.transform.translation.y = 0;
	transform.transform.translation.z = 0;
	labust::tools::quaternionFromEulerZYX(M_PI,0,M_PI/2,
			transform.transform.rotation);
	transform.child_frame_id = "local";
	transform.header.frame_id = "world";
	transform.header.stamp = ros::Time::now();
	broadcast.sendTransform(transform);
}

void SimCore::publishSimBaseLink()
{
	const vector& eta = model.Eta();

	geometry_msgs::TransformStamped transform;
	transform.transform.translation.x = eta(RBModel::x);
	transform.transform.translation.y = eta(RBModel::y);
	transform.transform.translation.z = eta(RBModel::z);
	labust::tools::quaternionFromEulerZYX(eta(RBModel::phi),
			eta(RBModel::theta),
			eta(RBModel::psi),
			transform.transform.rotation);
	transform.child_frame_id = "base_link_sim";
	transform.header.frame_id = "local";
	transform.header.stamp = ros::Time::now();
	broadcast.sendTransform(transform);
}

void SimCore::start()
{
//	SimSensorInterface::Hook hook(model,broadcast,listener);

	while (ros::ok())
	{
		boost::mutex::scoped_lock model_lock(model_mux);
		if (calcState)
		{
			{
				boost::mutex::scoped_lock l(tau_mux);
				for (size_t i=0; i<wrap;++i) model.step(tau);
			}

	//		{
	//			boost::mutex::scoped_lock l(sensor_mux);
	//			for (std::vector<SimSensorInterface::Ptr>::iterator it=sensors.begin();
	//					it != sensors.end(); ++it)
	//			{
	//				(*it)->step(hook);
	//			}
	//		}
		}
		//Publish states
		publishNavSts();
		publishOdom();
		//Publish transforms
		if (enablePublishWorld) publishWorld();
		if (enablePublishSimBaseLink) publishSimBaseLink();
		//Publish acceleration
		geometry_msgs::Vector3::Ptr accout(new geometry_msgs::Vector3());
		const vector& vacc=model.NuAcc();
		labust::tools::vectorToPoint(vacc, *accout);
		acc.publish(accout);

		model_lock.unlock();
		rate.sleep();
	}
}

void SimCore::modelReport()
{
	ROS_INFO("Loaded the model:");
	ROS_INFO(" sampling-time: %f, mass: %f, gravity: %f, density: %f",
			model.dT, model.m, model.g_acc, model.rho);
	std::ostringstream str;
	str<<"["<<model.eta0.transpose()<<"], ["<<model.nu0.transpose()<<"]"<<"\n";
	ROS_INFO(" (Eta0,Nu0): %s",str.str().c_str());
	str.str("");
	str<<"["<<model.current.transpose()<<"]";
	ROS_INFO(" Current: %s",str.str().c_str());
	ROS_INFO(" Bounding ellipsoid: (%f,%f,%f)",model.ae, model.be, model.ce);
	ROS_INFO(" Is coupled model: %d",model.isCoupled);
	str.str("");
	str<<model.Io<<"\n"<<model.Ma<<"\n"<<model.Dlin<<"\n"<<model.Dquad;
	ROS_INFO("(Io,Ma,Dlin,Dquad):\n%s",str.str().c_str());
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"uvsim");
	ros::NodeHandle nh;

	labust::simulation::SimCore simulator;

	ros::spin();
	return 0;
}
