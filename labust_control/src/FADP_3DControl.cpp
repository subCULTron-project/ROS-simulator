/*********************************************************************
 * FADP_3DControl.cpp
 *
 *  Created on: Apr 16, 2015
 *      Author: Dula Nad, Filip Mandic
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
 *********************************************************************/
#include <labust/control/HLControl.hpp>
#include <labust/control/EnablePolicy.hpp>
#include <labust/control/WindupPolicy.hpp>
#include <labust/control/PIFFController.h>
#include <labust/control/IPFFController.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/tools/conversions.hpp>

#include <Eigen/Dense>
#include <auv_msgs/BodyForceReq.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>

namespace labust
{
	namespace control{
		///The fully actuated dynamic positioning controller
		///\todo add tracking support
		struct FADPControl : DisableAxis
		{
			enum {x=0,y, z};

			FADPControl():Ts(0.1){};

			void init()
			{
				ros::NodeHandle nh;
				initialize_controller();
			}

			void windup(const auv_msgs::BodyForceReq& tauAch)
			{
				bool joint_windup = tauAch.windup.x || tauAch.windup.y;
				con[x].extWindup = joint_windup;
				con[y].extWindup = joint_windup;
				con[z].extWindup = tauAch.windup.z;

			};

			void idle(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state,
							const auv_msgs::BodyVelocityReq& track)
			{
				//Tracking external commands while idle (bumpless)
				Eigen::Quaternion<float> q;
				Eigen::Vector3f in, out;
				labust::tools::quaternionFromEulerZYX(
						state.orientation.roll,
						state.orientation.pitch,
						state.orientation.yaw, q);
				in<<track.twist.linear.x,track.twist.linear.y,track.twist.linear.z;
				out = q.matrix()*in;
				con[x].desired = state.position.north;
				con[y].desired = state.position.east;
				con[z].desired = state.position.depth;
				con[x].track = out(x);
				con[y].track = out(y);
				con[z].track = out(z);
				con[x].state = state.position.north;
				con[y].state = state.position.east;
				con[z].state = state.position.depth;

				labust::tools::quaternionFromEulerZYX(
						ref.orientation.roll,
						ref.orientation.pitch,
						ref.orientation.yaw, q);
				in<<ref.body_velocity.x,ref.body_velocity.y,ref.body_velocity.z;
				out = q.matrix()*in;

				PIFF_ffIdle(&con[x], Ts, float(out(x)));
				PIFF_ffIdle(&con[y], Ts, float(out(y)));
				PIFF_ffIdle(&con[z], Ts, float(out(z)));
			};

			void reset(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state)
			{
				//UNUSED
			};

			auv_msgs::BodyVelocityReqPtr step(const auv_msgs::NavSts& ref,
							const auv_msgs::NavSts& state)
			{
				con[x].desired = ref.position.north;
				con[y].desired = ref.position.east;
				con[z].desired = ref.position.depth;
				con[x].state = state.position.north;
				con[y].state = state.position.east;
				con[z].state = state.position.depth;
				//Calculate tracking values
				Eigen::Quaternion<float> qs,qr;
				Eigen::Vector3f in, out;
				labust::tools::quaternionFromEulerZYX(
						state.orientation.roll,
						state.orientation.pitch,
						state.orientation.yaw, qs);
				in<<state.body_velocity.x,state.body_velocity.y,state.body_velocity.z;
				out = qs.matrix()*in;
				con[x].track = out(x);
				con[x].track = out(y);
				con[x].track = out(z);
				//Calculate feed forward
				ROS_DEBUG("Position desired: %f %f %f", ref.position.north, ref.position.east, ref.position.depth);
				labust::tools::quaternionFromEulerZYX(
						ref.orientation.roll,
						ref.orientation.pitch,
						ref.orientation.yaw, qr);
				in<<ref.body_velocity.x,ref.body_velocity.y,ref.body_velocity.z;
				out = qr.matrix()*in;
				//Make step
				PIFF_ffStep(&con[x], Ts, float(out(x)));
				PIFF_ffStep(&con[y], Ts, float(out(y)));
				PIFF_ffStep(&con[z], Ts, float(out(z)));

				//Publish commands
				auv_msgs::BodyVelocityReqPtr nu(new auv_msgs::BodyVelocityReq());
				nu->header.stamp = ros::Time::now();
				nu->goal.requester = "fadp_3d_controller";
				labust::tools::vectorToDisableAxis(disable_axis, nu->disable_axis);
				in<<con[x].output,con[y].output,con[z].output;
				out = qs.matrix()*in;
				nu->twist.linear.x = out[x];
				nu->twist.linear.y = out[y];
				nu->twist.linear.z = out[z];

				return nu;
			}

			void initialize_controller()
			{
				ROS_INFO("Initializing dynamic positioning controller...");

				ros::NodeHandle nh;
				Eigen::Vector3d closedLoopFreq(Eigen::Vector3d::Ones());
				labust::tools::getMatrixParam(nh,"dp_controller/closed_loop_freq", closedLoopFreq);
				nh.param("dp_controller/sampling",Ts,Ts);

				disable_axis[x] = 0;
				disable_axis[y] = 0;
				disable_axis[z] = 0;


				enum {Kp=0, Ki, Kd, Kt};
				for (size_t i=0; i<=2;++i)
				{
					PIDBase_init(&con[i]);
					PIFF_tune(&con[i], float(closedLoopFreq(i)));
				}

				ROS_INFO("Dynamic positioning controller initialized.");
			}

		private:
			PIDBase con[3];
			double Ts;
		};
	}}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"fadp_3d_control");

	labust::control::HLControl<labust::control::FADPControl,
	labust::control::EnableServicePolicy,
	labust::control::WindupPolicy<auv_msgs::BodyForceReq> > controller;
	ros::spin();

	return 0;
}



