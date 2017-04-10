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
#include <labust/control/HLControl.hpp>
#include <labust/control/EnablePolicy.hpp>
#include <labust/control/PIFFController.h>
#include <labust/control/IPFFController.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/tools/conversions.hpp>

#include <Eigen/Dense>
#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/FSPathInfo.h>
#include <ros/ros.h>

namespace labust
{
	namespace control{
		///The underactuated virtual target controller
		struct UVTControl : DisableAxis
		{
			enum {u=0,r=5};

			UVTControl():
				Ts(0.1),
				k1(1.0),
				k2(1.0),
				kpsi(1.0),
				psia(M_PI/4){};

			void init()
			{
				initializeController();
			}

			void idle(const auv_msgs::FSPathInfo& ref, const auv_msgs::NavSts& state,
							const auv_msgs::BodyVelocityReq& track){};

			void reset(const auv_msgs::FSPathInfo& ref, const auv_msgs::NavSts& state){};

			auv_msgs::BodyVelocityReqPtr step(const auv_msgs::FSPathInfo& ref,
							const auv_msgs::NavSts& state)
			{
				con.desired = ref.orientation.yaw;
				con.state = (useIP?unwrap(state.orientation.yaw):state.orientation.yaw);
				con.track = state.orientation_rate.yaw;

				float errorWrap = labust::math::wrapRad(
								con.desired - con.state);
				//Zero feed-forward
				if (useIP)
				{
					IPFF_wffStep(&con,Ts, errorWrap, ref.orientation_rate.yaw);
				}
				else
				{
					PIFF_wffStep(&con,Ts, errorWrap, ref.orientation_rate.yaw);
				}


				auv_msgs::BodyVelocityReqPtr nu(new auv_msgs::BodyVelocityReq());
				labust::tools::vectorToDisableAxis(disable_axis, nu->disable_axis);
				nu->header.stamp = ros::Time::now();
				nu->goal.requester = "uvt_controller";

				double ddelta = -psia*kpsi*(-ref.curvature* sdot * s1 + vt*sin(psi))/cosh(kpsi*y1)^2;
				nu->twist.angular.z = ddelta - k1*(psi-delta) + 1/radius*sdot;

				return nu;
			}

			void initializeController()
			{
				ros::NodeHandle nh;
				nh.param("uvt_controller/closed_loop_freq", closedLoopFreq, closedLoopFreq);
				nh.param("uvt_controller/sampling",Ts,Ts);

				disable_axis[r] = 0;
				disable_axis[u] = 0;

				ROS_INFO("Heading controller initialized.");
			}

		private:
			///Sampling time
			double Ts;
			///S1 parameter
			double k1;
			///Y1 parameter
			double k2;
			///Psi tracking parameter
			double kpsi;
			///The attack angle
			double psia;
		};
	}}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"uvt_control");

	labust::control::HLControl<labust::control::UVTControl,
	labust::control::EnableServicePolicy,
	labust::control::NoWindup,
	auv_msgs::BodyVelocityReq,
	auv_msgs::NavSts,
	auv_msgs::SFPathInfo> controller;


	ros::spin();

	return 0;
}



