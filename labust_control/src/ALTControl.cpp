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
#include <labust/control/WindupPolicy.hpp>
#include <labust/control/PSatDController.h>
#include <labust/control/IPFFController.h>
#include <labust/control/PIFFController.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/tools/conversions.hpp>

#include <Eigen/Dense>
#include <auv_msgs/BodyForceReq.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

namespace labust
{
	namespace control{
		///The altitude/depth controller
		struct ALTControl : DisableAxis
		{
			ALTControl():Ts(0.1),manRefFlag(true),depth_threshold(0.1),depth_timeout(10){};

			void init()
			{
				ros::NodeHandle nh;
				manRefSub = nh.subscribe<std_msgs::Bool>("manRefAltitude",1,&ALTControl::onManRef,this);
				initialize_controller();
			}

			void onManRef(const std_msgs::Bool::ConstPtr& state)
			{
				manRefFlag = state->data;
			}
  		void windup(const auv_msgs::BodyForceReq& tauAch)
			{
				//Copy into controller
  			con.extWindup = tauAch.windup.z;
			};

  		void idle(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state,
  				const auv_msgs::BodyVelocityReq& track)
  		{
  			//Tracking external commands while idle (bumpless)
  			con.desired = state.altitude;
  			con.state = state.altitude;
  			con.track = -track.twist.linear.z;
  			PIFF_ffIdle(&con, Ts, -ref.body_velocity.z);
  		};

  		void reset(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state)
  		{
  			//UNUSED
  		};

			auv_msgs::BodyVelocityReqPtr step(const auv_msgs::NavSts& ref,
					const auv_msgs::NavSts& state)
			{
				con.desired = ref.altitude;
				con.state = state.altitude;
				con.track = -state.body_velocity.z;

				double tmp_output;
				auv_msgs::BodyVelocityReqPtr nu(new auv_msgs::BodyVelocityReq());

				if(manRefFlag)
				{
					if (state.position.depth > depth_threshold) underwater_time = ros::Time::now();
					//If more than depth_timeout on surface stop the controller
					if (((ros::Time::now() - underwater_time).toSec() > depth_timeout)) 
					{
						PIFF_ffIdle(&con, Ts, 0);
						tmp_output = 0;
					}
					else
					{
						//PIFF_wffStep(&con,Ts, werror, wperror, 0*ref.orientation_rate.yaw);
						PIFF_ffStep(&con, Ts, 0*(-1)*ref.body_velocity.z);
						tmp_output = -con.output;
					}

					//tmp_output = -con.output;
				}
				else
				{
					PIFF_ffIdle(&con, Ts, -ref.body_velocity.z);
					tmp_output = ref.body_velocity.z;
				}

				nu->header.stamp = ros::Time::now();
				nu->goal.requester = "alt_controller";
				labust::tools::vectorToDisableAxis(disable_axis, nu->disable_axis);

				nu->twist.linear.z = tmp_output;

				return nu;
			}

			void initialize_controller()
			{
				ROS_INFO("Initializing altitude controller...");

				ros::NodeHandle nh;
				double closedLoopFreq(1);
				nh.param("alt_controller/closed_loop_freq", closedLoopFreq, closedLoopFreq);
				nh.param("alt_controller/sampling",Ts,Ts);
				nh.param("depth_controller/depth_threshold",depth_threshold,depth_threshold);
				nh.param("depth_controller/depth_timeout",depth_timeout,depth_timeout);

				disable_axis[2] = 0;

				PIDBase_init(&con);
				PIFF_tune(&con, float(closedLoopFreq));

				ROS_INFO("Altitude controller initialized.");
			}

		private:
			ros::Subscriber alt_sub;
			PIDBase con;
			double Ts;
			double depth_threshold;
			double depth_timeout;
			ros::Time underwater_time;
			ros::Subscriber manRefSub;
			bool manRefFlag;
		};
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"alt_control");

	labust::control::HLControl<labust::control::ALTControl,
	labust::control::EnableServicePolicy,
	labust::control::WindupPolicy<auv_msgs::BodyForceReq> > controller;

	ros::spin();

	return 0;
}



