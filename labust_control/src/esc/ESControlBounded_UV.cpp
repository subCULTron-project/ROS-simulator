/*********************************************************************
 * ESControlBounded_UV.cpp
 *
 *  Created on: Jan 30, 2015
 *      Author: Filip Mandic
 *
 ********************************************************************/

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, LABUST, UNIZG-FER
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

#include <labust/control/esc/EscBounded.hpp>
#include <labust/control/HLControl.hpp>
#include <labust/control/EnablePolicy.hpp>
#include <labust/control/WindupPolicy.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/tools/conversions.hpp>

#include <Eigen/Dense>
#include <auv_msgs/BodyForceReq.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>

namespace labust{
	namespace control{

		/*************************************************************
		 *** Extremum seeking controller with speed control inputs.
		 ************************************************************/

		struct ESCControlBounded_UV : DisableAxis{

			enum {x = 0, y};

			ESCControlBounded_UV():Ts(0.1), esc_controller(2,Ts), count(0),newRange(false), esc_Ts(1.0){};

			void init(){

				ros::NodeHandle nh;
				initialize_controller();
				subRange = nh.subscribe<std_msgs::Float32>("range",1,& ESCControlBounded_UV::onRange,this);

			}

			void onRange(const std_msgs::Float32::ConstPtr& msg){
				newRange = true;
			}

  		void windup(const auv_msgs::BodyForceReq& tauAch){

		};

  		void idle(const std_msgs::Float32& ref, const auv_msgs::NavSts& state,
  				const auv_msgs::BodyVelocityReq& track){

  		};

  		void reset(const std_msgs::Float32& ref, const auv_msgs::NavSts& state){

  		};

		auv_msgs::BodyVelocityReqPtr step(const std_msgs::Float32& ref, const auv_msgs::NavSts& state){

			//if((count++)%20 == 0){
							int t_step = esc_Ts/Ts;
							//if(newRange){
							if((count++)%t_step == 0){

								newRange = false;

				Eigen::Vector2d out, in;
				Eigen::Matrix2d R;

				in = esc_controller.step(ref.data);

				auv_msgs::BodyVelocityReqPtr nu(new auv_msgs::BodyVelocityReq());
				nu->header.stamp = ros::Time::now();
				nu->goal.requester = "esc_controller_bounded";
				labust::tools::vectorToDisableAxis(disable_axis, nu->disable_axis);

				double yaw = state.orientation.yaw;
				R<<cos(yaw),-sin(yaw),sin(yaw),cos(yaw);
				out = R.transpose()*in;

				nu->twist.linear.x = out[x];
				nu->twist.linear.y = out[y];

				nu_past = nu;
				return nu;
			}else{

				ROS_ERROR("PAST");
				return nu_past;
			}
		}

			void initialize_controller(){

				ROS_INFO("Initializing extremum seeking controller...");

				ros::NodeHandle nh;

//				double K = 1.5;
//				double	alpha = 1;
//				double	omega =  0.5;

//				double K = 1.5;
//				double	alpha = 1;
//				double	omega =  0.75;

				double K = 1.1;
				double	alpha = 0.7;
				double	omega =  0.3;
				double sampling_time = 0.1;

				nh.param("esc_bounded/K", K, K);
				nh.param("esc_bounded/alpha", alpha, alpha);
				nh.param("esc_bounded/omega", omega, omega);
				nh.param("esc_bounded/sampling_time", sampling_time, sampling_time);

				esc_Ts = sampling_time;

				esc_controller.initController(K, omega, alpha, sampling_time);

				disable_axis[x] = 0;
				disable_axis[y] = 0;

				ROS_INFO("Extremum seeking controller initialized.");
			}

		private:

			double Ts;
			esc::EscBounded esc_controller;
			auv_msgs::BodyVelocityReqPtr nu_past;
			int count;
			double esc_Ts;
						bool newRange;
						ros::Subscriber subRange;
		};
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"esc_classic_control");

	/***
	template <
			class Controller,
			class Enable = NoEnable,
			class Windup = NoWindup,
			class OutputType = auv_msgs::BodyVelocityReq,
			class InputType = auv_msgs::NavSts,
			class ReferenceType = auv_msgs::NavSts
			>
	***/

	labust::control::HLControl<labust::control::ESCControlBounded_UV,
	labust::control::EnableServicePolicy,
	labust::control::NoWindup,
	auv_msgs::BodyVelocityReq,
	auv_msgs::NavSts,
	std_msgs::Float32> controller;
	ros::spin();

	return 0;
}







