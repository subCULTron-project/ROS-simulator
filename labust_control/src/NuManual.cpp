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
#include <labust/control/ManControl.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/tools/conversions.hpp>

#include <auv_msgs/BodyVelocityReq.h>

namespace labust
{
	namespace control{
		///The nu manual controller
		template <class Enable>
		struct NuManual : public Enable, ConfigureAxesPolicy
		{
			NuManual():
			nu_max(Eigen::Vector6d::Zero())
			{this->init();};

			void init()
			{
				ros::NodeHandle nh;
				joy = nh.subscribe<sensor_msgs::Joy>("joy", 1,
						&NuManual::onJoy,this);
				nuRef = nh.advertise<auv_msgs::BodyVelocityReq>("nuRef", 1);

				initialize_manual();
			}

			void onJoy(const sensor_msgs::Joy::ConstPtr& joyIn)
			{
				if (!Enable::enable) return;
				auv_msgs::BodyVelocityReq::Ptr nu(new auv_msgs::BodyVelocityReq());
				Eigen::Vector6d mapped;
				mapper.remap(*joyIn, mapped);

				nu->header.stamp = ros::Time::now();
				nu->header.frame_id = "base_link";
				nu->goal.requester = "nu_manual";
				nu->disable_axis = this->disable_axis;

				mapped = nu_max.cwiseProduct(mapped);
				labust::tools::vectorToPoint(mapped, nu->twist.linear);
				labust::tools::vectorToPoint(mapped, nu->twist.angular, 3);

				nuRef.publish(nu);
			}

			void initialize_manual()
			{
				ROS_INFO("Initializing manual nu controller...");

				ros::NodeHandle nh;
				labust::tools::getMatrixParam(nh,"nu_manual/maximum_speeds", nu_max);

				ROS_INFO("Manual nu controller initialized.");
			}

		private:
			Eigen::Vector6d nu_max;
			ros::Subscriber joy;
			ros::Publisher nuRef;
			JoystickMapping mapper;
		};
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"nu_manual");

	labust::control::NuManual<labust::control::EnableServicePolicy> controller;
	ros::spin();

	return 0;
}



