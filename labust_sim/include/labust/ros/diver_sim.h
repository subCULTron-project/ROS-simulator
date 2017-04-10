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
#ifndef LABUST_SIM_DIVERSIM_H
#define LABUST_SIM_DIVERSIM_H
#include <labust/simulation/kinematic_model.h>

#include <sensor_msgs/JointState.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <ros/ros.h>

#include <boost/thread.hpp>

#include <string>
#include <vector>

namespace labust
{
	namespace simulation
	{
		/**
		 *  This class implements the diver simulator. The class implements joint-state publishing
		 *  for diver movement emulation and uses the kinematic model for diver movement.
		 */
		class DiverSim
		{
		public:
			///The generic constructor.
			DiverSim();

			///Initializes the ROS node and configures the model from the ROS parameters.
			void onInit();

			///Start the simulator
			void start();

		private:
			///Single simulation step
			void step();
			///Step joints
			void stepJoints(double w, double hpan=0, double htilt=0);
			///Handle incoming speed request
			void onNu(const auv_msgs::BodyVelocityReq::ConstPtr& nu)
			{
				boost::mutex::scoped_lock l(nu_mux);
				labust::tools::pointToVector(nu->twist.linear, this->nu);
				labust::tools::pointToVector(nu->twist.angular, this->nu,3);
			}

			///Navigation state publisher
			ros::Publisher navsts;
			///Joint state publisher
			ros::Publisher joints;
			///Subscriber to commanded speeds
			ros::Subscriber nusub;
			///The incoming speed vector
			labust::simulation::vector nu;
			///The incoming speed mutex
			boost::mutex nu_mux;

			///Joint names
			std::vector<std::string> jnames;
			///Joint defaults
			std::vector<double> jdefaults;
			///Current joint state
			sensor_msgs::JointState jstate;
			/**
			 * Flag to use roll/pitch rate for moving the head
			 * instead of the body directly.
			 */
			bool move_head;

			///Sim sampling time
			double Ts;
			///Simulated kinematics
			labust::simulation::KinematicModel model;

			///Runner thread for the simulator
			boost::thread runner;
			///Run flag
			bool runSim;
		};
	}
}

/* LABUST_SIM_DIVERSIM_H */
#endif
