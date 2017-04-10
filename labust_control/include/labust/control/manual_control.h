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
*********************************************************************/
#ifndef LABUST_CONTROL_MANUAL_CONTROL_H
#define LABUST_CONTROL_MANUAL_CONTROL_H

#include <navcon_msgs/ManualConfigure.h>
#include <navcon_msgs/ManualSelect.h>
#include <navcon_msgs/ManualConfiguration.h>
#include <auv_msgs/NavSts.h>
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

#include <string>
#include <map>

namespace labust
{
	namespace control
	{
		/**
		 * The class contains the implementation of a manual reference generator
		 * for \tau, \nu and \eta. The class uses the joystick topic of ROS to
		 * remap the joystick data. The controller is driven by the joystick topic input.
		 *
		 * The class performs joystick mappings from received axes to regular tau axes.
		 * Tau axes are defined as (X,Y,Z,K,M,N) and (-1,1). Joystick axes are defined
		 * as: (1-n) where n is the maximum axis number. The mapping is a row vector (1x6)
		 * (axes_map) that has the desired axis defined in the according place of the
		 * tau axis. For disabled axes define the axis number -1.
		 * The scaling is defined in (scale_map) as a desired double number.
		 */
		class ManualControl
		{
			enum {X=0,Y,Z,K,M,N,DOF=6,A=6};

			typedef boost::array<int,DOF> GenArray;
			typedef std::vector<double> DataType;
			typedef navcon_msgs::ManualConfigure::Request CRequest;
			typedef navcon_msgs::ManualConfigure::Response CResponse;
			typedef navcon_msgs::ManualSelect::Request SRequest;
			typedef navcon_msgs::ManualSelect::Response SResponse;

		public:
			///Main constructor
			ManualControl();

			///Initialize and setup controller.
			void onInit();

		private:
			///Handle the incoming joystick message.
			void onJoystick(const sensor_msgs::Joy::ConstPtr& joy);
			///Handle the current navigation state.
			void onNavSts(const auv_msgs::NavSts::ConstPtr& nav);
			///On server configuration
			bool onConfiguration(CRequest& req, CResponse& resp);
			///On generator selection
			bool onSelect(SRequest& req, SResponse& resp);

			///Load the configuration
			bool setupConfig();
			///Helper method to set the defaults values of configuration
			void setDefaultConfig();
			///Remap the joystick
			void remap(const sensor_msgs::Joy::ConstPtr& joy);
			///Publish the Tau values
			void pubTau(const DataType& tauv);
			///Publish the Nu values
			void pubNu(const DataType& nuv);
			///Publish the Eta values
			void pubEta(const DataType& etaff);

			///\tau manual reference publisher
			ros::Publisher tauref;
			///\nu manual reference publisher
			ros::Publisher nuref;
			///\eta manual reference publisher
			ros::Publisher etaref;
			///Joystick subscriber
			ros::Subscriber joyin;
			///Navigation state subscriber
			ros::Subscriber navsts;
			///Configuration service server.
			ros::ServiceServer config_server;
			///Selection service
			ros::ServiceServer select_server;

			///The node configuration
			navcon_msgs::ManualConfiguration config;
			///The current generators
			GenArray generators;
			///The remapped joystick values
			DataType mapped;
			///Config and selector mutex
			boost::mutex cfg_mutex;

			///The current navigation state
			DataType navstate;
			///Navigation state validity flag
			bool nav_valid;
			///Nav-mutex protector
			boost::mutex nav_mutex;
		};
	}
}

/* LABUST_CONTROL_MANUAL_CONTROL_H */
#endif
