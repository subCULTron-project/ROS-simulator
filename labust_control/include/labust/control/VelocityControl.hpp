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
#ifndef VELOCITYCONTROL_HPP_
#define VELOCITYCONTROL_HPP_
#include <labust/control/PIDBase.h>
#include <navcon_msgs/VelConConfig.h>
#include <navcon_msgs/ConfigureVelocityController.h>
#include <navcon_msgs/EnableControl.h>
#include <navcon_msgs/ModelParamsUpdate.h>
#include <dynamic_reconfigure/server.h>

#include <auv_msgs/NavSts.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/BodyForceReq.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>

#include <string>
#include <map>

namespace labust
{
	namespace control
	{
		/**
		 * The class contains the implementation of the velocity controller, manual control manager and model identification.
		 * \todo Create the merge subclass for velocity controller via external TAU
		 * \todo Implement the modes (manual, external(control), disable, tracking)
		 * \todo Switch to vector operation rather than repeated statements
		 * \todo Add model retuning
		 * \todo Separate dynamic reconfigure option to optional subclass
		 * \todo Remove other modes
		 * \todo Remove everything related to identification and replace it via the merger
		 * \todo Deprecate measurements subscription
		 * \todo Reduce the internal tauAch code calculation
		 * \todo Switch velocity controller to async mode ?
		 * \todo Add feed forward for disturbances ?
		 */
		class VelocityControl
		{
			enum {u=0,v,w,p,q,r};
			enum {X=0,Y,Z,K,M,N};
			enum {alpha=0,beta,betaa};
			enum {Kp=0,Ki,Kd,Kt};
			enum {disableAxis=0,
				manualAxis=1,
				controlAxis=2,
				identAxis=3,
			  directAxis=4};
			enum {numcnt = 6};

		public:
			/**
			 * Main constructor
			 */
			VelocityControl();
			/**
			 * Initialize and setup controller.
			 */
			void onInit();
			/**
			 * Performs one iteration.
			 */
			void step();
			/**
			 * Start the controller loop.
			 */
			void start();

		private:
			/**
			 * Handle incoming reference message.
			 */
			void handleReference(const auv_msgs::BodyVelocityReq::ConstPtr& ref);
			/**
			 * Handle incoming estimates message.
			 */
			void handleEstimates(const auv_msgs::NavSts::ConstPtr& estimate);
			/**
			 * Handle incoming estimates message.
			 */
			void handleMeasurement(const auv_msgs::NavSts::ConstPtr& meas);
			/**
			 * Handle incoming estimates message.
			 */
			void handleWindup(const auv_msgs::BodyForceReq::ConstPtr& tau);
			/**
			 * Handle incoming estimates message.
			 */
			void handleModelUpdate(const navcon_msgs::ModelParamsUpdate::ConstPtr& update);
			/**
			 * Handle external identification input.
			 */
			void handleExt(const auv_msgs::BodyForceReq::ConstPtr& tau);
			/**
			 * Handle incoming estimates message.
			 */
			void handleManual(const sensor_msgs::Joy::ConstPtr& joy);
			/**
			 * Handle server request for configuration.
			 */
			bool handleServerConfig(navcon_msgs::ConfigureVelocityController::Request& req,
					navcon_msgs::ConfigureVelocityController::Response& resp);
			/**
			 * Handle the enable control request.
			 */
			bool handleEnableControl(navcon_msgs::EnableControl::Request& req,
					navcon_msgs::EnableControl::Response& resp);
			/**
			 * Dynamic reconfigure callback.
			 */
			void dynrec_cb(navcon_msgs::VelConConfig& config, uint32_t level);
			/**
			 * The safety test.
			 */
			void safetyTest();
			/**
			 * Update the dynamic reconfiguration settings.
			 */
			void updateDynRecConfig();
			/**
			 * The ROS node handles.
			 */
			ros::NodeHandle nh,ph;
			/**
			 * Last message times.
			 */
			ros::Time lastRef, lastMan, lastEst, lastMeas;
			/**
			 * Timeout
			 */
			double timeout;

			/**
			 * Initialize the controller parameters etc.
			 */
			void initialize_controller();
			/**
			 * The velocity controllers.
			 */
			PIDBase controller[r+1];
			/**
			 * The mesurement needed for identification.
			 */
			double measurement[r+1];
			/**
			 * Joystick message.
			 */
			float tauManual[N+1], tauExt[N+1];
			/**
			 * Joystick scaling.
			 */
			double joy_scale, Ts;
			/**
			 * Enable/disable controllers, external windup flag.
			 */
			boost::array<int32_t,r+1> axis_control;
			/**
			 * Timeout management.
			 */
			bool suspend_axis[r+1], externalIdent, use_gvel;
			/**
			 * Dynamic reconfigure server.
			 */
			boost::recursive_mutex serverMux;

			/**
			 * The publisher of the TAU message.
			 */
			ros::Publisher tauOut, tauAchW;
			/**
			 * The subscribed topics.
			 */
			ros::Subscriber velocityRef, stateHat, manualIn, tauAch, measSub, identExt, modelUpdate;
			/**
			 * High level controller service.
			 */
			ros::ServiceServer highLevelSelect, enableControl;
			/**
			 * The dynamic reconfigure parameters.
			 */
			navcon_msgs::VelConConfig config;
			/**
			 * The dynamic reconfigure server.
			 */
		  dynamic_reconfigure::Server<navcon_msgs::VelConConfig> server;
		  /**
		   * Variable access helper mass.
		   */
		  const static std::string dofName[r+1];
		  /**
		   * Safety test flag.
		   */
		  bool doSafetyTest;
		};
	}
}

/* VELOCITYCONTROL_HPP_ */
#endif
