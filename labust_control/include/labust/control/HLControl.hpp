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
 *  Created on: 26.06.2013.
 *  Author: Dula Nad
 *********************************************************************/
#ifndef HLCONTROL_HPP_
#define HLCONTROL_HPP_
#include <auv_msgs/NavSts.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <navcon_msgs/ConfigureAxes.h>
#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

namespace labust
{
	namespace control
	{
		struct NoEnable{static const bool enable=true;};
		struct NoWindup{
			template <class Base>
			inline void get_windup(Base b){};
		};

		struct DisableAxis
		{
			DisableAxis()
			{
				for (int i=0; i<6; ++i) disable_axis[i]=1;
			}
			bool disable_axis[6];
		};

		///\todo Reverse the disable/enable axes logic at one time ?
		class ConfigureAxesPolicy
		{
		public:
			ConfigureAxesPolicy()
			{
				ros::NodeHandle nh;
				configureAxes = nh.advertiseService("ConfigureAxes",
						&ConfigureAxesPolicy::onConfiguration, this);
			}

			bool onConfiguration(navcon_msgs::ConfigureAxes::Request& req,
					navcon_msgs::ConfigureAxes::Response& resp)
			{
				this->disable_axis = req.disable_axis;
				return true;
			}

		protected:
			/**
			 * Axes setup.
			 */
			auv_msgs::Bool6Axis disable_axis;
			/**
			 * High level controller service.
			 */
			ros::ServiceServer configureAxes;
		};

		/**
		 * The class contains the ROS template for high level controllers.
		 *
		 * \todo Add windup type. Convert into template. EnablePolicy add service, topic option selection.
		 * \todo Consider joining the windup type and output type - this will require republishing of windup up the cascade ?
		 */
		template <
		class Controller,
		class Enable = NoEnable,
		class Windup = NoWindup,
		class OutputType = auv_msgs::BodyVelocityReq,
		class InputType = auv_msgs::NavSts,
		class ReferenceType = auv_msgs::NavSts
		>
		class HLControl : public Controller, Enable, Windup
		{
			enum {TIMEOUT=2};
		public:
			/**
			 * Main constructor
			 */
			HLControl():
				lastEn(false),
				refOk(false)
			{
				onInit();
			}
			/**
			 * Initialize and setup controller.
			 */
			void onInit()
			{
				ros::NodeHandle nh;
				//Initialize publishers
				outPub = nh.advertise<OutputType>("out", 1);

				//Initialize subscribers
				stateSub = nh.subscribe<InputType>("state", 1,
						&HLControl::onEstimate,this);
				trackState = nh.subscribe<ReferenceType>("ref", 1,
						&HLControl::onRef,this);
				extSub = nh.subscribe<OutputType>("track", 1,
						&HLControl::onTrack,this);

				Controller::init();
			}

		private:

			void onRef(const typename ReferenceType::ConstPtr& ref)
			{
				boost::mutex::scoped_lock l(cnt_mux);
				refOk = true;
				this->ref = *ref;
			}

			void onTrack(const typename OutputType::ConstPtr& ext)
			{
				boost::mutex::scoped_lock l(cnt_mux);
				this->ext = *ext;
			}

			void onEstimate(const typename InputType::ConstPtr& estimate)
			{
				if (!Enable::enable)
				{
					this->idle(ref, *estimate, ext);
					lastEn = false;
					refOk = false;
					return;
				}

				if (!refOk) return;

				//Copy into controller
				Windup::get_windup(this);
				boost::mutex::scoped_lock l(cnt_mux);
				if (Enable::enable && !lastEn)
				{
					//this->reset(ref, *estimate);
					lastEn = true;
				}
				outPub.publish(Controller::step(ref, *estimate));
			}

			/**
			 * The publisher of the TAU message.
			 */
			ros::Publisher outPub;
			/**
			 * The subscribed topics.
			 */
			ros::Subscriber stateSub, trackState, extSub;
			/**
			 * The desired state to track.
			 */
			ReferenceType ref;
			///The external achieved reference
			OutputType ext;
			/**
			 * The disabled axis.
			 */
			bool disabled_axis[6], lastEn, refOk;
			/**
			 * Control locker.
			 */
			boost::mutex cnt_mux;
		};
	}
}

/* HLCONTROL_HPP_ */
#endif
