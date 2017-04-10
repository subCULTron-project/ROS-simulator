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
#ifndef ASYNCMERGER_HPP_
#define ASYNCMERGER_HPP_
#include <labust/tools/conversions.hpp>

#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

#include <vector>
#include <string>
#include <map>

namespace labust
{
	namespace control
	{
		class AsyncMergerBase
		{
			typedef std::map<std::string,bool> ControllerMap;
		public:
			typedef boost::shared_ptr<AsyncMergerBase> Ptr;
		protected:
			/**
			 * Main constructor.
			 */
			AsyncMergerBase();
			/**
			 * Main destructor.
			 */
			virtual ~AsyncMergerBase(){};
			/**
			 * Helper method to copy values from force topic to array.
			 */
			void copyToVector(const auv_msgs::BodyForceReq::ConstPtr& topic, bool* axis, double* val);
			/**
			 * Helper method to copy values from force topic to array.
			 */
			void copyFromVector(const auv_msgs::BodyForceReq::Ptr& topic, bool* axis, double* val);
			/**
			 * Helper method to copy values from velocity topic to array.
			 */
			void copyToVector(const auv_msgs::BodyVelocityReq::ConstPtr& topic, bool* axis, double* val);
			/**
			 * Helper method to copy values from velocity topic to array.
			 */
			void copyFromVector(const auv_msgs::BodyVelocityReq::Ptr& topic, bool* axis, double* val);

			/**
			 * Method to handle new and old requesters.
			 */
			void handleRequester(const std::string& requester);
			/**
			 * Method to determine if all controller messages have arrived.
			 */
			bool allArrived();
			/**
			 * Method clears all arrived flags.
			 */
			void clearAll();
			/**
			 * Helper method to reset axes.
			 */
			void reset_axes();
			/**
			 * Check if all controllers arrived.
			 */
			void checkArrived(const ros::TimerEvent& event);
			/**
			 * The final publish.
			 */
			virtual void publishFinal() = 0;

			/**
			 * The highlevel controller set.
			 */
			ControllerMap controllers;
			/**
			 * Subscriber to the Topic.
			 */
			ros::Subscriber in;
			/**
			 * Publisher on the merged Topic.
			 */
			ros::Publisher out, loop_time;
			/**
			 * The sync timer.
			 */
			ros::Timer timer;
			/**
			 * Last time all arrived.
			 */
			ros::Time lastAll;

			/**
			 * The final disable_axis vector.
			 */
			bool outaxis[6];
			/**
			 * The final value vector.
			 */
			double outval[6];
			/**
			 * Data mutex.
			 */
			boost::mutex cnt_mux;
			/**
			 * Sampling time.
			 */
			double Ts;
		};


		template <class Topic>
		struct AsyncMerger : public virtual AsyncMergerBase
		{

		public:
			AsyncMerger(){this->onInit();};

			void onInit()
			{
				ros::NodeHandle nh;
				in = nh.subscribe<Topic>("in", 0, &AsyncMerger::onTopic, this);
				out = nh.advertise<Topic>("out",1);
			}

		private:
			/**
			 * The topic handler.
			 */
			void onTopic(const typename Topic::ConstPtr& topic)
			{
				bool axis[6];
				double val[6];
				this->copyToVector(topic,axis,val);

				boost::mutex::scoped_lock l(cnt_mux);
				for(int i=0; i<6; ++i)
				{
					if (!axis[i])
					{

						outaxis[i] = axis[i];
						outval[i] = val[i];
					}
				}
				this->handleRequester(topic->goal.requester);
			}

			/**
			 * Publish final data.
			 */
			void publishFinal()
			{
				typename Topic::Ptr outTopic(new Topic());
				outTopic->header.stamp = ros::Time::now();
				outTopic->goal.requester = "async_merger";

				this->copyFromVector(outTopic, outaxis, outval);
				out.publish(outTopic);
				std_msgs::Float32 data;
				data.data = (ros::Time::now() - lastAll).toSec();
				loop_time.publish(data);
			}

		};
	}
}
/* ASYNCMERGER_HPP_ */
#endif
