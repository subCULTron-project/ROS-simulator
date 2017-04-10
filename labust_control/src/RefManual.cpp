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
#include <labust/control/EnablePolicy.hpp>
#include <labust/control/ManControl.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/MatrixLoader.hpp>

#include <auv_msgs/NavSts.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <std_msgs/Bool.h>

#include <boost/thread/mutex.hpp>

namespace labust
{
	namespace control{
		///The manual reference controller
		struct RefManual
		{
			enum {u=0,v,w,p,q,r,a};
			RefManual():
				nu_max(Eigen::Vector6d::Zero()),
				manRefFlag(Eigen::Vector6i::Zero()),
				manRef(Eigen::MatrixXd::Zero(7,1)),
				Ts(0.1),
				stateReady(false),
				stateAcquired(false),
				useFF(false),
				lastFF(Eigen::Vector6d::Zero()),
				ffstate(Eigen::Vector6d::Zero()),
				yawRef(0.0),
				joyTreshold(0.001),
				fineThresholdYaw(0.5),
				fineThresholdPos(0.25),
				fineThresholdAlt(0.5),
				yawFlag(false),
				enable(false){this->init();};

			void init()
			{
				ros::NodeHandle nh;

				//Initialize publishers
				stateRef = nh.advertise<auv_msgs::NavSts>("stateRef", 1);

				manRefHeadingPub = nh.advertise<std_msgs::Bool>("manRefHeading", 1);
				manRefPositionNorthPub = nh.advertise<std_msgs::Bool>("manRefNorthPosition", 1);
				manRefPositionEastPub = nh.advertise<std_msgs::Bool>("manRefEastPosition", 1);
				manRefPositionDepthPub = nh.advertise<std_msgs::Bool>("manRefDepthPosition", 1);
				manRefAltitudePub = nh.advertise<std_msgs::Bool>("manRefAltitude", 1);


				//Initialize subscribers
				stateHat = nh.subscribe<auv_msgs::NavSts>("stateHat", 1,
						&RefManual::onEstimate,this);

				nuRefSub = nh.subscribe<auv_msgs::BodyVelocityReq>("nuRef", 1,
						&RefManual::onNuRef,this);

				joyIn = nh.subscribe<sensor_msgs::Joy>("joy", 1,
						&RefManual::onJoy,this);
				
				enableControl = nh.advertiseService("Enable",
						&RefManual::onEnableControl, this);

				initialize_manual();
			}
			
			bool onEnableControl(navcon_msgs::EnableControl::Request& req,
					navcon_msgs::EnableControl::Response& resp)
			{
				this->enable = req.enable;
				if (req.enable)
				{
				  manRefFlag = Eigen::Vector6i::Zero();
				  if (stateAcquired)
				  {
					baseRef = lastState;
					stateReady = true;
				  }
				  else
				  {
					stateReady = false;
					return false;
				  }
				}
				else
				{
					stateReady = false;
				}
				
				return true;
			}


			void onEstimate(const auv_msgs::NavSts::ConstPtr& state)
			{
				boost::mutex::scoped_lock l(cnt_mux);
				lastState = *state;
				stateAcquired = true;
				if (!stateReady) baseRef = lastState;
				//Switch occured
				//if (Enable::enable && !stateReady)
				//{
				//	this->baseRef.position = state->position;
				//	this->baseRef.orientation = state->orientation;
				//	this->baseRef.position.depth = state->position.depth;
				//	this->baseRef.altitude = state->altitude;
				//}
				//stateReady = Enable::enable;
			}

			void onNuRef(const auv_msgs::BodyVelocityReq::ConstPtr& state)
			{
				//yawRef = state->twist.angular.z;
			}

			void onJoy(const sensor_msgs::Joy::ConstPtr& joy)
			{
				if (!stateReady) return;
				if (!stateAcquired) return;

				Eigen::Vector6d mapped;
				mapper.remap(*joy, mapped);
				mapped = nu_max.cwiseProduct(mapped);

				Eigen::Vector2f out, in;
				Eigen::Matrix2f R;
				in<<mapped[u],mapped[v];
				double yaw(baseRef.orientation.yaw);
				R<<cos(yaw),-sin(yaw),sin(yaw),cos(yaw);
				out = R*in;

				/*** Check if heading joystick input is active ***/
				std_msgs::Bool data;
				if(std::abs(mapped[r])<fineThresholdYaw)
				{
					data.data = true;
					manRefHeadingPub.publish(data);
					if(manRefFlag[r] == false)
					{
						manRef[r] = lastState.orientation.yaw;
						manRefFlag[r] = true;
					}

					if (std::abs(mapped[r]) > joyTreshold)
					{
						manRef[r] += mapped[r]*Ts;						
					}
				}
				else
				{
					data.data = false;
					manRefHeadingPub.publish(data);
					manRefFlag[r]= false;
				}
				/*** u ***/
				if(std::abs(out[u])<fineThresholdPos)
				{
					data.data = true;
					manRefPositionNorthPub.publish(data);
					if(manRefFlag[u] == false)
					{
						manRef[u] = lastState.position.north;
						manRefFlag[u] = true;
					}

					if (std::abs(out(u)) > joyTreshold)
					{
						manRef[u] += out(u)*Ts;						
					}
				}
				else
				{
					data.data = false;
					manRefPositionNorthPub.publish(data);
					manRefFlag[u]= false;
				}

				/*** v ***/
				if(std::abs(out[v])<fineThresholdPos)
				{
					data.data = true;
					manRefPositionEastPub.publish(data);
					if(manRefFlag[v] == false)
					{
						manRef[v] = lastState.position.east;
						manRefFlag[v] = true;
					}

					if (std::abs(out(v)) > joyTreshold)
					{
						manRef[v] += out(v)*Ts;						
					}
				}
				else
				{
					data.data = false;
					manRefPositionEastPub.publish(data);
					manRefFlag[v]= false;
				}

				/*** w ***/
				if (std::abs(mapped[w]) < fineThresholdAlt)
				{
					data.data = true;
					manRefPositionDepthPub.publish(data);
					manRefAltitudePub.publish(data);

					if(manRefFlag[w] == false)
					{
						manRef[w] = lastState.position.depth;
						manRef[a] = lastState.altitude;
						manRefFlag[w] = true;
					}

					if (std::abs(mapped[w]) > joyTreshold)
					{
						manRef[w] += mapped[w]*Ts;
						manRef[a] -= mapped[w]*Ts;						
					}
				}
				else
				{
					data.data = false;
					manRefPositionDepthPub.publish(data);
					manRefAltitudePub.publish(data);
					manRefFlag[w]= false;
				}

				baseRef.header.stamp = ros::Time::now();
				baseRef.header.frame_id = "local";

				baseRef.position.north = (std::abs(out[u])<fineThresholdPos)?manRef[u]:lastState.position.north;
				baseRef.position.east = (std::abs(out[v])<fineThresholdPos)?manRef[v]:lastState.position.east;
				baseRef.position.depth = (std::abs(mapped[w])<fineThresholdAlt)?manRef[w]:lastState.position.depth;
				baseRef.orientation.roll += mapped[p]*Ts;
				baseRef.orientation.pitch += mapped[q]*Ts;
				baseRef.orientation.yaw = (std::abs(mapped[r])<fineThresholdYaw)?manRef[r]:lastState.orientation.yaw;

				baseRef.altitude = (std::abs(mapped[w])<fineThresholdAlt)?manRef[a]:lastState.altitude;
				
				if (useFF)
				{
				  //double T=0.1;
			      //ffstate = (mapped*Ts + ffstate*T)/(Ts+T);
				  baseRef.body_velocity.x = mapped[u];
				  baseRef.body_velocity.y = mapped[v]; // + (mapped(v) - ffstate(v))/T;
				  baseRef.body_velocity.z = mapped[w];
				  baseRef.orientation_rate.roll = mapped[p];
				  baseRef.orientation_rate.pitch = mapped[q];
				  baseRef.orientation_rate.yaw = mapped[r];

				}
				
				auv_msgs::NavSts::Ptr refOut(new auv_msgs::NavSts());
				*refOut = baseRef;
				refOut->orientation.roll =  labust::math::wrapRad(refOut->orientation.roll);
				refOut->orientation.pitch =  labust::math::wrapRad(refOut->orientation.pitch);
				refOut->orientation.yaw =  labust::math::wrapRad(refOut->orientation.yaw);
				ROS_ERROR("stateRef pub"); //PGA
				stateRef.publish(refOut);
			}

			double guide_test(double baseRef, double state, double speed, double max)
			{
				//V1
				//return baseRef + speed;
				//V2
				if (fabs(speed) < max/100)
				{
				   return baseRef + speed;
				}
				else
				{
				   return state + speed;
				}			
			}

			void initialize_manual()
			{
				ROS_INFO("Initializing manual ref controller...");

				ros::NodeHandle nh;
				labust::tools::getMatrixParam(nh,"ref_manual/maximum_speeds", nu_max);
				nh.param("ref_manual/sampling_time",Ts,Ts);
				nh.param("ref_manual/feedforward_speeds",useFF,useFF);
				nh.param("ref_manual/joy_deadzone",joyTreshold,joyTreshold);
				nh.param("ref_manual/finezone_yaw",fineThresholdYaw,fineThresholdYaw);
				nh.param("ref_manual/finezone_pos",fineThresholdPos,fineThresholdPos);
				nh.param("ref_manual/finezone_alt",fineThresholdAlt,fineThresholdAlt);

				ROS_INFO("Manual ref controller initialized.");
			}

		private:
			ros::Subscriber stateHat, joyIn, nuRefSub;
			ros::Publisher stateRef, manRefHeadingPub, manRefPositionNorthPub,
							manRefPositionEastPub, manRefPositionDepthPub, manRefAltitudePub;
			Eigen::Vector6d nu_max;
			Eigen::VectorXd manRef;
			Eigen::Vector6i manRefFlag;
			double Ts;
			JoystickMapping mapper;
			auv_msgs::NavSts baseRef;
			boost::mutex cnt_mux;
			auv_msgs::NavSts lastState;
			bool stateReady;
			bool stateAcquired;
			bool useFF;
			bool enable;
			bool yawFlag;
			Eigen::Vector6d lastFF;
			Eigen::Vector6d ffstate;
			ros::ServiceServer enableControl;
			double yawRef;
			double joyTreshold;
			double fineThresholdYaw;
			double fineThresholdPos;
			double fineThresholdAlt;
		};
	}}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"ref_manual");

	//labust::control::RefManual<labust::control::EnableServicePolicy> controller;
	labust::control::RefManual controller;
	ros::Duration(10).sleep();
	ros::spin();

	return 0;
}



