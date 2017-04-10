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
#ifndef MANCONTROL_HPP_
#define MANCONTROL_HPP_
#include <labust/tools/MatrixLoader.hpp>

#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

#include <Eigen/Dense>

namespace Eigen
{
	typedef Eigen::Matrix<double,6,1> Vector6d;
	typedef Eigen::Matrix<int,6,1> Vector6i;
}

namespace labust
{
	namespace control
	{
		/**
		 * The class performs joystick mappings from received axes to
		 * regular tau axes. Tau axes are defined as (X,Y,Z,K,M,N). Joystick
		 * axes are defined as: (1-n) where n is the maximum axis number.
		 * The mapping is a row vector (1x6) (joystick_mapping/axes_map) that
		 * has the desired axis defined in the according place of the tau axis.
		 * For disabled axis define the axis number -1.
		 * The scaling is defined in (joystick_mapping/scale_map) as a desired double
		 * number.
		 * \deprecated
		 */
		struct JoystickMapping
		{
		public:
			/**
			 * Main constructor.
			 */
			JoystickMapping()
			{
				ros::NodeHandle nh;

				//Set default values
				axes_map<<1,0,3,-1,-1,2;
				scale_map<<1,-1,-1,0,0,-1;

				//Load configurations
				if (nh.hasParam("joystick_mapping/axes_map"))
				{
					labust::tools::getMatrixParam(nh, "joystick_mapping/axes_map", axes_map);
				}

				if (nh.hasParam("joystick_mapping/scale_map"))
				{
					labust::tools::getMatrixParam(nh, "joystick_mapping/scale_map", scale_map);
				}
			}

			void remap(const sensor_msgs::Joy& joy, Eigen::Vector6d& mapped)
			{

				for (int i=0;i<6;++i)
				{
					if (axes_map[i] != -1)
						mapped(i) = joy.axes[axes_map[i]]*scale_map[i];
					else
						mapped(i) = 0;
				}
			}

		private:
			Eigen::Vector6i axes_map;
			Eigen::Vector6d scale_map;
		};
	}
}

/* MANCONTROL_HPP_ */
#endif
