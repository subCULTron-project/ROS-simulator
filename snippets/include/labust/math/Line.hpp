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
#ifndef LINE_HPP_
#define LINE_HPP_
#include <Eigen/Dense>

namespace labust
{
	namespace math
	{
		/**
		 * This is a helper class for the line calculations.
		 */
		class Line
		{
			enum {xp,yp,zp};
		public:
			/**
			 * Default constructor.
			 */
			Line():
				Gamma(0),
				Xi(0),
				T1(Eigen::Vector3d::Zero()),
				T2(Eigen::Vector3d::Ones()){};
			/**
			 * Calculates the horizontal distance from the line.
			 */
			double calculatedH(double x0, double y0, double z0) const
			{
			  double Lp = sqrt((T2(xp)-T1(xp))*(T2(xp)-T1(xp))+(T2(yp)-T1(yp))*(T2(yp)-T1(yp)));
			  //Note the minus thingy
			  return ((T2(xp)-T1(xp))*(T1(yp)-y0)-(T1(xp)-x0)*(T2(yp)-T1(yp)))/Lp;
			}
			/**
			 * Calculate the vertical distance from the line.
			 */
			double calculatedV(double x0, double y0, double z0) const
			{
			  double Lp = sqrt(std::pow(T2(xp)-T1(xp),2) + std::pow(T2(yp)-T1(yp),2));
			  double L = sqrt(Lp*Lp + std::pow(T2(zp) - T1(zp),2));
			  double n = (T2(xp)-T1(xp))*(x0-T1(xp))+(T2(yp)-T1(yp))*(y0-T1(yp));
			  n *= -(T2(zp)-T1(zp));
			  n -= (T1(zp)-z0)*Lp*Lp;

			  return n/(Lp*L);
			}
			/**
			 * Set the line parameters.
			 *
			 * \param T1 Position of the line start.
			 * \param T2 Position of the line end.
			 */
			void setLine(const Eigen::Vector3d& T1, const Eigen::Vector3d& T2)
			{
			  this->T1 = T1;
			  this->T2 = T2;
			  //select default north line.
			  if ((T1(0) == T2(0)) && (T1(1) == T2(1))) this->T2(0) = -1;

			  this->Gamma = atan2(T2(yp) - T1(yp),T2(xp) - T1(xp));
			  this->Xi = atan2(T2(zp) - T1(zp),
			  		sqrt(std::pow(T2(xp)-T1(xp),2) + std::pow(T2(yp)-T1(yp),2)));
			}
			/**
			 * Get the line parameter.
			 */
			inline const Eigen::Vector3d& getT1()
			{
				return T1;
			}
			/**
			 * Get the line parameter.
			 */
			inline const Eigen::Vector3d& getT2()
			{
				return T2;
			}
			/**
			 * Get the line elevation angle.
			 */
			inline double xi(){return this->Xi;};
			/**
			 * Get the line azimuth angle.
			 */
			inline double gamma(){return this->Gamma;};

		protected:
			/**
			 * Line orientation.
			 */
			double Gamma, Xi;
			/**
			 * Line points.
			 */
			Eigen::Vector3d T1,T2;
		};
	}
}

/* LINE_HPP_ */
#endif
