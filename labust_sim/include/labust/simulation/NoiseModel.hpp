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
*  Created: 01.02.2010.
*********************************************************************/
#ifndef NOISEMODEL_HPP_
#define NOISEMODEL_HPP_
#include <labust/simulation/matrixfwd.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/nondet_random.hpp>

namespace labust
{
	namespace simulation
	{
		/**
		 * The class incorporates the process and measurement noise used for
		 * the UUV simulation.
		 */
		class NoiseModel
		{
			typedef boost::variate_generator<boost::random_device&, boost::normal_distribution<> > noise_generator;
			typedef boost::shared_ptr<noise_generator> ng_ptr;
		public:
			/**
			 * Generic constructor initializes noise deviance to zero.
			 */
			NoiseModel():
				w(vector::Zero()),
				v(vector::Zero())
			{
				this->setNoiseParams(vector::Zero(),vector::Zero());
			}

			/**
			 * Set the process and measurements noise parameters.
			 *
			 * \param pn Standard deviance of the process noise.
			 * \param mn Standard deviance of the measurement noise.
			 */
			void setNoiseParams(const vector& pn, const vector& mn)
			{
			  for (size_t i=0; i<ngW.size();++i)
			  {
			  	ngV[i].reset(new noise_generator(rd,boost::normal_distribution<>(0,pn(i))));
			  	ngW[i].reset(new noise_generator(rd,boost::normal_distribution<>(0,mn(i))));
			  }
			}

			/**
			 * The method returns the process noise vector.
			 *
			 * \return The process noise.
			 */
			inline const vector& calculateW()
			{
				for(size_t i=0; i<ngW.size(); ++i)	w(i) = (*ngW[i])();
				return w;
			}
			/**
			 * The method returns the measurement noise vector.
			 *
			 * \return The process noise.
			 */
			inline const vector& calculateV()
			{
				for(size_t i=0; i<ngV.size(); ++i) 	v(i) = (*ngV[i])();
				return v;
			}

		private:
			/**
			 * The process and measurement noise vectors.
			 */
			vector w,v;
			/**
			 * Stochastic noise generator.
			 */
			boost::random_device rd;
			/**
			 * Gaussian distributions for noise.
			 */
			boost::array<ng_ptr,6> ngW,ngV;
		};
	}
}
/* NOISEMODEL_HPP_ */
#endif
