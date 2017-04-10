/*********************************************************************
 * EscBounded.hpp
 *
 *  Created on: Jan 30, 2015
 *      Author: Filip Mandic
 *
 ********************************************************************/

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, LABUST, UNIZG-FER
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

#ifndef ESCBOUNDEDBASE_HPP_
#define ESCBOUNDEDBASE_HPP_

#include <Eigen/Dense>
#include <stdint.h>

namespace labust{
	namespace control{
		namespace esc{

		/*************************************************************
		 *** Abstract class definition
		 ************************************************************/
			template <typename precission = double>
			class EscBoundedBase {

			public:

				typedef precission numericprecission;

				typedef Eigen::Matrix<precission, Eigen::Dynamic, Eigen::Dynamic> matrix;
				typedef Eigen::Matrix<precission, Eigen::Dynamic, 1> vector;

				EscBoundedBase(int ctrlNum, numericprecission Ts):Ts_(Ts),cycle_count_(0),controlNum(ctrlNum){

					state_initialized_ = false;
					initialized_ = false;
					old_vals_initialized_ = false;
				}

			    virtual ~EscBoundedBase(){}

				/*****************************************************
				 *** Class functions
				 ****************************************************/

				 virtual vector generateControl(numericprecission argument){
					 //control_ = control_+gain_.cwiseProduct(postFiltered)*Ts_;
					 return vector::Zero(controlNum);
				 }

				 virtual numericprecission calculateArgument(numericprecission cost){
					 return K_*cost+omega_*Ts_*cycle_count_;
				 }

				 virtual vector step(numericprecission cost_signal, vector additional_input = vector::Zero(2) ){

					 numericprecission argument =  calculateArgument(cost_signal);
					 vector controlInput =  generateControl(argument);

					 old_vals_initialized_ = true;
					 cycle_count_++;

					 return controlInput;
				 }

				virtual void reset(){

					state_initialized_ = false;
					initialized_ = false;
					old_vals_initialized_ = false;
					cycle_count_ = 0;
				}

				/*****************************************************
				 *** General parameters
				 ****************************************************/

				/*** Sampling time */
				numericprecission Ts_;

				/*** Cycle */
				uint32_t cycle_count_;

				/*** Status flags */
				bool state_initialized_, initialized_, old_vals_initialized_;

				/*** Number of control inputs (states) */
				int controlNum;

				/*** ES variables */
				numericprecission K_, omega_, alpha_;


			};
		}
	}
}


#endif /* ESCBOUNDED_HPP_ */
