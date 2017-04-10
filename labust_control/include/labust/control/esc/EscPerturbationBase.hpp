/*********************************************************************
 * EscPerturbation.hpp
 *
 *  Created on: Dec 15, 2014
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

#ifndef ESCPERTURBATIONBASE_HPP_
#define ESCPERTURBATIONBASE_HPP_

#include <Eigen/Dense>
#include <stdint.h>

#include <ros/ros.h>

namespace labust{
	namespace control{
		namespace esc{

		/*************************************************************
		 *** Abstract class definition
		 ************************************************************/
			template <typename precission = double>
			class EscPerturbationBase {

			public:

				typedef precission numericprecission;

				typedef Eigen::Matrix<precission, Eigen::Dynamic, Eigen::Dynamic> matrix;
				typedef Eigen::Matrix<precission, Eigen::Dynamic, 1> vector;

				EscPerturbationBase(int ctrlNum, numericprecission Ts):Ts_(Ts),cycle_count_(0),controlNum(ctrlNum){

					state_initialized_ = false;
					initialized_ = false;
					old_vals_initialized_ = false;
				}

			    virtual ~EscPerturbationBase(){}

				/*****************************************************
				 *** Class functions
				 ****************************************************/

				 virtual numericprecission preFiltering(numericprecission cost_signal){

					 return cost_signal;
				 }

				 virtual vector gradientEstimation(numericprecission cost_signal_filtered, vector additional_input) = 0;

				 virtual vector postFiltering(vector estimated_gradient){
					 return estimated_gradient;
				 }

				 virtual vector controllerGain(vector postFiltered){
					 control_ = control_+gain_.cwiseProduct(postFiltered)*Ts_;
					 return control_;
				 }

				 virtual vector superimposePerturbation(vector control) = 0;

				 virtual vector step(numericprecission cost_signal, vector additional_input = vector::Zero(2) ){

					 numericprecission filtered_cost =  preFiltering(cost_signal);

					 vector estimated_gradient = gradientEstimation(filtered_cost, additional_input);

					 vector control = controllerGain(postFiltering(estimated_gradient));

					 vector controlInput =  superimposePerturbation(control);

					 pre_filter_input_old_ = cost_signal;
					 pre_filter_output_old_ = filtered_cost;
					 estimated_gradient_old_ = estimated_gradient;

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
				numericprecission pre_filter_input_old_, pre_filter_output_old_;
				vector estimated_gradient_old_, control_, gain_;

			};
		}
	}
}

#endif /* ESCPERTURBATION_HPP_ */
