/*
 * esc_bounded.cpp
 *
 *  Created on: Jan 30, 2015
 *      Author: Filip Mandic
 */

#include <labust/control/esc/EscBounded.hpp>

#include <ros/ros.h>

namespace labust{
	namespace control{
		namespace esc{

			typedef EscBounded Base;

			Base::EscBounded(int ctrlNum, numericprecission Ts):EscBoundedBase<double>(ctrlNum, Ts),
										state_(vector::Zero(controlNum)){

				state_initialized_ = true;
			}

			Base::~EscBounded(){

			}

			void Base::initController(double K, double omega, double alpha, double Ts){

				K_ = K;
				omega_ = omega;
				alpha_ = alpha;
				Ts_ = Ts;
				cycle_count_ = 0;

				state_initialized_ = false;
				old_vals_initialized_ = false;
				initialized_ = true;
			}

			Base::vector Base::generateControl(numericprecission argument){

				vector control = vector::Zero(2);
				double speed = std::sqrt(alpha_*omega_);
				control(0) = speed*std::cos(argument);
				control(1) = speed*std::sin(argument);
				return control;
			}
		}
	}
}



