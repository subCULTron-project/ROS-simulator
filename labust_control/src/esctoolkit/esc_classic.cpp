/*
 * esc_classic.cpp
 *
 *  Created on: Dec 15, 2014
 *      Author: Filip Mandic
 */

#include <labust/control/esc/EscClassic.hpp>

#include <ros/ros.h>

namespace labust{
	namespace control{
		namespace esc{

			typedef EscClassic Base;

			Base::EscClassic(int ctrlNum, numericprecission Ts):EscPerturbationBase<double>(ctrlNum, Ts),
										state_(vector::Zero(controlNum)){


				lpf_out_old_.resize(controlNum);
				signal_demodulated_old_.resize(controlNum);
				comp_old_.resize(controlNum);
				corr_signal_.resize(controlNum);
				control_ref_.resize(controlNum);

				sin_amp_.resize(controlNum);
				sin_freq_.resize(controlNum);
				gain_.resize(controlNum);
				control_.resize(controlNum);
				low_pass_pole_.resize(controlNum);
				comp_pole_.resize(controlNum);
				comp_zero_.resize(controlNum);

				lpf_out_old_.setZero();
				hpf_out_old_ = 0;
				control_ref_ = state_;
				lpf_out_old_.setZero();
				signal_demodulated_old_.setZero();
				comp_old_.setZero();
				corr_signal_.setZero();

				phase_shift_.resize(controlNum);
				phase_shift_[0] = 0;
				for (size_t i = 1; i<controlNum; i++){
					phase_shift_[i] = i*M_PI/((double)controlNum);
				}
				state_initialized_ = true;

			}

			Base::~EscClassic(){

			}

			void Base::initController(double sin_amp, double sin_freq, double corr_gain, double high_pass_pole, double low_pass_pole, double comp_zero, double comp_pole, double Ts){
				sin_amp_.setConstant(sin_amp);
				sin_freq_.setConstant(sin_freq);
				gain_.setConstant(corr_gain);
				high_pass_pole_ = high_pass_pole;
				low_pass_pole_.setConstant(low_pass_pole);
				comp_pole_.setConstant(comp_pole);
				comp_zero_.setConstant(comp_zero);
				Ts_ = Ts;
				obj_val_old_ = 0;
				cycle_count_ = 0;
				hpf_out_old_ = 0;
				//opt_dim_ = 0;
				state_initialized_ = false;
				old_vals_initialized_ = false;
				initialized_ = true;
			}
			Base::numericprecission Base::preFiltering(numericprecission cost_signal){

				 // Check this limit
				numericprecission filtered_cost = (-(Ts_*high_pass_pole_-2)*pre_filter_output_old_+2*cost_signal-2*pre_filter_input_old_)/(2+high_pass_pole_*Ts_);
				ROS_ERROR("filtered_cost: %f", filtered_cost);

				filtered_cost = (abs(filtered_cost)> 0.5)?filtered_cost/abs(filtered_cost)*0.5:filtered_cost;

				ROS_ERROR("filtered_cost after limit: %f", filtered_cost);

				return filtered_cost;
			}

			Base::vector Base::gradientEstimation(numericprecission cost_signal_filtered, vector additional_input){

				vector signal_demodulated(controlNum);

				for (size_t i = 0; i<controlNum; i++){

					signal_demodulated(i)= cost_signal_filtered*sin_amp_(i)*std::sin(double(cycle_count_*Ts_*sin_freq_(i) + phase_shift_(i)));
				}

				return signal_demodulated;
			}

			Base::vector Base::postFiltering(vector estimated_gradient){

				vector lpf_out(controlNum);
				vector comp_out(controlNum);

				for (size_t i = 0; i<controlNum; i++){

					if(low_pass_pole_[i] == 0)
						lpf_out[i] = estimated_gradient[i];
					else
						lpf_out[i] = ((2.0-low_pass_pole_[i]*Ts_)*lpf_out_old_[i]+low_pass_pole_[i]*Ts_*estimated_gradient[i]+low_pass_pole_[i]*Ts_*estimated_gradient_old_[i])/(2.0+low_pass_pole_[i]*Ts_);

					if(comp_pole_[i] == 0)
						comp_out[i] = lpf_out[i];
					else
						comp_out[i]= ((2.0+Ts_*comp_zero_[i])*lpf_out[i]+(Ts_*comp_zero_[i]-2.0)*lpf_out_old_[i]-(Ts_*comp_pole_[i]-2.0)*comp_old_[i])/(2.0+Ts_*comp_pole_[i]);

					lpf_out_old_[i] = lpf_out[i];
					comp_old_[i] = comp_out[i];
				}

				return comp_out;
			}

			Base::vector Base::controllerGain(vector postFiltered){

				ROS_ERROR("GAIN:");
				ROS_ERROR_STREAM(gain_);

				control_ = gain_.cwiseProduct(postFiltered);
				return control_;
			}

			Base::vector Base::superimposePerturbation(Base::vector control){

				for (size_t i = 0; i<controlNum; i++){
					if(!old_vals_initialized_)
						control_ref_[i] = sin_amp_[i]*std::sin(double(cycle_count_*Ts_*sin_freq_[i] + phase_shift_[i]+M_PI/2));
					else
						control_ref_[i] = control[i]+sin_amp_[i]*std::sin(double(cycle_count_*Ts_*sin_freq_[i] + phase_shift_[i]+M_PI/2));
				}

				return control_ref_;
			}
		}
	}
}





