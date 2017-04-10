/*
 * esc_ekf_grad.cpp
 *
 *  Created on: Dec 22, 2014
 *      Author: Filip Mandic
 */
#include <labust/control/esc/EscEKF.hpp>

#include <ros/ros.h>

namespace labust{
	namespace control{
		namespace esc{

			typedef EscEkfGrad Base;

			Base::EscEkfGrad(int ctrlNum, numericprecission Ts):EscPerturbationBase<double>(ctrlNum, Ts),
										state_(vector::Zero(controlNum)){

				signal_demodulated_old_.resize(controlNum);
				control_ref_.resize(controlNum);
				sin_amp_.resize(controlNum);
				sin_freq_.resize(controlNum);
				gain_.resize(controlNum);
				control_.resize(controlNum);

				control_ref_ = state_;
				signal_demodulated_old_.setZero();

				A = matrix::Identity(3,3);
				L = matrix::Identity(3,3);
				H = matrix::Zero(3,3);
				M = matrix::Identity(3,3);

				Pk_plu = 1.0e-3*matrix::Identity(3,3);
				xk_plu = vector::Zero(3);

				//Qk = 1e-0*diag([0.75 0.75 0.5]); % Process noise vector OVO JE DOBRO 2
				//Rk = 1e-0*diag([1 1 1]); % Measurement noise vector

				Eigen::Vector3d tmp;
				//tmp << 10, 10, 4;
				tmp << 0.75, 0.75, 0.5;
				Q = tmp.asDiagonal();
				//tmp << 0.01, 0.01, 0.01;
				tmp << 1, 1, 1;
				R = tmp.asDiagonal();

				n1 = vector::Zero(3);
				n2 = vector::Zero(3);
				yk = vector::Zero(3);
				input = vector::Zero(6);

				phase_shift_.resize(controlNum);
				phase_shift_[0] = 0;

				for (size_t i = 1; i<controlNum; i++){
					phase_shift_[i] = i*M_PI/((double)controlNum);
				}
				state_initialized_ = true;
			}

			Base::~EscEkfGrad(){

			}

			void Base::initController(double sin_amp, double sin_freq, double corr_gain, double high_pass_pole, double low_pass_pole, double comp_zero, double comp_pole, double Ts, vector Q0, vector R0){

				sin_amp_.setConstant(sin_amp);
				sin_freq_.setConstant(sin_freq);
				gain_.setConstant(corr_gain);
				Ts_ = Ts;
				cycle_count_ = 0;

				Q = Q0.asDiagonal();
				R = R0.asDiagonal();

				state_initialized_ = false;
				old_vals_initialized_ = false;
				initialized_ = true;
			}


			Base::vector Base::gradientEstimation(numericprecission cost_signal_filtered, vector additional_input){

				vector signal_demodulated(controlNum);

				input << additional_input(0), n1(0), n2(0), additional_input(1), n1(1), n2(1);
				yk << cost_signal_filtered, n1(2), n2(2);

				n2 = n1;
				n1 << additional_input, cost_signal_filtered;

				H << input(0), input(3), 1,
					 input(1), input(4), 1,
					 input(2), input(5), 1;

				Pk_min = A*Pk_plu*A.transpose() + L*Q*L.transpose();
				xk_min =  modelUpdate(xk_plu, input);

				hk = outputUpdate(xk_min, input);
				matrix tmp = H*Pk_min*H.transpose()+M*R*M.transpose();
				Kk = (Pk_min*H.transpose())*tmp.inverse();

				xk_plu = xk_min+Kk*(yk-hk);
				Pk_plu = (matrix::Identity(3,3)-Kk*H)*Pk_min;
				input_past = input;

				signal_demodulated << xk_plu(0), xk_plu(1);
				return signal_demodulated;
			}


			Base::vector Base::controllerGain(vector postFiltered){
				control_ = gain_.cwiseProduct(postFiltered);
				ROS_ERROR("GAIN");
				ROS_ERROR_STREAM(gain_);
				ROS_ERROR("CONTROL");
				ROS_ERROR_STREAM(control_);
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

			Base::vector Base::modelUpdate(vector state, vector input){

				return A*state;
			}

			Base::vector Base::outputUpdate(vector state, vector input){
				// outputUpdate = [u1k*dF1+u2k*dF2+d u1k1*dF1+u2k1*dF2+d u1k2*dF1+u2k2*dF2+d].';

				vector output(3);
				output << state(0)*input(0)+state(1)*input(3)+state(2),
						  state(0)*input(1)+state(1)*input(4)+state(2),
						  state(0)*input(2)+state(1)*input(5)+state(2);
				return output;
			}
		}
	}
}





