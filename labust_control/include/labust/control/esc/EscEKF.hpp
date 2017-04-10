/*
 * EscEKF.hpp
 *
 *  Created on: Apr 13, 2015
 *      Author: Filip Mandic
 */

#ifndef ESCEKF_HPP_
#define ESCEKF_HPP_

#include <labust/control/esc/EscPerturbationBase.hpp>

namespace labust{
	namespace control{
		namespace esc{


			class EscEkfGrad : public EscPerturbationBase<double> {

			public:

				EscEkfGrad(int ctrlNum, numericprecission Ts);

				~EscEkfGrad();

				 void initController(double sin_amp, double sin_freq, double corr_gain, double high_pass_pole, double low_pass_pole, double comp_zero, double comp_pole, double period, vector Q, vector R);

				 vector gradientEstimation(numericprecission cost_signal_filtered, vector additional_input);

				 vector controllerGain(vector postFiltered);

				 vector superimposePerturbation(vector control);

				 vector modelUpdate(vector state, vector input);

				 vector outputUpdate(vector state, vector input);

				 /***
				 * K - gain
				 * A0 - perturbation amplitude
				 */
				vector sin_amp_, sin_freq_, corr_gain_;
				vector control_ref_, signal_demodulated_old_, phase_shift_;
				/*** Controlled state */
				vector state_;

				/*** EKF */

				matrix A, L, H, M, Q, R;
				matrix Pk_plu, Pk_min, Kk;
				vector xk_plu, xk_min, hk;

				vector input, input_past;

				/*** Measurement vector */
				vector yk;

				Eigen::Vector3d n1, n2;

			};
		}
	}
}




#endif /* ESCEKF_HPP_ */
