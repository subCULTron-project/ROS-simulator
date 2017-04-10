/*
 * ESCClassic.hpp
 *
 *  Created on: Apr 13, 2015
 *      Author: Filip Mandic
 */

#ifndef ESCCLASSIC_HPP_
#define ESCCLASSIC_HPP_

#include <labust/control/esc/EscPerturbationBase.hpp>

namespace labust{
	namespace control{
		namespace esc{


			class EscClassic : public EscPerturbationBase<double> {

			public:

				EscClassic(int ctrlNum, numericprecission Ts);

				~EscClassic();

				 void initController(double sin_amp, double sin_freq, double corr_gain, double high_pass_pole, double low_pass_pole, double comp_zero, double comp_pole, double period);

				 numericprecission preFiltering(numericprecission cost_signal);

				 vector gradientEstimation(numericprecission cost_signal_filtered, vector additional_input);

				 vector postFiltering(vector estimated_gradient);

				 vector controllerGain(vector postFiltered);

				 vector superimposePerturbation(vector control);

				 /***
				 * K - gain
				 * A0 - perturbation amplitude
				 */
				vector sin_amp_, sin_freq_, corr_gain_, low_pass_pole_, comp_pole_, comp_zero_;
				vector control_ref_, signal_demodulated_old_, lpf_out_old_,corr_signal_, phase_shift_,comp_old_;
				/*** Controlled state */
				vector state_;

				numericprecission high_pass_pole_, hpf_out_old_, obj_val_old_;
			};
		}
	}
}





#endif /* ESCCLASSIC_HPP_ */
