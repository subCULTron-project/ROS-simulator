/*
 * EscBounded.hpp
 *
 *  Created on: Apr 13, 2015
 *      Author: filip
 */

#ifndef ESCBOUNDED_HPP_
#define ESCBOUNDED_HPP_

#include <labust/control/esc/EscBoundedBase.hpp>

namespace labust{
	namespace control{
		namespace esc{


			class EscBounded : public EscBoundedBase<double> {

			public:

				EscBounded(int ctrlNum, numericprecission Ts);

				~EscBounded();

				 void initController(double K, double omega, double alpha, double Ts);

				 vector generateControl(numericprecission argument);

				/*** Controlled state */
				vector state_;
			};
		}
	}
}



#endif /* ESCBOUNDED_HPP_ */
