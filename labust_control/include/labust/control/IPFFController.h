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
 *********************************************************************/
#ifndef IPFFCONTROLLER_H_
#define IPFFCONTROLLER_H_
#include <labust/control/PIDBase.h>
#include <math.h>
/**
 * Autotune the IP controller based on the supplied
 * PT1Model and desired closed loop frequency.
 * \todo Document the binomial autotuning (pole-placement)
 */
void IPFF_modelTune(PIDBase* self,
		const PT1Model* const model,
		float w);
/**
 * Autotune the IP controller using the plant model 1.
 * Valid for higher level controllers.
 */
void IPFF_tune(PIDBase* self, float w);
/**
 * Calculate one step of the IP controller with externally
 * supplied error and feedforward calculation.
 */
void IPFF_wffStep(PIDBase* self, float Ts, float error, float ff);
/**
 * Calculate one step of the IPFF controller.
 */
inline void IPFF_step(PIDBase* self, float Ts)
{
	IPFF_wffStep(self, Ts,
			self->desired - self->state,
			(self->model.betaa*fabs(self->state))*self->state);
}
/**
 * Calculate one step of the IPFF controller with externally
 * supplied error calculation.
 */
inline void IPFF_wStep(PIDBase* self, float Ts, float error)
{
	IPFF_wffStep(self, Ts,
			error,
			(self->model.betaa*fabs(self->state))*self->state);
}
/**
 * Calculate one step of the IPFF controller with externally
 * supplied feedforward calculation.
 */
inline void IPFF_ffStep(PIDBase* self, float Ts, float ff)
{
	IPFF_wffStep(self, Ts,
			self->desired - self->state,
			ff);
}

/* IPFFCONTROLLER_H_ */
#endif
