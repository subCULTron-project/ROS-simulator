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
#ifndef PIFFCONTROLLER_H_
#define PIFFCONTROLLER_H_
#include <labust/control/PIDBase.h>
#include <math.h>
/**
 * Autotune the PIFF controller based on the supplied
 * PT1Model and desired closed loop frequency.
 * \todo Document the binomial autotuning (pole-placement)
 */
void PIFF_modelTune(PIDBase* self,
		const PT1Model* const model,
		float w, float a=1.5);
/**
 * Autotune the PIFF controller using the plant model 1.
 * Valid for higher level controllers.
 */
void PIFF_tune(PIDBase* self, float w, float a=1.5);
/**
 * Calculate one step of the PIFF controller with externally
 * supplied error and feedforward calculation.
 */
void PIFF_wffStep(PIDBase* self, float Ts, float error, float perror, float ff);
/**
 * The idle step of the controller.
 */
void PIFF_wffIdle(PIDBase* self, float Ts, float error, float perror, float ff);
/**
 * Calculate one step of the PIFF controller.
 */
inline void PIFF_step(PIDBase* self, float Ts)
{
	PIFF_wffStep(self, Ts,
			self->desired - self->state,
			self->b*self->desired - self->state,
			//Feed-forward as feedback linearization
			(self->model.betaa*fabs(self->state))*self->state);
}
/**
 * The idle step of the controller.
 */
void PIFF_idle(PIDBase* self, float Ts)
{
	PIFF_wffIdle(self, Ts,
			self->desired - self->state,
			self->b*self->desired - self->state,
			//Feed-forward as feedback linearization
			(self->model.betaa*fabs(self->state))*self->state);
}
/**
 * Calculate one step of the PIFF controller with externally
 * supplied error calculation.
 */
inline void PIFF_wStep(PIDBase* self, float Ts, float error, float perror)
{
	PIFF_wffStep(self, Ts,
			error,
			perror,
			//Feed-forward as feedback linearization
			(self->model.betaa*fabs(self->state))*self->state);
}
/**
 * The idle step of the controller.
 */
void PIFF_wIdle(PIDBase* self, float Ts, float error, float perror)
{
	PIFF_wffIdle(self, Ts,
			error,
			perror,
			//Feed-forward as feedback linearization
			(self->model.betaa*fabs(self->state))*self->state);
}
/**
 * Calculate one step of the PIFF controller with externally
 * supplied feedforward calculation.
 */
inline void PIFF_ffStep(PIDBase* self, float Ts, float ff)
{
	PIFF_wffStep(self, Ts,
			self->desired - self->state,
			self->b*self->desired - self->state,
			ff);
}
/**
 * The idle step of the controller.
 */
void PIFF_ffIdle(PIDBase* self, float Ts, float ff)
{
	PIFF_wffIdle(self, Ts,
			self->desired - self->state,
			self->b*self->desired - self->state,
			ff);
}

/* PIFFCONTROLLER_H_ */
#endif
