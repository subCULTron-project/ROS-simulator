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
#ifndef PIDBASE_H_
#define PIDBASE_H_
/**
 * The extended PT1 model used for vehicle. The model can be used
 * to auto-tune the PIDController class. The model is defined as:
 *
 *  alpha * nu' = (beta + betaa * abs(nu))*nu + tau
 */
typedef struct PT1Model
{
	/**
	 * The inertia parameter.
	 */
	float alpha;
	/**
	 * The linear drag parameter.
	 */
	float beta;
	/**
	 * The non-linear drag parameter.
	 */
	float betaa;
} PT1Model;

/**
 * The PID controller base.
 * \todo Use doxygen grouping and reduce size
 * \todo Document difference between auto-windup and ext-windup
 * \todo Rename lastError, lastState to yk_1, ek_1,ek_2 etc.
 */
typedef struct PIDBase
{
	/**
	 * The proportional, integral, derivative,
	 * filter and tracking gain.
	 */
	float Kp, Ki, Kd, Tf, Kt, b;
	/**
	 * Automatic tracking flag.
	 */
	char autoWindup;
	///Backward re-calculation flag
	char useBackward;
	/**
	 * The windup flag
	 */
	char windup, extWindup;
	/**
	 * The maximum output limit. The output saturation is symmetric.
	 */
	float outputLimit;
	/**
	 * Internal state of the backward euler.
	 */
	float internalState, lastRef, lastError, lastPError, lastFF, lastState, llastError, llastState, lastDerivative, lastI, I;

	/**
	 * The reference, state, output, feedforward, tracking
	 */
	float desired, state, output, track;

	/**
	 * The internal model parameters.
	 */
	PT1Model model;
	/**
	 * Closed loop frequency.
	 */
	float w;
} PIDBase;

/**
 * Initialize the controller base. Set all the values to zero
 * and disable auto windup detection.
 */
void PIDBase_init(PIDBase* self);

/**
 * The helper function for output saturation.
 */
float sat(float u, float low, float high);
/* PIDBASE_H_ */
#endif
