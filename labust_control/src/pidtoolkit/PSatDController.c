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
#include <labust/control/PSatDController.h>
#include <math.h>

void PSatD_tune(PIDBase* self, float w, float aAngle, float surge)
{
	//Check for zero surge
	self->Kp = self->Kd = 0;
	if (surge != 0.0)
	{
		self->Kp = w*w/surge;
		self->Kd = 2*w/surge;
	}
	self->outputLimit = self->Kd*surge*sin(aAngle);
	self->Ki = self->Kt = self->Tf = 0;
}

void PSatD_dStep(PIDBase* self, float Ts, float derivative)
{
	//Proportional term
	self->internalState = sat(self->Kp*(self->desired - self->state),
			-self->outputLimit, self->outputLimit);
	//Derivative term
  self->internalState += self->Kd*derivative;
	//Set final output
	self->output = self->internalState;

	self->lastError = (self->desired - self->state);
}

void PSatD_dIdle(PIDBase* self, float Ts, float derivative)
{
	self->lastError = (self->desired - self->state);
}


