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
#include <labust/control/PIDFFController.h>
#include <math.h>

void PIDFF_modelTune(PIDBase* self,
		const PT1Model* const model,
		float w)
{
	self->Kp = model->alpha*w*w;
	self->Ki = model->alpha/(w*w*w);
	self->Kd = model->alpha*w - model->beta;
	self->Kt = self->Tf = 0;

	self->model.alpha = model->alpha;
	self->model.beta = model->beta;
	self->model.betaa = model->betaa;
}

void PIDFF_tune(PIDBase* self, float w)
{
	self->Kp = 2*w;
	self->Ki = w*w;
	self->Kd = self->Kt = self->Tf = 0;
}

void PIDFF_wffStep(PIDBase* self, float Ts, float error, float ff)
{
	//Perform windup test if automatic mode is enabled.
	if (self->autoWindup == 1)
	{
		self->windup = (self->internalState > self->output) && (error>0);
		self->windup = (self->windup) ||
				((self->internalState < self->output) && (error<0));
	}
	else
	{
		//Experimental
		//self->windup = ((self->windup >0) && (error>0)) ||
		//		((self->windup <0) && (error<0));
	}

	//Proportional term
	self->internalState += self->Kp*(error-self->lastError);
	//Integral term
	//Disabled if windup is in progress.
  if (!self->windup) self->internalState += self->Ki*Ts*error;
  //Derivative
  self->internalState += self->Kd*1/Ts*(error-2*self->lastError+self->llastError);
	//Feed forward term
	self->internalState += ff - self->lastFF;
	//Set final output
	self->output = self->internalState;

	if (self->autoWindup == 1)
	{
		self->output = sat(self->output,-self->outputLimit, self->outputLimit);
	}

	self->llastError = self->lastError;
	self->lastError = error;
	self->lastRef = self->desired;
	self->lastFF = ff;
}

void PIDFF_dwffStep(PIDBase* self, float Ts, float error, float ff, float ds)
{
	//Perform windup test if automatic mode is enabled.
	if (self->autoWindup == 1)
	{
		self->windup = (self->internalState > self->output) && (error>0);
		self->windup = (self->windup) ||
				((self->internalState < self->output) && (error<0));
	}
	else
	{
		//Experimental
		//self->windup = ((self->windup >0) && (error>0)) ||
		//		((self->windup <0) && (error<0));
	}

	//Proportional term
	self->internalState += self->Kp*(error-self->lastError);
	//Integral term
	//Disabled if windup is in progress.
  if (!self->windup) self->internalState += self->Ki*Ts*error;
  //Derivative
  self->internalState += self->Kd*(self->lastDerivative - ds);
	//Feed forward term
	self->internalState += ff - self->lastFF;
	//Set final output
	self->output = self->internalState;

	if (self->autoWindup == 1)
	{
		self->output = sat(self->output,-self->outputLimit, self->outputLimit);
	}

	self->lastDerivative = ds;
	self->llastError = self->lastError;
	self->lastError = error;
	self->lastRef = self->desired;
	self->lastFF = ff;
}



