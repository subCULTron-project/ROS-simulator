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
#ifndef DYNAMICSPARAM_HPP_
#define DYNAMICSPARAM_HPP_
#include <labust/simulation/matrixfwd.hpp>

namespace labust
{
	namespace simulation
	{
		struct DynamicsParams
		{
			/**
			 * The basic initialization constructor.
			 */
			DynamicsParams():
				Io(matrix3::Identity()),
				Mrb(matrix::Identity()),
				Ma(matrix::Identity()),
				Crb(matrix::Zero()),
				Ca(matrix::Zero()),
				Dlin(matrix::Identity()),
				Dquad(matrix::Zero()),
				rg(vector3::Zero()),
				rb(vector3::Zero()),
				m(1),
				g_acc(9.81),
				rho(1000){};
			/**
			 * Inertia matrix of the model.
			 */
			matrix3 Io;
			/**
			 * Rigid-body mass and added mass matrices.
			 */
			matrix Mrb,Ma;
			/**
			 * The coriolis and centripetal matrices of the model and the added mass.
			 */
			matrix Crb,Ca;
			/**
			 * Linear and quadratic parts of the damping matrix.
			 */
			matrix Dlin,Dquad;
			/**
			 * Center of gravity and buoyancy.
			 */
			vector3 rg,rb;
			/**
			 * Model mass and acceleration.
			 */
			double m, g_acc;
			/**
			 * Fluid density.
			 */
			double rho;
		};
	}
}

/* ROSUTILS_HPP_ */
#endif
