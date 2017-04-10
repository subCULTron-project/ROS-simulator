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
 *
 *  Author: Dula Nad
 *  Created: 01.02.2010.
 *********************************************************************/
#include <labust/simulation/kinematic_model.h>
#include <labust/math/NumberManipulation.hpp>

using namespace labust::simulation;

KinematicModel::KinematicModel():
    dT(0.1),
    pos0(vector3::Zero()),
    quat0(1,0,0,0),
	quat(quat0),
    pos(pos0),
    current(vector3::Zero())
{
	this->init();
};

void KinematicModel::step(const vector& nu)
{
	//Assemble the linear and angluar velocity transformation matrices
	using namespace Eigen;
	pos += dT*(quat.toRotationMatrix()*nu.block<3,1>(0,0) + current);

	Eigen::Matrix<double,4,3> angMat;
	angMat<<quat.w(),-quat.z(),quat.y(),
			quat.z(),-quat.w(),-quat.x(),
			-quat.y(),quat.x(),quat.w(),
			-quat.x(),-quat.y(),-quat.z();
	angMat = 0.5*angMat;
	Eigen::Matrix<double,4,1> dq=dT*angMat*nu.block<3,1>(3,0);
	quat.coeffs() += Eigen::Quaternion<double>(dq(3), dq(0),dq(1),dq(2)).coeffs();
	quat.normalize();
}
