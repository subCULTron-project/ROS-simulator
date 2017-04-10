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
#include <labust/simulation/RBModel.hpp>
#include <labust/math/NumberManipulation.hpp>

using namespace labust::simulation;

RBModel::RBModel():
    ae(0.15),be(0.2),ce(0.2),
    B(0),
    dT(0.1),
    waterLevel(0),
    nu0(vector::Zero()),
    eta0(vector::Zero()),
    nu(vector::Zero()),
    eta(vector::Zero()),
    g(vector::Zero()),
    isCoupled(false),
    current(vector3::Zero())
	{this->init();};

RBModel::~RBModel(){};

void RBModel::calculate_mrb()
{
	using namespace labust::math;
	Mrb.block<3,3>(0,0) = m*matrix3::Identity();
	Mrb.block<3,3>(0,3) = -m*skewSymm3(rg);
	Mrb.block<3,3>(3,0) = -m*skewSymm3(rg);
	Mrb.block<3,3>(3,3) = Io;
}

void RBModel::step(const vector& tau)
{
	//Assemble the linear and angluar velocity transformation matrices
	using namespace Eigen;
	matrix3 J1;
	J1 = AngleAxisd(eta(psi), Vector3d::UnitZ())
			* AngleAxisd(eta(theta), Vector3d::UnitY())
			* AngleAxisd(eta(phi), Vector3d::UnitX());
	double c1 = cos(eta(phi)), s1 = sin(eta(phi));
	double c2 = cos(eta(theta)), t2 = tan(eta(theta));
	if (!c2) c2 = 0.1;
	matrix3 J2;
	J2<<1,s1*t2,c1*t2,
			0,c1,-s1,
			0,s1/c2,c1/c2;
	//Calculate restoring forces
	restoring_force(J1);

	matrix M = Ma + Mrb;
	vector nu_old(nu);
	if (isCoupled)
	{
		//Assemble coriolis
		coriolis();
		//Calculate the absolute values of the speed
		matrix CD = Ca + Crb + Dlin + Dquad*nu.cwiseAbs2().asDiagonal();
		FullPivLU<matrix> decomp(M+dT*CD);
		if (decomp.isInvertible())
		{
			//Backward propagation model
			nu = decomp.inverse()*(M*nu + dT*(tau-g));
		}
		else
		{
			std::cerr<<"Mass matrix is singular."<<std::endl;
		}
	}
	else
	{
		//Simplification for uncoupled
		for (size_t i=0; i<nu.size(); ++i)
		{
			double beta = Dlin(i,i) + Dquad(i,i)*fabs(nu(i));
			//Backward propagation model
			nu(i) = (M(i,i)*nu(i) + dT*(tau(i)- g(i)))/(M(i,i) + dT*beta);
		};
	}
	nuacc = (nu - nu_old)/dT;

	//From body to world coordinates
	eta.block<3,1>(0,0) += dT*(J1*nu.block<3,1>(0,0)+current);
	eta.block<3,1>(3,0) += dT*J2*nu.block<3,1>(3,0);
}

void RBModel::coriolis()
{
	using namespace labust::math;
	//We assume that Mrb and Ma matrices are set
	vector3 nu1 = nu.block<3,1>(0,0);
	vector3 nu2 = nu.block<3,1>(3,0);

	Crb.block<3,3>(0,3) = -m*skewSymm3(nu1) - m*skewSymm3(nu2)*skewSymm3(rg);
	Crb.block<3,3>(3,0) = -m*skewSymm3(nu1) + m*skewSymm3(rg)*skewSymm3(nu2);
	Crb.block<3,3>(3,3) = -skewSymm3(Io*nu2);

	matrix3 M11=Ma.block<3,3>(0,0);
	matrix3 M12=Ma.block<3,3>(0,3);
	matrix3 M21=Ma.block<3,3>(3,0);
	matrix3 M22=Ma.block<3,3>(3,3);

	Ca.block<3,3>(0,3) = -skewSymm3(M11*nu1 + M12*nu2);
	Ca.block<3,3>(3,0) = -skewSymm3(M11*nu1 + M21*nu2);
	Ca.block<3,3>(3,3) = -skewSymm3(M21*nu1 + M22*nu2);
}

void RBModel::restoring_force(const matrix3& J1)
{
	matrix3 invJ1;
	invJ1=J1.inverse();

	//Update the lift force
	//Currently a simple model
	B=2*labust::math::coerce((eta(z)+waterLevel+rb(z)/2)/ce+1,0,2)*M_PI/3*ae*be*ce*rho*g_acc;

	vector3 fg = invJ1*(m*g_acc*vector3::UnitZ());
	vector3 fb = -invJ1*(B*vector3::UnitZ());

	g.block<3,1>(0,0) = -fg-fb;
	g.block<3,1>(3,0) = -skewSymm3(rg)*fg-skewSymm3(rb)*fb;
}
