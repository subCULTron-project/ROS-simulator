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
 *  Created: 01.02.2013.
 *********************************************************************/
#ifndef DYNAMICSLOADER_HPP_
#define DYNAMICSLOADER_HPP_
#include <labust/tools/MatrixLoader.hpp>
#include <ros/ros.h>

namespace labust
{
	namespace tools
	{
		/**
		 * The extender expands a diagonal vector representation of the matrix
		 * into a full rank diagonal matrix.
		 */
		template <class Derived>
		inline void matrixExtender(const ros::NodeHandle& nh,
				const std::string& name, Eigen::MatrixBase<Derived>& matrix)
		{
			std::pair<int,int> rowcols = getMatrixParam(nh,name,matrix);
			if ((matrix.rows() > rowcols.first) && (rowcols.first == 1))
			{
				matrix=matrix.row(0).eval().asDiagonal();
			}
		}

		/**
		 * The function loads and configures the dynamics parameters from the
		 * supplied ROS node handle.
		 */
		template <class Model>
		void loadDynamicsParams(const ros::NodeHandle& nh,
				Model& model)
		{
			nh.param("mass",model.m, model.m);
			nh.param("gravity",model.g_acc, model.g_acc);
			nh.param("fluid_density",model.rho, model.rho);

			getMatrixParam(nh,"rg",model.rg);
			getMatrixParam(nh,"rb",model.rb);

			matrixExtender(nh,"added_mass",model.Ma);
			matrixExtender(nh,"linear_damping",model.Dlin);
			matrixExtender(nh,"quadratic_damping",model.Dquad);
			matrixExtender(nh,"inertia_matrix",model.Io);
		}
	}
}
/* DYNAMICSLOADER_HPP_ */
#endif
