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
#ifndef MATRIXLOADER_HPP_
#define MATRIXLOADER_HPP_
#include <ros/ros.h>
#include <Eigen/Dense>

#include <string>

namespace labust
{
	namespace tools
	{
		template <class Derived, class XmlRpcVal>
		inline typename Eigen::MatrixBase<Derived>::Scalar	xmlRpcConvert(XmlRpcVal& data, bool intType)
		{
			return static_cast< typename Eigen::MatrixBase<Derived>::Scalar >(
					intType?static_cast<int>(data):static_cast<double>(data));
		}

		/**
		 * Reads a vector or matrix list and populates the supplied vector or matrix.
		 */
		template <class Derived>
		std::pair<int,int>	getMatrixParam_fixedsize(const ros::NodeHandle& nh,
				const std::string& name,
				const Eigen::MatrixBase<Derived>& matrix)
		{
			if (!nh.hasParam(name))
			{
				ROS_WARN("Configuration parameter %s not found.", name.c_str());
				return std::make_pair(-1,-1);
			}
			else
			{
				ROS_INFO("Found configuration parameter %s.", name.c_str());
			}

			XmlRpc::XmlRpcValue data;
			nh.getParam(name, data);
			ROS_ASSERT(data.getType() == XmlRpc::XmlRpcValue::TypeArray);

			size_t col(0),row(0);
			for(size_t i=0; i<data.size(); ++i)
			{
				//Check validity
				bool doubleType = data[i].getType() == XmlRpc::XmlRpcValue::TypeDouble;
				bool intType = data[i].getType() == XmlRpc::XmlRpcValue::TypeInt;
				bool vectorType = data[i].getType() == XmlRpc::XmlRpcValue::TypeArray;
				ROS_ASSERT(doubleType || intType || vectorType);

				if (doubleType || intType)
				{
					const_cast< Eigen::MatrixBase<Derived>& >
					(matrix)(row,col++) = xmlRpcConvert<Derived>(data[i], intType);

					if (col>=matrix.cols())
					{
						++row;
						col = col%matrix.cols();
					}
				}
				else
				{
					row = i;
					col = 0;
					for(size_t j=0; j<data[i].size(); ++j)
					{
						//ROS_DEBUG("Access element %d %d",i,j);
						bool intType = data[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt;
						const_cast< Eigen::MatrixBase<Derived>& >
						(matrix)(row,col++) = xmlRpcConvert<Derived>(data[i][j], intType);
					}
				}
			}

			return std::make_pair(row,col);
		}

		/**
		 * Reads a vector or matrix list and populates the supplied vector or matrix.
		 */
		template <class Derived>
		std::pair<int,int>	getMatrixParam_variablesize(const ros::NodeHandle& nh,
				const std::string& name,
				const Eigen::MatrixBase<Derived>& matrix)
		{
			if (!nh.hasParam(name))
			{
				ROS_WARN("Configuration parameter %s not found.", name.c_str());
				return std::make_pair(-1,-1);
			}
			else
			{
				ROS_INFO("Found configuration parameter %s.", name.c_str());
			}

			XmlRpc::XmlRpcValue data;
			nh.getParam(name, data);
			ROS_ASSERT(data.getType() == XmlRpc::XmlRpcValue::TypeArray);

			std::vector<typename Derived::Scalar> temp;

			size_t col(0),row(0);
			bool expandDiagonal(false);
			for(size_t i=0; i<data.size(); ++i)
			{
				if (i==0 && (data[i].getType() == XmlRpc::XmlRpcValue::TypeString))
				{
					std::string matrix_type(static_cast<std::string>(data[i]));
					std::cout<<"Matrix type: "<<matrix_type<<std::endl;
					expandDiagonal = (matrix_type == "diagonal");
					continue;
				}
				//Check validity
				bool doubleType = data[i].getType() == XmlRpc::XmlRpcValue::TypeDouble;
				bool intType = data[i].getType() == XmlRpc::XmlRpcValue::TypeInt;
				bool vectorType = data[i].getType() == XmlRpc::XmlRpcValue::TypeArray;
				ROS_ASSERT(doubleType || intType || vectorType);

				if (doubleType || intType)
				{
					temp.push_back(xmlRpcConvert<Derived>(data[i], intType));
					col++;
				}
				else
				{
					row = i;
					col = 0;
					for(size_t j=0; j<data[i].size(); ++j)
					{
						//ROS_DEBUG("Access element %d %d",i,j);
						bool intType = data[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt;
						temp.push_back(xmlRpcConvert<Derived>(data[i][j], intType));
						col++;
					}
				}
			}


			if (expandDiagonal)
			{
				Eigen::Map< Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> > vector(&temp[0],col);
				Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> max(vector.asDiagonal());
			  const_cast< Eigen::MatrixBase<Derived>& >(matrix) = max;
			}
			else
			{
				const_cast< Eigen::MatrixBase<Derived>& >(matrix) = matrix.derived().Map(&temp[0],col,row+1).transpose();
			}

			return std::make_pair(row,col);
		}
		
		/**
		 * Reads a vector or matrix list and populates the supplied vector or matrix.
		 *
		 * \todo Derive one function for variable and fixed size.
		 */
		template <class Derived>
		inline std::pair<int,int>	getMatrixParam(const ros::NodeHandle& nh,
				const std::string& name,
				const Eigen::MatrixBase<Derived>& matrix)
		{
			if ((matrix.RowsAtCompileTime <= 0) ||
					(matrix.ColsAtCompileTime <=0))
			{
				return getMatrixParam_variablesize(nh,name,matrix);
			}
			else
			{
				return getMatrixParam_fixedsize(nh,name,matrix);
			}
		}
	}
}

/* MATRIXLOADER_HPP_ */
#endif
