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
 *  Created: 06.03.2013.
 *********************************************************************/
#ifndef THRUSTALLOCATION_HPP_
#define THRUSTALLOCATION_HPP_
#include <labust/simulation/matrixfwd.hpp>
#include <labust/math/NumberManipulation.hpp>

#include <vector>
#include <ros/ros.h>

namespace labust
{
	namespace vehicles
	{
		/**
		 * Performs thrust allocation based on configuration. Added support for thruster grouping
		 * to support multi-scaling allocation.
		 *
		 * \todo Add ability to register an external allocation algorithm when the need arises
		 * \todo Is it better to have a bunch of preprogramed simple allocation types that have static alllocation data ?
		 * \todo Split into a compilable cpp when more than one larger class exists for vehicle support
		 * \todo Remove the ROS_ERROR debug messages and any ROS dependencies
		 * \todo Add the windup indication for groups of scaled allocations
		 */
		class ThrustAllocator
		{
			enum {NoAlloc=0,SimpleAlloc=1, ScaleAlloc=2};
		public:
			/**
			 * Main constructor.
			 */
			ThrustAllocator():
				dofs(6,1),
				groups(1,dofs),
				B(labust::simulation::matrix::Identity()),
				Binv(B),
				tmax(1),
				tmin(-tmax),
				coercion(Eigen::VectorXi::Zero(6)),
				type(NoAlloc),
				num_thrusters(6){};

			/**
			 * Performs allocation and necessary conversions.
			 */
			void allocate(const labust::simulation::vector& tauIn,
					labust::simulation::vector& tauOut)
			{
				Eigen::VectorXd vi(dofs.size());
				for (int i=0; i<dofs.size(); ++i) vi(i)=tauIn(dofs[i]);

				tdes = Binv*vi;

				//ROS_ERROR("Before tdes: %f %f %f %f",tdes(0), tdes(1), tdes(2), tdes(3));

				switch (type)
				{
				case NoAlloc:
					//noalloc
					break;
				case SimpleAlloc:
					simple_alloc();
					break;
				case ScaleAlloc:
					scale_alloc();
					break;
				default:
					throw std::runtime_error("ThrustAllocator: Undefined allocation type.");
				}

				//ROS_ERROR("Forces %f %f %f",tdes(0), tdes(1), tdes(2), tdes(3));
				vi = B*tdes;

				for (int i=0; i<dofs.size(); ++i) tauOut(dofs[i])=vi(i);

				bool scaling = false;

				//Determine coercion for windup
				switch (type)
				{
				case NoAlloc:
					//noalloc
					break;
				case SimpleAlloc:
					//It is safe to compare these
					for (int i=0; i<dofs.size(); ++i)
					{
						coercion(dofs[i]) = 0;
						if (tauIn(dofs[i])>tauOut(dofs[i]))	coercion(dofs[i])=1;
						else if (tauIn(dofs[i])<tauOut(dofs[i]))	coercion(dofs[i])=-1;
					}
					break;
				case ScaleAlloc:
//						for (int i=0; i<group_scales.size(); ++i) if ((scaling = (group_scales[i]>1))) break;
//
//						if (scaling)
//						{
//							for (int i=0; i<dofs.size(); ++i)
//							{
//								if (tauIn(dofs[i])>=tauOut(dofs[i]))	coercion(dofs[i])=1;
//								else if (tauIn(dofs[i])<=tauOut(dofs[i]))	coercion(dofs[i])=-1;
//							}
//						}

					//Alternative to scaling inspection
					for (int i=0; i<dofs.size(); ++i)
					{
						coercion(dofs[i]) = 0;
						//If there is deviation between input and output we assume
						if (fabs(tauIn(dofs[i]) - tauOut(dofs[i])) > 0.01)
						{
							if (tauIn(dofs[i])>=tauOut(dofs[i]))	coercion(dofs[i])=1;
							else if (tauIn(dofs[i])<=tauOut(dofs[i]))	coercion(dofs[i])=-1;
						}
					}
					break;
				default:
					break;
				}
			}


			template <class AllocMatrix, class DOFMatrix, class ThrusterGroups>
			void init(int alloc_type, const AllocMatrix& alloc, const DOFMatrix& use_dofs,
					const ThrusterGroups& thrust_groups)
			{
				type = alloc_type;
				if (type >= 0)
				{
					dofs.clear();
					for (size_t i=0; i<use_dofs.size(); ++i) if (use_dofs(i)) dofs.push_back(i);
					groups.clear();
					num_thrusters=0;
					//Handle the optional group parameter
					if ((thrust_groups.cols() == 0) || (thrust_groups.rows() == 0))
					{
						//If the table was not specified put all in one group
						groups.push_back(std::vector<int>());
						for(int i=0; i<alloc.cols(); ++i) groups[0].push_back(i);
						num_thrusters=alloc.cols();
					}
					else
					{
						//If the table is specified fill the vectors
						for (size_t i=0; i<thrust_groups.rows();++i)
						{
							//For each new row create a new group
							groups.push_back(std::vector<int>());
							//If thruster in group add it to the list
							for (size_t j=0; j<thrust_groups.cols(); ++j)
								if (thrust_groups(i,j))
								{
									++num_thrusters;
									groups[i].push_back(j);
								}
						}
					}
					group_scales.resize(groups.size());

					this->B=alloc;
					Binv = B.transpose()*(B*B.transpose()).inverse();
				}
				else
				{
					type = NoAlloc;
				};

				assert((B.cols() == num_thrusters) &&
						"Allocation matrix must have number of columns equal to number of thrusters.");

				assert((B.rows() == dofs.size()) &&
						"Allocation matrix must have the same number of rows as controllable DOFs.");
			}

			/**
			 * The method sets the thruster maximum and minimum force.
			 */
			inline void setThrusterMinMax(double min, double max)
			{
				this->tmax = max;
				this->tmin = min;
			}

			inline const Eigen::VectorXd& getThrusterForces(){return tdes;};

			inline const Eigen::VectorXi& getCoercion(){return coercion;};

		protected:
			/**
			 * Do simple allocation.
			 */
			inline void simple_alloc()
			{
				for (size_t i=0; i<tdes.rows();++i)
				{
					tdes(i)=labust::math::coerce(tdes(i),tmin,tmax);
				}
			};

			/**
			 * Do scaled allocation.
			 */
			void scale_alloc()
			{
				for (size_t i=0; i<groups.size(); ++i)
				{
					//ROS_ERROR("Group size %d",groups[i].size());
					Eigen::VectorXd ttdes(groups[i].size());
					//Map from vector into group
					for (size_t j=0; j<groups[i].size(); ++j) ttdes(j)=tdes(groups[i][j]);

					//ROS_ERROR("Before: %f %f %f %f",ttdes(0), ttdes(1), ttdes(2), ttdes(3));
					//Scale inside the group
					double scale_max = 1;
					for (size_t j=0; j<ttdes.rows();++j)
					{
						double scale = fabs((ttdes(j)>0)?ttdes(j)/tmax:ttdes(j)/tmin);
						if (scale>scale_max) scale_max=scale;
					}
					//ROS_ERROR("Scale max: %f",scale_max);
					group_scales[i] = scale_max;
					ttdes = ttdes/scale_max;
					//ROS_ERROR("After: %f %f %f %f",ttdes(0), ttdes(1), ttdes(2), ttdes(3));

					//Map from group to final vector
					for (size_t j=0; j<groups[i].size(); ++j) tdes(groups[i][j])=ttdes(j);
				}
			}

			/**
			 * Desired dofs.
			 */
			std::vector<int> dofs;
			/**
			 * Thruster groups.
			 */
			std::vector< std::vector<int> > groups;
			/**
			 * The scales per group.
			 */
			std::vector<double> group_scales;

			/**
			 * The allocation matrix.
			 */
			Eigen::MatrixXd B, Binv;
			/**
			 * The maximum and minimum thrust.
			 */
			double tmax, tmin;
			/**
			 * The allocated thrusters.
			 */
			Eigen::VectorXd tdes;
			/**
			 * The coercion vector.
			 */
			Eigen::VectorXi coercion;
			/**
			 * Allocation type, number of thrusters.
			 */
			int type, num_thrusters;
		};
	}
}

/* ALLOCATIONMODELS_HPP_ */
#endif
