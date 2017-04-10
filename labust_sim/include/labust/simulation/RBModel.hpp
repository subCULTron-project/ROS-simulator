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
#ifndef RBMODEL_HPP_
#define RBMODEL_HPP_
#include <labust/simulation/matrixfwd.hpp>
#include <labust/simulation/DynamicsParams.hpp>
#include <labust/simulation/NoiseModel.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/vehicles/ThrustAllocation.hpp>

namespace labust
{
  namespace simulation
  {
  	/**
     *  This class implements a 6DOF marine vehicle model. This model is based on
     *  the equations of motion derived in chapter 2 of Guidance and control of
     *  Ocean Vehicles by Fossen (1994).
     *
     *  \todo Add allocation type and thruster modeling
     */
    class RBModel : public labust::simulation::DynamicsParams
    {
    public:
      enum {x=0,y,z,phi,theta,psi};
      enum {u=0,v,w,p,q,r};
      enum {X=0,Y,Z,K,M,N};
      /**
       * Default constructor for common use without configuration.
       */
      RBModel();
      /**
       * Generic destructor.
       */
      ~RBModel();

      /**
       * The method performs one simulation step. The model is propagated in time for one sampling period.
       * To access states after the propagation use the accessor methods.
       *
       * \param tau Vector of forces and moments acting on the model.
       */
      void step(const vector& tau);

      /**
       * The method the additional allocation step and generates a new tau.
       */
      inline void alloc_step(const vector& tauIn, vector& tauAch)
      {
      	this->allocator.allocate(tauIn,tauAch);
      	this->step(tauAch);
      }

      /**
       * Method to get the current model states. This vector returns the model position and orientation
       * in world coordinates.
       *
       * \return Constant reference to the model states.
       */
      inline const vector& Eta() const {return this->eta;};
      /**
       * Method to get the current model states. This vector returns the model linear and rotational speeds in
       * the body-fixed coordinate frame.
       *
       * \return Constant reference to the model states.
       */
      inline const vector& Nu() const {return this->nu;};
      /**
       * Method to get the current model acceleration. This vector returns the model linear and rotational accelerations in
       * the body-fixed coordinate frame.
       *
       * \return Constant reference to the model states.
       */
      inline const vector& NuAcc() const {return this->nuacc;};
      /**
       * Method to get the current model states. This vector returns the model position and orientation
       * in world coordinates.
       *
       * \return Constant reference to the model states.
       */
      inline const vector& EtaNoisy() const {return this->etaN = this->eta + this->noise.calculateW();};
      /**
       * Method to get the current model states. This vector returns the model linear and rotational speeds in
       * the body-fixed coordinate frame.
       *
       * \return Constant reference to the model states.
       */
      inline const vector& NuNoisy() const {return this->nuN = this->nu + this->noise.calculateV();};

      /**
       * The method sets the initial states of the model.
       *
       * \param nu Linear and rotational velocities in the body-fixed coordinate frame.
       * \param eta Position and orientation in the earth-fixed coordinate frame.
       */
      inline void setNuAndEta(const vector& nu,const vector& eta)
      {
      	this->nu = this->nu0 = nu;
      	this->eta = this->eta0 = eta;
      }
      /**
       * Returns the pressure based on depth.
       */
      inline double getPressure(double h) const{return this->rho*this->g_acc*h;};

      /**
       * Initialize the model.
       */
      inline void init()
      {
    	this->reset();
      	calculate_mrb();
      	//Calculate and set the the neutral depth where W=B
        eta(z) = (m/(2*ae*be*ce*M_PI/3*rho)-1)*ce - waterLevel - rb(z);
      }
      /**
       * The method restarts the model to initial parameters.
       */
      void reset()
      {
        this->eta = this->eta0;
        this->nu = this->nu0;
        this->B=2*labust::math::coerce((eta(z)+waterLevel+rb(z)/2)/ce+1,0,2)*M_PI/3*ae*be*ce*rho*g_acc;
      };

      /**
       * The sampling step.
       */
      double dT;
      /**
       * Coupled dynamics flag.
       */
      bool isCoupled;
      /**
       * The bounding ellipsoid parameters.
       */
      double ae,be,ce;
      /**
       * The current water-level for simple wave simulation.
       */
      double waterLevel;
      /**
       * The external current disturbance vector.
       */
      vector3 current;

      /**
       * The initial speeds and position vector.
       */
      vector nu0, eta0;
      /**
       * The noise generator.
       */
      mutable labust::simulation::NoiseModel noise;

      /**
       * The thrust allocator.
       */
      labust::vehicles::ThrustAllocator allocator;

    protected:
      /**
       * The method calculates the Coriolis matrix of the model.
       */
      void coriolis();
      /**
       * The method calculates the restoring forces.
       */
      void restoring_force(const matrix3& J1);
      /**
       * The method calculates the rigid-body mass matrix of the model.
       */
      void calculate_mrb();

      /**
       * The calculated buoyancy.
       */
      double B;
      /**
       * Linear and orientation velocity and their initial state.
       */
      vector nu,nuacc;
      /**
       * Position and orientation and their initial state.
       */
      vector eta;
      /**
       * The noisy measurements.
       */
      mutable vector etaN, nuN;
      /**
       * The restoring forces vector.
       */
      vector g;
    };

    /**
     * The function computes the skew-symmetric matrix from a given vector.
     * The function operates with vector size 3.
     *
     * \param vec The desired vector of size 3.
     *
     * \tparam in The input matrix type.
     *
     * \return The calculated 3x3 skew symmetric matrix.
     */
    template<class Derived>
    matrix3 skewSymm3(const Eigen::MatrixBase<Derived>& vec)
    {
      matrix3 mat;
      mat<<0,-vec(2),vec(1),
      		vec(2),0,-vec(0),
      		-vec(1),vec(0),0;
      return mat;
    };
  }
}

/* RBMODEL_HPP_ */
#endif
