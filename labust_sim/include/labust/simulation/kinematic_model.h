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
#ifndef LABUST_SIM_KINEMATICMODEL_H
#define LABUST_SIM_KINEMATICMODEL_H
#include <labust/simulation/matrixfwd.hpp>
#include <labust/tools/conversions.hpp>

namespace labust
{
  namespace simulation
  {
  	/**
     *  This class implements the kinematics of a 6DOF rigid-body. This model is based on
     *  the equations of motion derived in chapter 2 of Guidance and control of
     *  Ocean Vehicles by Fossen (1994).
     */
    class KinematicModel
    {
    public:
      enum {x=0,y,z};
      enum {phi=0,theta,psi};
      enum {u=0,v,w,p,q,r};
      ///Default constructor for common use without configuration.
      KinematicModel();
      /**
       * The method performs one simulation step. The model is propagated in time for one sampling period.
       * To access states after the propagation use the accessor methods.
       *
       * \param nu Vector of linear and angular speeds.
       */
      void step(const vector& nu);
      /**
       * Method to get the current position. This vector returns the model position
       * in world coordinates.
       *
       * \return Constant reference to the model states.
       */
      inline const vector3& position() const {return this->pos;};
      /**
       * Method to get the current model states. This vector returns the model orientation in
       * Euler ZYX in world coordinates.
       *
       * \return Constant reference to the model states.
       */
      inline const vector3& orientation()
      {
    	  labust::tools::eulerZYXFromQuaternion(this->quat, this->rpy);
    	  return rpy;
      };
      /**
       * The method sets the initial states of the model.
       *
       * \param pos Position in the earth-fixed coordinate frame.
       * \param rpy Euler ZYX orientation in the earth-fixed coordinate frame.
       */
      inline void setPosition(const vector3& pos, const vector3& rpy)
      {
      	this->pos = this->pos0 = pos;
      	labust::tools::quaternionFromEulerZYX(rpy(phi),rpy(theta),rpy(psi), this->quat);
      }
      /**
       * The method sets the initial states of the model.
       *
       * \param pos Position in the earth-fixed coordinate frame.
       * \param quat Quaternion orientation in the earth-fixed coordinate frame.
       */
      inline void setPosition(const vector3& pos, const quaternion& quat)
      {
      	this->pos = this->pos0 = pos;
      	this->quat = this->quat0 = quat;
      }
      /**
       * Initialize the model.
       *
       * \param depth Defaults to zero but can be used to set a neutral depth.
       */
      inline void init(double depth = 0)
      {
    	this->reset();
      	//Calculate and set the the neutral depth where W=B
        pos(z) = depth;
      }
      /**
       * The method restarts the model to initial parameters.
       */
      void reset()
      {
        this->pos = this->pos0;
        this->quat = this->quat0;
      };
      ///Set the sampling time in seconds.
      inline void setTs(double Ts){this->dT = Ts;};

    protected:
      ///The model sampling step.
      double dT;
      ///The external current disturbance in world coordinates.
      vector3 current;
      ///The initial position vector.
      vector3 pos0;
      ///Position and orientation state.
      vector3 pos;
      ///The initial quaternion vector
      quaternion quat0;
      ///The orientation quaternion
      quaternion quat;
      ///The internal rpy representation
      vector3 rpy;
    };
  }
}

/* LABUST_SIM_KINEMATICMODEL_H */
#endif
