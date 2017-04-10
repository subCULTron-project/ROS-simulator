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
#ifndef NUMBERMANIPULATION_HPP_
#define NUMBERMANIPULATION_HPP_

#define _USE_MATH_DEFINES
#include <cmath>
#include <labust/math/Limits.hpp>

namespace labust
{
  namespace math
  {
    //const double M_2PI = 2*M_PI;
    /**
     * The function wraps any value into the [-pi,pi> range.
     *
     * \param angle The arbitrary value.
     * \return The wrapped value in the [-pi,pi> interval.
     */
    inline double wrapRad(double angle)
    {
      angle = fmod(angle,2*M_PI);
      if (angle > M_PI) return -2*M_PI + angle;
      if (angle <= -M_PI) return 2*M_PI + angle;
      return angle;
    }
    /**
     * The function wraps any value into the [-180,180> range.
     *
     * \param angle The arbitrary value.
     * \return The wrapped value in the [-pi,pi> interval.
     */
    inline double wrapDeg(double angle)
    {
      angle = fmod(angle,360.);
      if (angle > 180) return -360. + angle;
      if (angle <= -180) return 360. + angle;
      return angle;
    }
    /**
     * The function coerces a value into the [min,max] range based on the labust::math::Limit parameter.
     *
     * \param value The arbitrary value.
     * \param limit The range limit of the coerced value.
     * \return The coerced value.
     */
    template <class type, typename precission>
    inline type coerce(type value, const Limit<precission>& limit)
    {
      if (value>limit.max) return limit.max;
      if (value<limit.min) return limit.min;
      return value;
    }
    /**
     * The function coerces a value into the [min,max] range.
     *
     * \param value The arbitrary value.
     * \param min The coercion range minimum.
     * \param max The coercion range maximum.
     * \return The coerced value.
     */
    template<class type>
    inline type coerce(type value, double min, double max)
    {
      if (value>max) return max;
      if (value<min) return min;
      return value;
    }
    /**
     * The function calculates the vector mean over a range.
     *
     * \param first Iterator to the first element.
     * \param last Iterator to the last element.
     *
     * \tparam Template used for polymorphism.
     */
    template <class Iterator>
    inline double mean(const Iterator& first, const Iterator& last)
    {
    	double sum(0);
    	unsigned int size(0);
    	for (Iterator cnt = first; cnt!=last; ++cnt,++size) sum += (*cnt);
    	return (size)?(sum/size):0;
    }
    /**
     * The function calculates the vector mean over a range.
     *
     * \param vec Data vector.
     *
     * \tparam Template used for polymorphism.
     */
    template <class Vector>
    inline double mean(const Vector& vec)
    {
    	double sum(0);
    	unsigned int size(0);
    	for (typename Vector::const_iterator cnt = vec.begin(); cnt!=vec.end(); ++cnt,++size) sum += (*cnt);
    	return (size)?(sum/size):0;
    }
    /**
     * The function calculates the vector deviation over a range.
     *
     * \param vec Data vector.
     *
     * \tparam Template used for polymorphism.
     */
    template<class Vector>
    inline double std2(const Vector& vec)
    {
      double sum(0),mean(labust::math::mean(vec));
      unsigned int size(0);
      for(typename Vector::const_iterator cnt = vec.begin(); cnt != vec.end(); ++cnt, ++size) sum+=std::pow(((*cnt) - mean),2);
      return size?std::sqrt(sum/size):-1;
    }

    /**
     * The function calculates the vector deviation over a range.
     *
     * \param vec Data vector.
     *
     * \param mean Mean value of the same data
     *
     * \tparam Template used for polymorphism.
     */
    template<class Vector>
    inline double std2(const Vector& vec, double mean)
    {
      double sum(0);
      unsigned int size(0);
      for(typename Vector::const_iterator cnt = vec.begin(); cnt != vec.end(); ++cnt, ++size) sum+=std::pow(((*cnt) - mean),2);
      return size?std::sqrt(sum/size):-1;
    }  
      
    /**
     * The structure handles angle unwrapping. We use a structure for re-entrant behaviour.
     * When doing estimation or control use the angle unwrapper to condition the vehicle angle.
     * We recommend putting this in vehicle driver to supply a unwrapped vehicle yaw.
     */
    struct unwrap
    {
    	/**
    	 * Generic constructor.
    	 */
    	unwrap():anglek_1(0),index(0){};
    	/**
    	 * Based on the current angle and history calculates the unwrapped angle value.
    	 * Useful for tracking the amount of full circles the vehicle has taken.
    	 *
    	 * \param angle Angle in radians.
    	 */
    	double operator()(double angle)
    	{
    		double u = angle - this->anglek_1;
    		this->anglek_1=angle;

    		if (u<=-M_PI)
    		{
    			++index;
    		}
    		else if (u>=M_PI) --index;

    		return this->index*2*M_PI + angle;
    	}

    private:
    	/**
    	 * The last angle.
    	 */
    	double anglek_1;
    	/**
    	 * The wrap-around index.
    	 */
    	int index;
    };
  }
}
/* NUMBERMANIPULATION_HPP_ */
#endif
