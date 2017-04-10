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
#ifndef LIMITS_HPP_
#define LIMITS_HPP_

#include <limits>

namespace labust
{
  namespace math
  {
    /**
     * The class implements a simple limit class. The class is similar to the
     * std::pair however it instantiates the member variables to NaN values so
     * that they are comparison neutral if not defined.
     *
     * \tparam T The precision template variable.
     */
    template <class T = double>
    struct Limit
    {
    	/**
    	 * Default constructor. Instantiates member variables to the NaN.
    	 */
      Limit():
        min(std::numeric_limits<T>::quiet_NaN()),
        max(std::numeric_limits<T>::quiet_NaN()){};
      /**
       * Main constructor. Take the minimum and maximum limits.
       *
       * \param min Minimum of the limited range.
       * \param max Maximum of the limited range.
       */
      Limit(T min, T max):
        min(min),
        max(max){};

      /**
       * Minimum and maximum of the limit range.
       */
      T min,max;
    };

    /**
     * The class implements input/output limits. It contains separate labust::math::Limit values
     * for block input and output. To be used with black-box classes that have IO constraints.
     *
     * \tparam T The precision template variable.
     */
    template <class T= double>
    struct LimitIO
    {
    	/**
    	 * Default constructor. Instantiates all limits to NaN.
    	 */
      LimitIO():
        in(),
        out(){};
      /**
       * Main constructor. Takes two single limits for the block input and output.
       *
       * \param in Input range limit.
       * \param out Output range limit.
       */
      LimitIO(const Limit<T>& in, const Limit<T>& out):
        in(in.min,in.max),
        out(out.min,out.max){};
      /**
       * Main constructor. Takes all the range constraints separately.
       *
       * \param minIn Minimum of the input range.
       * \param maxIn Maximum of the input range.
       * \param minOut Minimum of the output range.
       * \param maxOut Maximum of the output range.
       */
      LimitIO(T minIn, T maxIn, T minOut, T maxOut):
        in(minIn,maxIn),
        out(minOut,maxOut){};

      /**
       * Input and output range limiters.
       */
      Limit<T> in,out;
    };
  }
}
/* LIMITS_HPP_ */
#endif
