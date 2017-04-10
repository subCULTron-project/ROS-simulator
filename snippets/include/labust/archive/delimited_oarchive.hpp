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
 *  Created on: 15.11.2013.
 *  Author: Dula Nad
 *********************************************************************/
#ifndef DELIMITED_OSERIALIZATOR_HPP_
#define DELIMITED_OSERIALIZATOR_HPP_
#include <cstddef>
#include <boost/archive/detail/common_oarchive.hpp>

namespace labust
{
	namespace archive
	{
		class delimited_oarchive :
		    public boost::archive::detail::common_oarchive<delimited_oarchive>
		{
		    // permit serialization system privileged access to permit
		    // implementation of inline templates for maximum speed.
		    friend class boost::archive::save_access;

		    // member template for saving primitive types.
		    // Specialize for any types/templates that special treatment
		    template<class T>
		    void save(T & t){out<<t<<delimiter;}

		    std::ostream& out;
		    char delimiter;
		public:
		    //////////////////////////////////////////////////////////
		    // public interface used by programs that use the
		    // serialization library

		    // archives are expected to support this function
		    void save_binary(void *address, std::size_t count);

		    delimited_oarchive(std::ostream& out, char delimiter = ' '):out(out),delimiter(' '){};
		};
	}
}

/* DELIMITED_OARCHIVE_HPP_ */
#endif
