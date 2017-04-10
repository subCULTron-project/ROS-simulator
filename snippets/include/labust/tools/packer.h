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
 *  Created: 29.04.2013.
 *********************************************************************/
#ifndef USBL_COMMS_PACKER_H
#define USBL_COMMS_PACKER_H
#include <vector>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/stream.hpp>

namespace labust
{
	namespace tools
	{
		/**
		 * Encodes all messages that have a pack/unpack interface.
		 */
		template <class Packable>
		inline void encodePackable(const Packable& data, std::vector<char>* binary)
		{
			using namespace boost::iostreams;
			typedef back_insert_device< std::vector<char> > smsink;
			smsink sink(*binary);
			stream<smsink> os(sink);
			boost::archive::binary_oarchive outser(os, boost::archive::no_header);
			data.pack(outser);
			os.flush();
		}

		/**
		 * Decodes all messages that have a pack/unpack interface.
		 */
		template <class VectorType, class Packable>
		inline bool decodePackable(const std::vector< VectorType >& binary, Packable* data)
		try
		{
			using namespace boost::iostreams;
			array_source source(reinterpret_cast<const char*>(binary.data()),
					binary.size());
		  stream<array_source> is(source);
			boost::archive::binary_iarchive inser(is, boost::archive::no_header);
			data->unpack(inser);
			return true;
		}
		catch (std::exception& e)
		{
			return false;
		}
	}
}
/* USBL_COMMS_PACKER_H*/
#endif



