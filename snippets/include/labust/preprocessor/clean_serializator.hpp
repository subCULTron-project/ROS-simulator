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
/*********************************************************************
 * Author: Đula Nađ
 *   Date: 15.11.2013.
 *********************************************************************/
#ifndef CLEAN_SERIALIZATOR_HPP_
#define CLEAN_SERIALIZATOR_HPP_
#include <boost/archive/detail/oserializer.hpp>
#include <boost/archive/detail/iserializer.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/collections_save_imp.hpp>

/**
 * This is a specialization of the array saver to prevent writing the
 * array length by default.
 * It takes the generic save_array_type from the boost archive classes and
 * comments the NVP notation.
 */
#define PP_LABUST_CLEAN_ARRAY_OSERIALIZATOR_IMPL(CLASS_NAME) \
		namespace boost{namespace archive{namespace detail{ \
			template<> \
			struct save_array_type<CLASS_NAME> \
			{ \
			template<class T> \
			static void invoke(CLASS_NAME &ar, const T &t){ \
				typedef BOOST_DEDUCED_TYPENAME boost::remove_extent< T >::type value_type; \
				boost::archive::save_access::end_preamble(ar); \
				std::size_t c = sizeof(t) / ( \
						static_cast<const char *>(static_cast<const void *>(&t[1])) \
						- static_cast<const char *>(static_cast<const void *>(&t[0])) \
				); \
				boost::serialization::collection_size_type count(c); \
				ar << boost::serialization::make_array(static_cast<value_type const*>(&t[0]),count); \
			} \
			};}}};

/**
 * This is a specialization of the array loader to prevent writing the
 * array length by default.
 * It takes the generic save_array_type from the boost archive classes and
 * comments the NVP notation.
 */
#define PP_LABUST_CLEAN_ARRAY_ISERIALIZATOR_IMPL(CLASS_NAME) \
		namespace boost{namespace archive{namespace detail{ \
			template<> \
struct load_array_type<CLASS_NAME> { \
	template<class T> \
	static void invoke(CLASS_NAME &ar, T &t){ \
		typedef BOOST_DEDUCED_TYPENAME remove_extent< T >::type value_type; \
		std::size_t current_count = sizeof(t) / ( \
				static_cast<char *>(static_cast<void *>(&t[1])) \
				- static_cast<char *>(static_cast<void *>(&t[0])) \
		); \
		boost::serialization::collection_size_type count(current_count); \
		ar >> serialization::make_array(static_cast<value_type*>(&t[0]),count); \
	} \
};\
}}};\

/**
 * This is brute-force method for clean serialization of std::string.
 */
template <class Archive>
void clean_string_serialize(Archive &ar, const std::string& s)
{
	uint32_t size = s.size();
	for(int i=0; i<size; ++i) ar & s[i];
}

/* CLEAN_SERIALIZATOR_HPP_ */
#endif
