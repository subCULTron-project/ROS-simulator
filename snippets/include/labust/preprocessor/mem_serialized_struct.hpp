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
 *   Date: 20.10.2010.
 *********************************************************************/
#ifndef MEM_SERIALIZED_STRUCT_HPP_
#define MEM_SERIALIZED_STRUCT_HPP_
#include <labust/preprocessor/class_adaptor.hpp>
#include <boost/serialization/level.hpp>
#include <boost/preprocessor/seq/for_each.hpp>

#define PP_LABUST_ADD_TO_BOOST_ARCHIVE(R, ATTRIBUTE_TUPEL_SIZE, ATTRIBUTE) \
	ar &  object.BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPEL_SIZE,1,ATTRIBUTE);

#define PP_LABUST_MAKE_INLINE_BOOST_SERIALIZATOR(NAME, ATTRIBUTES) \
	namespace boost { namespace serialization{\\
	template<class Archive>\
  inline void serialize(Archive & ar, NAME & g, const unsigned int version) \
  {\
    BOOST_PP_SEQ_FOR_EACH_R(1, PP_LABUST_ADD_TO_BOOST_ARCHIVE, object, STRUCT_SEQ)\
  }}}\

#define PP_LABUST_MAKE_BOOST_SERIALIZATOR(NAME, ATTRIBUTES) \
	namespace boost { \
  namespace serialization { \
	template<class Archive>\
  void serialize(Archive & ar, NAME & object, const unsigned int version) \
  {\
		PP_LABUST_MACRO_ON_ATTRIBUTES(PP_LABUST_ADD_TO_BOOST_ARCHIVE,ATTRIBUTES)\
  }}}

#define PP_LABUST_MAKE_INLINE_BOOST_SERIALIZATOR_CLEAN(NAME, ATTRIBUTES) \
	PP_LABUST_MAKE_INLINE_BOOST_SERIALIZATOR(NAME, ATTRIBUTES) \
	BOOST_CLASS_IMPLEMENTATION(NAME, boost::serialization::object_serializable)

#define PP_LABUST_MAKE_BOOST_SERIALIZATOR_CLEAN(NAME, ATTRIBUTES) \
	PP_LABUST_MAKE_BOOST_SERIALIZATOR(NAME, ATTRIBUTES) \
	BOOST_CLASS_IMPLEMENTATION(NAME, boost::serialization::object_serializable)

#define PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT(NAMESPACE_SEQ, NAME, ATTRIBUTES)\
		PP_LABUST_NAMESPACE_DEFINITIONS_BEGIN((0)NAMESPACE_SEQ)\
		struct NAME\
		{\
			PP_LABUST_MACRO_ON_ATTRIBUTES(PP_LABUST_ATTRIBUTE_EXPAND,ATTRIBUTES)\
		};\
		PP_LABUST_NAMESPACE_DEFINITIONS_END((0)NAMESPACE_SEQ)\
		PP_LABUST_MAKE_BOOST_SERIALIZATOR(PP_LABUST_NAMESPACE_DEFINITIONS_ENUM((0)NAMESPACE_SEQ)NAME,ATTRIBUTES)

#define PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN(NAMESPACE_SEQ, NAME, ATTRIBUTES)\
	PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT(NAMESPACE_SEQ, NAME, ATTRIBUTES)\
	BOOST_CLASS_IMPLEMENTATION(PP_LABUST_NAMESPACE_DEFINITIONS_ENUM((0)NAMESPACE_SEQ)NAME, boost::serialization::object_serializable)


/* MEM_SERIALIZED_STRUCT_HPP_ */
#endif
