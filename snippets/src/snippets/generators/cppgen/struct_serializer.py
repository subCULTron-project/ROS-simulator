"""
Created on Nov 22, 2014

Software License Agreement (BSD License)
 Copyright (c) 2014, LABUST, UNIZG-FER
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:
 
 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above
   copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided
   with the distribution.
 * Neither the name of the LABUST nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

@author: Dula Nad
"""

def gen_serializer(struct, ns = None, bindent = '', indent = '  '):
    """
    Generates the serialization code from the structure 
    object.
        
    bindent -- the current indent in the document
    indent -- the usual minimum indentation
    """
    code = 'BOOST_CLASS_IMPLEMENTATION(' 
    if ns != None:
        code = code + ns + '::'
    code = code + (struct.sname + ',' +
              ' boost::serialization::' + struct.sserialization) + ')\n'
                   
    if struct.sserialization != 'primitive_type':
        #Create a serializer
        code = code + 'namespace boost { namespace serialization {\n'
        code = code + 'template<class Archive>\n'
        code = code + 'void serialize(Archive& ar, '
        if ns != None:
            code = code + ns + '::'
            
        code = code + struct.sname + '& object, '
        code = code + 'const unsigned int version)\n{\n'       
        for var in struct.svariables:
            if var.vcond != None:
                code = code + 'if (object.' + var.vcond + ') '
            code = code + 'ar & object.' + var.vname + ';\n' 
        
        code = code + '}}};\n'
        
    return code