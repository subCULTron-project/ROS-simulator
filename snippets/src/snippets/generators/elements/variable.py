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

class Variable:
    """
    Represents the C++ data member/variable in the object form.
    
    Class attributes:
    _XMLTAG -- constant string with the xml tag description
    """
    _XMLTAG = 'var'

    def __init__(self, xmlnode = None):
        """
        Populates the variable from a XML node info.
        When the XML node is not defined, does nothing.
        
        xmlnode -- the XML node with the member definition
            
        Required attributes:
        name    -- name of the variable
        type    -- type name of the variable
        Optional attributes:
        bits - number of bits that should be reserved for the variable
        min - minimal value of the variable
        max - maximal value of the variable
        quant - the quantization of the variable
        cond - conditional serialization (will be serialized only when the value is True)
        default - the default value of the variable
        """
        self.name = 'unnamed'
        self.type = 'void'
        self.default = None
        self.cond = None
        self.bits = None
        self.min = None
        self.max = None
        self.quant = None
        
        if xmlnode != None: self.from_xml(xmlnode)
        
    def from_xml(self, xmlnode):
        """
        Extracts the variable attributes from the XML node.
        
        xmlnode -- the XML node with the member definition
        """
        
        if xmlnode.tag != self._XMLTAG:
            raise NameError(self.__class__.__name__ + 
                            ' expected XML tag: "' + 
                            self._XMLTAG + 
                            '"')
        
        self.name = xmlnode.get('name',self.name)
        self.type = xmlnode.get('type',self.type)
        self.default = xmlnode.get('default', self.default)
        self.cond = xmlnode.get('if', self.cond)
        self.bits = xmlnode.get('bits', self.bits)
        self.min = xmlnode.get('min', self.min)
        self.max = xmlnode.get('max', self.max)
        self.quant = xmlnode.get('quant', self.quant)        
        
        