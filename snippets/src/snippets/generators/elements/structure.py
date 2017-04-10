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

from .variable import Variable
from .enum import Enum

class Structure:
    """
    Represents the C++ data structure in the object form.
    
    Class attributes:
    _XMLTAG -- constant string with the xml tag description
    """
    _XMLTAG = 'struct'
    
    def __init__(self, xmlnode = None):
        """
        Populates the structure from a XML node info.
        When the XML node is not defined, does nothing.
        
        xmlnode -- the XML node with the member definition 
        
        Required attributes:
        name    -- name of the structure
        Optional attributes:
        inherit -- the class that is inherited
        assert_size -- add size assertion in bytes (useful for bitfields)
        serialization -- the type of serialization (defaults to object serializable)
        bitfield -- marks the structure as a bit-field
        """
        self.name = 'Unnamed'
        self.inherit = None
        self.assert_size = None  
        self.serialization = 'object_serializable'
        self.bitfield = False
        
        self.variables = []
        self.methods = []
        self.enums = []
        self.typedefs = []
         
        if xmlnode != None: self.from_xml(xmlnode)  
        
    def from_xml(self, xmlnode):
        """
        Extracts the structure attributes from the XML node.
        Memeber variables and functions are extracted from the 
        child nodes as well.
        
        xmlnode -- the XML node with the member definition
        """
        
        if xmlnode.tag != self._XMLTAG:
            raise NameError(self.__class__.__name__ +
                            ' expected XML tag: "' + 
                            self._XMLTAG + 
                            '"')
        
        self.name = xmlnode.get('name', self.name)
        self.inherit = xmlnode.get('inherit', self.inherit)
        self.assert_size = xmlnode.get('assert_size', self.assert_size)
        self.serialization = xmlnode.get('serialization', self.serialization)
        bitfield = xmlnode.get('bitfield', '')
        self.bitfield = (bitfield == "1") or (bitfield.upper() == "TRUE")
          
        for node in xmlnode.findall('.//'+Variable._XMLTAG):
            self.variables.append(Variable(node))
        
        for node in xmlnode.findall(Enum._XMLTAG):
            self.enums.append(Enum(node))
        
        