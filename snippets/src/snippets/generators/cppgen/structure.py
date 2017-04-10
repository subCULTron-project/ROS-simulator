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
    
    class VarGroup:
        """
        Represents a variable group inside the structure object.
    
        Class attributes:
        _XMLTAG -- constant string with the xml tag description
        """
        _XMLTAG = 'group'
        
        def __init__(self, xmlnode = None):
            """
            Populates the structure from a XML node info.
            When the XML node is not defined, does nothing.
        
            xmlnode -- the XML node with the member definition 
        
            Instance attributes:
            gvariables -- the member variables list
            gcond -- the group conditional
            """
            self.gcond = None
         
            if xmlnode != None: self.from_xml(xmlnode) 
            
        def from_xml(self, xmlnode):
            """
            Extracts the group attributes from the XML node.
            Memeber variables are extracted from the child 
            nodes as well.
        
            xmlnode -- the XML node with the member definition
            """
        
            if xmlnode.tag != self._XMLTAG:
                raise NameError(self.__class__.__name__ +
                                ' expected XML tag: "' + 
                                self._XMLTAG + 
                                '"')
            
            self.gcond = xmlnode.get('if', self.gcond)
                   
            for node in xmlnode.findall(Variable._XMLTAG):
                self.gvariables.append(Variable(node))
        

    def __init__(self, xmlnode = None):
        """
        Populates the structure from a XML node info.
        When the XML node is not defined, does nothing.
        
        xmlnode -- the XML node with the member definition 
        
        Instance attributes:
        sname    -- name of the structure
        sinherit -- the class that is inherited
        svariables -- the direct member variables list
        sgroups -- the grouped member variables list
        smethods -- the member methods list
        senums -- the enumerations list
        sassert_size -- add size assertion in bytes (useful for bitfields)
        sserialization -- the type of serialization (defaults to object serializable)
        sid -- the numeric id constant (useful for command structs)
        """
        self.sname = 'Unnamed'
        self.sinherit = None
        self.svariables = []
        self.sgroups = []
        self.smethods = []
        self.stypedefs = []  
        self.senums = []
        self.sassert_size = None  
        self.sserialization = 'object_serializable'
         
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
        
        self.sname = xmlnode.get('name', self.sname)
        self.sinherit = xmlnode.get('inherit', self.sinherit)
        self.sassert_size = xmlnode.get('assert_size', self.sassert_size)
        self.sserialization = xmlnode.get('serialization', self.sserialization)
        
        for node in xmlnode.findall('.//'+Variable._XMLTAG):
            self.svariables.append(Variable(node))
        
        for node in xmlnode.findall(Enum._XMLTAG):
            self.senums.append(Enum(node))
        
    def gen_code(self, bindent = '', indent = '  '):
        """
        Generates the full structure definition C++ code and
        returns it as a string.
        
        bindent -- the current indent in the document
        indent -- the usual minimum indentation
        """
        code = bindent + indent + 'struct ' + self.sname
        if self.sinherit != None:
            code = code + ': public ' + self.sinherit
        
        code = code + '\n{\n'
        
        for t in self.stypedefs:
            code = code + 'typedef ' + t[0] + ' ' + t[1] + ';\n' 
        
        for enum in self.senums:
            code = code + enum.gen_code() + '\n'
            
        if len(self.senums): code = code + '\n'
            
        for method in self.smethods:
            code = code + method.gen_code() + '\n'
            
        for var in self.svariables:
            code = code + var.gen_code() + '\n'
            
        code = code + '};\n'  
            
        if self.sassert_size != None:
            code = code + ('BOOST_STATIC_ASSERT((sizeof(' + self.sname +') == ' + 
                self.sassert_size + ')' + ' && ("' + self.sname + 
                ' structure is assumed as size ' + self.sassert_size + 
                ' bytes."));\n')
                    
        return code  
    
    def gen_impl(self, bindent = '', indent = '  '):
        """
        Generates the method implementations in C++ code and
        returns it as a string.
        
        bindent -- the current indent in the document
        indent -- the usual minimum indentation
        """ 
        code = ''         
        for method in self.smethods:
            code = code + method.gen_impl(namespace=self.sname) + '\n'
                    
        return code  
        
    def __str__(self):
        """
        Overriden string specifier returns the C++ code snippet 
        without indentation.
        """
        return self.gen_code(indent = '')
        
        
        