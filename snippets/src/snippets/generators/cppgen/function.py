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

class Function:
    """
    Represents the C++ function in the object form.
    
    Class attributes:
    _XMLTAG -- constant string with the xml tag description
    """
    _XMLTAG = 'func'

    def __init__(self, xmlnode = None):
        """
        Populates the variable from a XML node info.
        When the XML node is not defined, does nothing.
        
        xmlnode -- the XML node with the member definition
            
        Instance attributes:
        fname -- name of the function
        fargs -- the argument list 
        fret -- the return type
        fbody -- function body
        finline -- flag i the function is to be in-line
        fqual -- the qualifer of the functon
        """
        self.fname = 'unnamed'
        self.fret = 'void'
        self.fargs = []
        self.fbody = ''
        self.finline = False
        self.fqual = None
        
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
        
        self.fname = xmlnode.get('name',self.fname)
        self.fret = xmlnode.get('return',self.fret)
        self.finline = xmlnode.get('inline',self.finline)
        self.fqual = xmlnode.get('qualifier',self.fqual)
        
        for node in xmlnode.findall('var'):
            self.fargs.append(Variable(node))  
            
        
        
    def gen_code(self, bindent = '', indent = '  '):
        """
        Generates the full member definition C++ code and
        returns it as a string
        
        bindent -- the current indent in the document
        indent -- the usual minimum indentation
        """
        code = bindent + indent + self.fret + ' '
        code = code + self.fname + '(' 
        
        for arg in self.fargs:
            code = code + arg.vtype + ' ' + arg.vname + ',' 
       
        if len(self.fargs): code = code[:-1]
        code = code + ')' 
        if self.fqual != None:
            code = code + ' ' + self.fqual
            
        if self.finline:
            code = code + '\n{\n' + bindent + indent + self.fbody + '\n}'
            
        return code + ";"
    
    def gen_impl(self, namespace = None, bindent = '', indent = '  '):
        """
        Generates the full member definition C++ code and
        returns it as a string
        
        bindent -- the current indent in the document
        indent -- the usual minimum indentation
        """
        if self.finline: return ''
        
        code = bindent + self.fret + ' '
        if namespace != None:
            code = code + namespace + '::'
        
        code = code + self.fname + '(' 
        
        for arg in self.fargs:
            code = code + arg.vtype + ' ' + arg.vname + ',' 
       
        if len(self.fargs): code = code[:-1]
        
        code = code + ')'
        if self.fqual != None:
            code = code + ' ' + self.fqual
            
        return code + '\n{\n' + indent + self.fbody + '\n}\n'
        
    def __str__(self):
        """
        Overriden string specifier returns the C++ code snippet 
        without indentation.
        """
        return self.gen_code(indent = '')
        
        
        