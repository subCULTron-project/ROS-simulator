#!/usr/bin/env python
from snippets.generators.elements.structure import Structure
from snippets.generators.elements.variable import Variable
from snippets.generators.elements.function import Function
import xml.etree.ElementTree as ET

class JavaGen():
    def __init__(self):
        
        self.storage_equiv = {
        'bool': 'byte',
        'int8_t': 'byte',
        'uint8_t': 'int',
        'int16_t': 'short',
        'uint16_t': 'int',
        'int32_t': 'int',
        'uint32_t': 'long',
        'int64_t': 'long',
        'uint64_t': 'float'
        }
        
        self.type_unsigned_corr = {
         'uint8_t': ' & 0xFF',
         'uint16_t': ' & 0xFFFF',
         'uint32_t': ' & 0xFFFFFFFFL'
        }
        
        self.type_equiv = {
        'bool': 'byte',
        'int8_t': 'byte',
        'uint8_t': 'byte',
        'int16_t': 'short',
        'uint16_t': 'short',
        'int32_t': 'int',
        'uint32_t': 'int',
        'int64_t': 'long',
        'uint64_t': 'long'
        }
        
        self.type_size = {
         'byte':1,
         'short':2,
         'int':4,
         'long':8,
         'float':4,
         'double':8}
               
        self.primitive_types = self.type_equiv.keys()
        self.packable_types = []
        self.special_types = {}
        self.array_len_type = 'int'
    
    def gen_variable(self, var, indent = ''):
        '''All variables default to a public specifier'''
        etype =  self.type_equiv.get(var.type,var.type)
        type = self.storage_equiv.get(var.type,etype)
        cinit = var.default
        if cinit == None:
            if self.is_array_type(var.type):
                len = self.get_array_len(var.type)
                etype = self.type_equiv.get(var.type,var.type)
                if len == '':
                    cinit = 'new ' + etype + '[0]'
                else:
                    cinit = 'new ' + etype + '[' + len + ']'
                type = etype + '[]'
            elif var.type not in self.primitive_types:
                cinit = 'new ' + type + '()'
                                 
        cvar = indent + 'public ' + type + ' ' + var.name
                
        if cinit != None:
            cvar = cvar + ' = ' + cinit
        
        return cvar + ';'
    
    def gen_enum(self, enum, indent = ''):
        return indent + 'public final static int ' + enum.name + ' = ' + enum.value + ';'
    
    def gen_method(self, method, indent):
        rtype = self.storage_equiv.get(method.ret,method.ret)
        code = indent;
        if method.static:
            code = code + 'static '
        code = code + 'public ' + rtype + ' '
        code = code +  method.name + '('
    
        for arg in method.args:
            type = self.type_equiv.get(arg.type,arg.type)
            code = code + type + ' ' + arg.name + ','
        
        if len(method.args): code = code[:-1]
            
        code = code + ')'
        if method.qual != None:
            code = code + ' ' + method.qual + ' '
            
        code = code + '{\n' 
        code = code + indent + '  ' + method.body + '\n'
        code = code + indent + '}'
        
        return code
    
    def gen_class(self, struct, package, imports = [], serialization = True):
        code = 'package ' + package + ';\n'
        code = code + "\n"
    
        #Imports
        if serialization:
            imports_ser = [
                'com.google.common.io.LittleEndianDataInputStream',
                'com.google.common.io.LittleEndianDataOutputStream',
                'java.io.IOException']
        
            for im in imports_ser:
                code = code + 'import ' + im + ';\n'
             
        for im in imports:
            code = code + 'import ' + im + ';\n' 
    
        code = code + '\n'
    
        #Class definition
        code = code + 'public class ' + struct.name
            
        if struct.inherit != None:
            code = code + ' implements ' + struct.inherit 
        code = code + ' {\n'
    
        #Skip typedefs
    
        #Generate enums as final static int
        for enum in struct.enums:
            code = code + self.gen_enum(enum, '  ') + '\n'
        code = code + "\n"
        
        for var in struct.variables:
            code = code + self.gen_variable(var, '  ') + '\n'
        
        code = code + "\n"
                    
        #Methods
        if serialization: self.add_serializers(struct, '  ')          
        for method in struct.methods:
            code = code + self.gen_method(method, '  ') + '\n\n'
            
        #End class definition
        code = code + '}\n' 
    
        return code
    
    def add_serializers(self, struct, indent):
        # Adhers to the packer interface
        fn = Function()
        fn.name = 'pack' 
        fn.ret = 'void'
        fn.qual = 'throws IOException'
        arg = Variable()
        arg.name = 'out'
        arg.type = 'LittleEndianDataOutputStream'
        fn.args.append(arg)
        fn.body = self.gen_serializer_body(struct, indent)
        struct.methods.append(fn)
        
        fn = Function()
        fn.name = 'unpack' 
        fn.ret = 'void'
        fn.qual = 'throws IOException'
        arg = Variable()
        arg.name = 'in'
        arg.type = 'LittleEndianDataInputStream'
        fn.args.append(arg)
        fn.body = self.gen_deserializer_body(struct, indent)
        struct.methods.append(fn)
        

    def gen_bitfield_serializer(self, struct, indent):
        store_type = ''
        sz = int(struct.assert_size)
        if sz == 1:
            store_type = 'byte'
        if sz == 2:
            store_type = 'short'
        elif (sz > 2) and (sz <= 4):
            store_type = 'int'
        elif (sz > 4) and (sz <= 8):
            store_type = 'long'
        elif sz != 1:
            print('Cannot create a serializer for size '+ str(sz))
            return '' 
        
        code = indent + store_type + ' storage = 0;\n'
        shift = 0  
        for var in struct.variables:
            if var.bits == None: 
                print('All variables need a bit size in a bitfield.')
                return ''
            code = code + indent + 'storage |= ( (' 
            code = code + store_type + ')(' + var.name
            code = code + ' & ((1<<' + var.bits + ')-1))'
            code = code + ' << ' + str(shift) + ');\n'
            shift += int(var.bits)
            
        # Write minimal byte count
        code = code + indent + 'ByteArrayOutputStream baos = new ByteArrayOutputStream(); \n'
        code = code + indent + 'LittleEndianDataOutputStream dos = new LittleEndianDataOutputStream(baos);\n'
        code = code + indent + 'dos.write' + store_type.title() + '(storage);\n'
        code = code + 'dos.close();\n'
        code = code + 'byte[] bb = baos.toByteArray();\n'
        code = code + 'for(int i=0; i<' + str(sz) + ';++i) out.writeByte(bb[i]);\n'
        
        return code
    
    def gen_bitfield_deserializer(self, struct, indent):
        store_type = ''
        sz = int(struct.assert_size)
        ssz = 1
        if sz == 1:
            store_type = 'byte'
        if sz == 2:
            ssz = 2
            store_type = 'short'
        elif (sz > 2) and (sz <= 4):
            ssz = 4
            store_type = 'int'
        elif (sz > 4) and (sz <= 8):
            ssz = 8
            store_type = 'long'
        elif sz != 1:
            print('Cannot create a serializer for size '+ str(sz))
            return '' 
        
        code = indent + 'byte[] buffer = new byte[' + str(ssz) + '];\n'
        code = code + indent + 'for(int i=0; i<' + str(sz) + ';++i) buffer[i] = in.readByte();\n'
        code = code + indent + 'ByteArrayInputStream bais = new ByteArrayInputStream(buffer); \n'
        code = code + indent + 'LittleEndianDataInputStream dis = new LittleEndianDataInputStream(bais);\n'
        code = code + indent + store_type + ' storage ='        
        code = code + indent + 'dis.read' + store_type.title() + '();\n'
        code = code + 'dis.close();\n'

        for var in struct.variables:
            if var.bits == None: 
                print('All variables need a bit size in a bitfield.')
                return ''
            code = code + indent + var.name + ' = ('
            code = code + self.type_equiv.get(var.type, var.type) + ') '
            code = code + '(storage & ((1<<' + var.bits + ')-1));\n' 
            code = code + indent + 'storage >>>= ' + str(var.bits) + ';\n'
        
        return code
        
    def is_array_type(self, type):
        return ((type.find('[') != -1) and
            (type.find(']') != -1))
    
    def get_array_len(self, type):
        sidx = type.find('[')
        eidx = type.find(']')
        return type[sidx+1:eidx]
    
    def get_array_type(self, type):
        sidx = type.find('[')
        return type[:sidx]
            
    def gen_array_serializer(self, var, indent):
        code = indent
        type = self.get_array_type(var.type)
        type = self.type_equiv.get(type,type)
        len = self.get_array_len(var.type)
               
        if len == '':
            code = code + indent + 'out.write' + self.array_len_type.title()
            code = code + '(' + var.name +'.length);\n'
        
        if type == 'byte':
            code = code + indent + 'out.write(' + var.name + ');'
        else:
            code = code + indent + 'for(int i=0; i<' + var.name + '.length; ++i) '
            code = code + 'out.write' + type.title() + '('
            code = code + var.name + '[i]);\n'
            
        return code
    
    def gen_array_deserializer(self, var, indent):
        code = indent
        type = self.get_array_type(var.type)
        type = self.type_equiv.get(type,type)
        len = self.get_array_len(var.type)
               
        if len == '':
            code = code + indent + var.name + ' = new ' + type
            code = code + '[' + 'in.read' + self.array_len_type.title() + '()];\n'
        
        if type == 'byte':
            code = code + indent + 'in.read(' + var.name + ');'
        else:
            code = code + indent + 'for(int i=0; i<' + var.name + '.length; ++i) '
            code = code + var.name + '[i] = ' + 'in.read' + type.title() + '();\n'
            
        return code
            
    def gen_serializer_body(self, struct, indent):
        # Specially handle bitfields          
        if struct.bitfield: return self.gen_bitfield_serializer(struct, indent)
                        
        # Serialize each memeber variable
        code = ''
        for var in struct.variables:  
            if var.cond != None:
                code = code + indent + 'if (' + var.cond + ' == 1){\n'
                          
            if self.is_array_type(var.type):
                code = code + indent + self.gen_array_serializer(var, indent)
            elif var.type in self.primitive_types:
                type = self.type_equiv.get(var.type,var.type)
                code = code + indent + 'out.write'
                code = code + type.title() + '( (' + type + ')'
                code = code + var.name + ');\n'
            elif var.type in self.packable_types:
                code = code + indent + var.name + '.pack(out);\n'
            elif var.type in self.special_types.keys():
                code = code + indent + self.special_types[var.type](var, "serializer")
            else:
                print('Cannot create a serializer for type: ' + var.type)
                
            if var.cond != None:
                code = code + indent + '}\n'
            
        
        return code
    
    def gen_deserializer_body(self, struct, indent):
        # Specially handle bitfields          
        if struct.bitfield: return self.gen_bitfield_deserializer(struct, indent)
                        
                # Serialize each memeber variable
        code = ''
        for var in struct.variables:            
            if var.cond != None:
                code = code + indent + 'if (' + var.cond + ' == 1){\n'
                
            if self.is_array_type(var.type):
                code = code + indent + self.gen_array_deserializer(var, indent)
            elif var.type in self.primitive_types:
                code = code + indent + var.name + ' = ('
                code = code + self.storage_equiv.get(var.type, var.type) + ') in.read'
                code = code + self.type_equiv.get(var.type,var.type).title()
                code = code + '()' + self.type_unsigned_corr.get(var.type, '') + ';\n'
            elif var.type in self.packable_types:
                code = code + indent + var.name + '.unpack(in);\n'
            elif var.type in self.special_types.keys():
                code = code + indent + self.special_types[var.type](var, "deserializer")
            else:
                print('Cannot create a serializer for type: ' + var.type)
            
            if var.cond != None:
                code = code + indent + '}\n'
        return code
