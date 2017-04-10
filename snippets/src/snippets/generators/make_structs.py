#!/usr/bin/env python
from snippets.generators.cppgen.cppgen import CppGen
from snippets.generators.elements.structure import Structure
from snippets.generators.elements.function import Function
from snippets.generators.elements.variable import Variable
import xml.etree.ElementTree as ET
from _dbus_bindings import String

def gen_factory_initializer(structs, bindent = '', indent = '  '):
    """
    Generates the factory map initializer.
        
    bindent -- the current indent in the document
    indent -- the usual minimum indentation
    """
    code = ''
    for struct in structs:
        code = (code + indent + '{' + struct.name + '::CID, createInstance<' +
         struct.name + '>},\n')   
    
    #Remove the last comma and newline
    return code[:-2]

def gen_names(structs, bindent = '', indent = '  '):
    """
    Generates the human readable names of CIDs.
        
    bindent -- the current indent in the document
    indent -- the usual minimum indentation
    """
    code = ''
    for struct in structs:
        code = (code + indent + '{' + struct.name + '::CID, \"' + 
            struct.name + '\"},\n')   
    
    #Remove the last comma and newline
    return code[:-2]


def create_datatypes(xmlroot, gen, folder = ''):
    structs = []
    
    for node in xmlroot.findall('struct'):
        structs.append(Structure(node))
        
    for struct in structs:
        gen.packable_types.append(struct.name)
        
    fdt = open(folder + '/' + 'datatype_defs.h','w')
    fdt2 = open(folder + '/' + 'datatype_impl.h','w')
    for struct in structs:
        fdt.write(gen.gen_header(struct))
        fdt.write('\n\n')
        fdt2.write(gen.gen_impl(struct, serialization = False))
        fdt2.write('\n\n')
        
    fdt.close()
    
    return structs
          
def create_messages(xmlroot, gen, prefix, folder = ''):
    structs = []
    for node in xmlroot.findall('struct'):
        structs.append(Structure(node))
    
    for struct in structs:
        gen.packable_types.append(struct.name)
        
    for s in structs:
        '''Add automatic inheritance and virtual method implementations'''
        s.inherit = 'SeatracMessage'
        #Ptr typedef
        t = ('boost::shared_ptr< ' + s.name +' > ', 'Ptr')     
        s.typedefs.append(t)  
        t = ('boost::shared_ptr< ' + s.name +' const > ', 'ConstPtr') 
        s.typedefs.append(t)  
        #get_cid function
        f = Function()
        f.name = 'getCid'
        f.ret = 'int'
        f.body = 'return ' + s.name + '::CID;'
        f.inline = True
        f.qual = 'const'
        s.methods.append(f)
        
        #Test message type
        f = Function()
        f.name = 'isCommand'
        f.ret = 'bool'
        if prefix == "command":
            f.body = 'return true;'
        else:
            f.body = 'return false;'
            
        f.qual = 'const'
        s.methods.append(f)
                      
    
    filebase = folder + '/' + prefix
    
    cmi = open(filebase + '_factory_initializer.h','w')
    cmi.write(gen_factory_initializer(structs))
    cmi.close()
    
    cmi = open(filebase + '_names.h','w')
    cmi.write(gen_names(structs))
    cmi.close()
    
    smsg = open(filebase + '_defs.h','w')
    smsgimpl = open(filebase + '_impl.h','w')
    for struct in structs:
        smsg.write(gen.gen_header(struct))
        smsg.write('\n\n')
        smsgimpl.write(gen.gen_impl(struct, serialization=False))
        smsgimpl.write('\n\n')
    smsg.close()
        
    return structs

def cpp_create_structs(gen, definition, folder, name):
    structs = []
    
    xmlroot = ET.parse(definition).getroot()
    
    for node in xmlroot.findall('struct'):
        structs.append(Structure(node))
        
    for struct in structs:
        gen.packable_types.append(struct.name)
        
    fdt = open(folder + '/' + name + '_defs.h','w')
    fdt2 = open(folder + '/' + name + '_impl.h','w')
        
    for struct in structs:
        fdt.write(gen.gen_header(struct))
        fdt.write('\n\n')
        fdt2.write(gen.gen_impl(struct, serialization = False))
        fdt2.write('\n\n')
        
    fdt.close()
    fdt2.close()

        
    return structs
    

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Create C++ or Java structures from XML defintion.')
    parser.add_argument('--definition', 
                        help='the structure XML definition file', required=True)
    parser.add_argument('--output-dir', default='.',
                        help='directory where to save the files (default: .)')
    parser.add_argument('--output-name', default='datatype',
                        help='base name of the definition and implementation file (default: datatype)')

    args = parser.parse_args()
    
    
    gen = CppGen()
    cpp_create_structs(gen, args.definition, args.output_dir, args.output_name)
    
#     
#     gen = CppGen()
#     gen.array_len_type = 'uint8_t'   
#     
#     '''Parse the Seatrac data types and create the definitions'''

#     '''Parse the Seatrac commands and create the initializer'''
#     cmd = create_messages(ET.parse('definitions/SeatracCommands.xml').getroot(),
#                           gen, 
#                           'command',
#                           'include/labust/seatrac/detail/')
#     '''Parse the Seatrac responses and create the initializer'''
#     resp = create_messages(ET.parse('definitions/SeatracResponses.xml').getroot(),
#                            gen, 
#                            'response',
#                            'include/labust/seatrac/detail/')  
#     
#     exit()
#     
#     structs.extend(dt);
#     structs.extend(cmd);
#     structs.extend(resp);
#   
#     '''Create boost serializator for all defined structures'''  
#     serdef = open('include/labust/seatrac/detail/serialization_defs.h','w')
#     for struct in structs:
#         serdef.write(struct_serializer.gen_serializer(struct, ns='labust::seatrac'))
#         serdef.write('\n')
        
