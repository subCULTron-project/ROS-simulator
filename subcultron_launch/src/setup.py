#!/usr/bin/env python

"""!
@package Initializes ROS node for high level mission control.
@author UNIZG-FER <barbara.arbanas@fer.hr>
"""

import sys
import rospkg
from copy import deepcopy
from random import uniform
from auv_msgs.msg import NED
import xml.etree.ElementTree

########################################

sceneSpecTemplate = "swarm_test_raw.xml"
sceneSpecFile = "swarm_test.xml"

launchFileTemplate = "standard_simulation_raw.launch"
launchFile = "standard_simulation.launch"

########################################

agents = []
controllerFile = ""
simulationSpecFile = ""

outputToScreen = True


def indent(elem, level=0):
    """!
    @brief Indents xml tree element to a desired level.
    @param elem Element of the xml tree.
    @param level Level of the element.
    """
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i


def fill_up_simulation_spec_file(root, n_pad, first_index_pad, n_fish, first_index_fish, n_mussel, first_index_mussel):
    """!
        @brief Fills up simulation specification file.

        @param root Root of xml simulation specification file.
        @param n_pad Total number of aPads.
        @param first_index_pad Index of first aPad used in the simulation.
        @param n_fish Total number of aFish.
        @param first_index_fish Index of first aFish used in the simulation.
        @param n_mussel Total number of aMussels.
        @param first_index_mussel Index of first aMussel used in the simulation.
    """
    
    vehicle_root = xml.etree.ElementTree.parse(rospack.get_path('apad') + '/data/simulation/' + 'vehicle.xml').getroot()
    size = len(root)
    
    for i in range(n_pad):
        tmp = deepcopy(vehicle_root)
        name = "apad" + str(first_index_pad + i + 1)
        tmp.find("name").text = name
        tmp.find("imu").find("name").text = "imu" + str(first_index_pad + i + 1)
        tmp.find("gpsSensor").find("name").text = "GPSSensor" + str(first_index_pad + i + 1)
        root.insert(size - 1, tmp)    

    vehicle_root = xml.etree.ElementTree.parse(rospack.get_path('afish') + '/data/simulation/' + 'vehicle.xml').getroot()
    size = len(root)
    
    for i in range(n_fish):
        tmp = deepcopy(vehicle_root)
        name = "afish" + str(first_index_fish + i + 1)
        tmp.find("name").text = name
        tmp.find("imu").find("name").text = "imu" + str(first_index_fish + i + 1)
        tmp.find("gpsSensor").find("name").text = "GPSSensor" + str(first_index_fish + i + 1)
        root.insert(size - 1, tmp)  
    
    vehicle_root = xml.etree.ElementTree.parse(rospack.get_path('amussel') + '/data/simulation/' + 'vehicle.xml').getroot()

    for i in range(n_mussel):
        tmp = deepcopy(vehicle_root)
        name = "amussel" + str(first_index_mussel + i + 1)
        tmp.find("name").text = name
        tmp.find("imu").find("name").text = "imu" + str(first_index_mussel + i + 1)
        tmp.find("gpsSensor").find("name").text = "GPSSensor" + str(first_index_mussel + i + 1)
        root.insert(size - 1, tmp)    

    root.append(xml.etree.ElementTree.Element("rosInterfaces"))    
    for i in range(n_pad):
        name = "apad" + str(first_index_pad + i + 1)
        tmp = xml.etree.ElementTree.Element("ROSOdomToPAT")
        tmp.append(xml.etree.ElementTree.Element("topic"))
        tmp[-1].text = name + "/uwsim_hook"
        
        tmp.append(xml.etree.ElementTree.Element("vehicleName"))
        tmp[-1].text = name
        
        root.find("rosInterfaces").append(tmp)

    for i in range(n_fish):
        name = "afish" + str(first_index_fish + i + 1)
        tmp = xml.etree.ElementTree.Element("ROSOdomToPAT")
        tmp.append(xml.etree.ElementTree.Element("topic"))
        tmp[-1].text = name + "/uwsim_hook"
        
        tmp.append(xml.etree.ElementTree.Element("vehicleName"))
        tmp[-1].text = name
        
        root.find("rosInterfaces").append(tmp)
          
    for i in range(n_mussel):
        name = "amussel" + str(first_index_mussel + i + 1)
        tmp = xml.etree.ElementTree.Element("ROSOdomToPAT")
        tmp.append(xml.etree.ElementTree.Element("topic"))
        tmp[-1].text = name + "/uwsim_hook"
        
        tmp.append(xml.etree.ElementTree.Element("vehicleName"))
        tmp[-1].text = name
        
        root.find("rosInterfaces").append(tmp)

        
def fill_up_launch_file(root, n_pad, positions_pad, first_index_pad, battery_pad, n_fish, positions_fish,
                        first_index_fish, battery_fish, n_mussel, positions_mussel, first_index_mussel, battery_mussel):
    """!
        @brief Fills up launch file.

        @param root Root of xml launch file.
        @param n_pad Total number of aPads.
        @param positions_pad Positions of aPads used in the simulation.
        @param first_index_pad Index of first aPad used in the simulation.
        @param battery_pad Battery levels of aPads used in the simulation.
        @param n_fish Total number of aFish.
        @param positions_fish Positions of aFish used in the simulation.
        @param first_index_fish Index of first aFish used in the simulation.
        @param battery_fish Battery levels of aFish used in the simulation.
        @param n_mussel Total number of aMussels.
        @param positions_mussel Positions of aMussels used in the simulation.
        @param first_index_mussel Index of first aMussel used in the simulation.
        @param battery_mussel Battery levels of aMussels used in the simulation.
    """

    if "aPad" in agents:

        for i in range(n_pad):
            name = 'apad' + str(first_index_pad + i + 1)
            group = xml.etree.ElementTree.Element("group", {"ns": name})
            
            # Load parameters
            group.append(xml.etree.ElementTree.Element("rosparam", {"file": "$(find apad)/data/navigation/params/"
                                                                            "nav_standard.yaml"}))
            group.append(xml.etree.ElementTree.Element("rosparam", {"file": "$(find apad)/data/control/params/"
                                                                            "controllers_standard.yaml"}))
            group.append(xml.etree.ElementTree.Element("rosparam", {"file": "$(find apad)/data/dynamics/apad.yaml"}))
            group.append(xml.etree.ElementTree.Element("rosparam", {"param": "eta0"}))
            group[-1].text = "[" + str(positions_pad[i].north) + "," + str(positions_pad[i].east) + "," + \
                             str(positions_pad[i].depth) + ",0,0,0]"

            # Location parameters
            group.append(xml.etree.ElementTree.Element("rosparam", {"command": "load", "file": "$(find apad)/data/"
                                                                                               "locations/"
                                                                                               "swarm_loc.yaml"}))
            
            # Static TF
            group.append(xml.etree.ElementTree.Element("include", {"file": "$(find apad)/data/devices/"
                                                                           "static_frames.xml"}))

            # Load the simulation
            group.append(xml.etree.ElementTree.Element("include", {"file": "$(find apad)/data/simulation/scenario/" +
                                                                           simulationSpecFile}))
            group[-1].append(xml.etree.ElementTree.Element("arg", {"name": "battery_level",
                                                                   "value": str(battery_pad[i])}))
            
            # Load the controllers
            group.append(xml.etree.ElementTree.Element("include", {"file": "$(find apad)/data/control/"
                                                                           "control_standard.xml"}))
            
            # Load visualization
            group.append(xml.etree.ElementTree.Element("include", {"file": "$(find apad)/data/simulation/"
                                                                           "visualization_standard.xml"}))
            group[-1].append(xml.etree.ElementTree.Element("arg", {"name": "hook_sel",
                                                                   "value": "apad" + str(first_index_pad + i + 1) +
                                                                            "/uwsim_hook"}))

            if outputToScreen:
                group.append(xml.etree.ElementTree.Element("node", {"pkg": "apad", "type":controllerFile,
                                                                    "name": "scenario_controller", "output": "screen"}))
                group.append(xml.etree.ElementTree.Element("node", {"pkg": "apad", "type": "action_server.py",
                                                                    "name": "action_server", "output": "screen"}))
            else:
                group.append(xml.etree.ElementTree.Element("node", {"pkg": "apad", "type": controllerFile,
                                                                    "name": "scenario_controller"}))
                group.append(xml.etree.ElementTree.Element("node", {"pkg": "apad", "type": "action_server.py",
                                                                    "name": "action_server"}))

            root.append(group)

    if "aFish" in agents:

        for i in range(n_fish):
            name = 'afish' + str(first_index_fish + i + 1)
            group = xml.etree.ElementTree.Element("group", {"ns": name})
            
            # Load parameters
            group.append(xml.etree.ElementTree.Element("rosparam", {"file": "$(find afish)/data/control/params/"
                                                                            "controllers_standard.yaml"}))
            group.append(xml.etree.ElementTree.Element("rosparam", {"file": "$(find afish)/data/dynamics/afish.yaml"}))
            group.append(xml.etree.ElementTree.Element("rosparam", {"param": "eta0"}))
            group[-1].text = "[" + str(positions_fish[i].north) + "," + str(positions_fish[i].east) + "," + \
                             str(positions_fish[i].depth) + ",0,0,0]"

            # Location parameters
            group.append(xml.etree.ElementTree.Element("rosparam", {"command": "load",
                                                                    "file": "$(find afish)/data/locations/"
                                                                            "swarm_loc.yaml"}))
            
            # Load the simulation
            group.append(xml.etree.ElementTree.Element("include", {"file": "$(find afish)/data/simulation/scenario/" +
                                                                           simulationSpecFile}))
            group[-1].append(xml.etree.ElementTree.Element("arg", {"name": "battery_level",
                                                                   "value": str(battery_fish[i])}))

            # Load the controllers
            group.append(xml.etree.ElementTree.Element("include", {"file": "$(find afish)/data/control/"
                                                                           "control_standard.xml"}))

            # Load visualization
            group.append(xml.etree.ElementTree.Element("include", {"file": "$(find afish)/data/simulation/"
                                                                           "visualization_standard.xml"}))
            group[-1].append(xml.etree.ElementTree.Element("arg", {"name": "hook_sel",
                                                                   "value": "afish" + str(first_index_fish + i + 1) +
                                                                            "/uwsim_hook"}))

            if outputToScreen:
                group.append(xml.etree.ElementTree.Element("node", {"pkg": "afish", "type": controllerFile,
                                                                    "name": "scenario_controller", "output": "screen"}))
            else:
                group.append(xml.etree.ElementTree.Element("node", {"pkg": "afish", "type": controllerFile,
                                                                    "name": "scenario_controller"}))
        root.append(group)

    if "aMussel" in agents:

        for i in range(n_mussel):
            name = 'amussel' + str(first_index_mussel + i + 1)
            group = xml.etree.ElementTree.Element("group", {"ns": name})
            
            # Load parameters
            group.append(xml.etree.ElementTree.Element("rosparam", {"file": "$(find amussel)/data/control/params/"
                                                                            "controllers_standard.yaml"}))
            group.append(xml.etree.ElementTree.Element("rosparam", {"file": "$(find amussel)/data/dynamics/"
                                                                            "amussel.yaml"}))
            group.append(xml.etree.ElementTree.Element("rosparam", {"param": "eta0"}))

            group[-1].text = "[" + str(positions_mussel[i].north) + "," + str(positions_mussel[i].east) + "," + \
                             str(positions_mussel[i].depth) + ",0,0,0]"

            # Location parameters
            group.append(xml.etree.ElementTree.Element("rosparam", {"command": "load", "file": "$(find amussel)/data/"
                                                                                               "locations/"
                                                                                               "swarm_loc.yaml"}))
            
            # Load the simulation
            group.append(xml.etree.ElementTree.Element("include", {"file": "$(find amussel)/data/simulation/"
                                                                           "scenario/" + simulationSpecFile}))
            group[-1].append(xml.etree.ElementTree.Element("arg", {"name": "battery_level",
                                                                   "value": str(battery_mussel[i])}))

            # Load the controllers
            group.append(xml.etree.ElementTree.Element("include", {"file": "$(find amussel)/data/control/"
                                                                           "control_standard.xml"}))
            
            # Load visualization
            group.append(xml.etree.ElementTree.Element("include", {"file": "$(find amussel)/data/simulation/"
                                                                           "visualization_standard.xml"}))
            group[-1].append(xml.etree.ElementTree.Element("arg", {"name": "hook_sel",
                                                                   "value": "amussel" + str(first_index_mussel + i + 1)
                                                                            + "/uwsim_hook"}))

            if outputToScreen:
                group.append(xml.etree.ElementTree.Element("node", {"pkg": "amussel", "type": controllerFile,
                                                                    "name": "scenario_controller", "output": "screen"}))
            else:
                group.append(xml.etree.ElementTree.Element("node", {"pkg": "amussel", "type": controllerFile,
                                                                    "name": "scenario_controller"}))
            
            root.append(group)  
       
if __name__ == "__main__":
    
    if len(sys.argv) < 4:
        print "USAGE: python setup.py scenario_name apad_number afish_number amussel_number [first_index_apad] [" \
              "first_index_afish] [first_index_amussel] \n [north_min] [north_max] [east_min] [east_max]"
        sys.exit(0)
        
    scenario = sys.argv[1]
    
    apad_number = int(sys.argv[2])
    north_range_pad = [-100, 100]
    east_range_pad = [-100, 100]
    apad_first_index = 0

    afish_number = int(sys.argv[3])
    north_range_fish = [-100, 100]
    east_range_fish = [-100, 100]
    afish_first_index = 0
    
    amussel_number = int(sys.argv[4])
    north_range_mussel = [-100, 100]
    east_range_mussel = [-100, 100]
    amussel_first_index = 0
    
    if len(sys.argv) >= 12:
        east_range_pad[1] = east_range_fish[1] = east_range_mussel[1] = float(sys.argv[11])
    if len(sys.argv) >= 11:
        east_range_pad[0] = east_range_fish[0] = east_range_mussel[0] = float(sys.argv[10])
    if len(sys.argv) >= 10:
        north_range_pad[1] = north_range_fish[1] = north_range_mussel[1] = float(sys.argv[9])
    if len(sys.argv) >= 9:
        north_range_pad[0] = north_range_fish[0] = north_range_mussel[0] = float(sys.argv[8])

    if len(sys.argv) >= 8:
        amussel_first_index = int(sys.argv[7])
    if len(sys.argv) >= 7:
        afish_first_index = int(sys.argv[6])
    if len(sys.argv) >= 6:
        apad_first_index = int(sys.argv[5])

    rospack = rospkg.RosPack()
    
    scenarioSpec = xml.etree.ElementTree.parse(rospack.get_path('subcultron_launch') +
                                               '/data/scenario/scenario_spec.xml').getroot()
    
    for child in scenarioSpec:
        if child.find('name').text == scenario:
            controllerFile = child.find('controllerFile').text
            simulationSpecFile = child.find('simulationXmlFile').text
            for ag in child.findall('agent'):
                agents.append(ag.text) 
                
    print ""    
    print "aPad number"
    if "aPad" in agents:
        print apad_number
        print "north range"
        print north_range_pad
        print "east range"
        print east_range_pad
        print "first index"
        print apad_first_index
    else:
        print "0"
        apad_number = 0
        
    print ""
    print ""    
    print "aFish number"
    if "aFish" in agents:
        print afish_number
        print "north range"
        print north_range_fish
        print "east range"
        print east_range_fish
        print "first index"
        print afish_first_index
    else:
        print "0"
        afish_number = 0
    
    print ""
    print ""    
    print "aMussel number"
    if "aMussel" in agents:
        print amussel_number
        print "north range"
        print north_range_mussel
        print "east range"
        print east_range_mussel
        print "first index"
        print amussel_first_index
    else:
        print "0" 
        amussel_number = 0
    print ""
    
    print "Scenario: " + scenario

    # generate random positions
    apad_positions = []
    posID_pad = []

    afish_positions = []
    posID_fish = []
       
    amussel_positions = []
    posID_mussel = []
    
    while len(apad_positions) < apad_number:
        north = uniform(north_range_pad[0], north_range_pad[1])
        east = uniform(east_range_pad[0], east_range_pad[1])
        tmp = '%.2f%.2f' % (north, east)
        if tmp not in posID_pad:
            posID_pad.append(tmp)
            apad_positions.append(NED(north, east, 0))
    
    while len(afish_positions) < afish_number:
        north = uniform(north_range_fish[0], north_range_fish[1])
        east = uniform(east_range_fish[0], east_range_fish[1])
        tmp = '%.2f%.2f' % (north, east)
        if tmp not in posID_pad:
            posID_pad.append(tmp)
            afish_positions.append(NED(north, east, 0))

    while len(amussel_positions) < amussel_number:
        north = uniform(north_range_mussel[0], north_range_mussel[1])
        east = uniform(east_range_mussel[0], east_range_mussel[1])
        tmp = '%.2f%.2f' % (north, east)
        if tmp not in posID_mussel:
            posID_mussel.append(tmp)
            amussel_positions.append(NED(north, east, 0))
    
    apad_battery = []
    afish_battery = []
    amussel_battery = []

    while len(apad_battery) < apad_number:
        apad_battery.append(uniform(70, 100))

    while len(afish_battery) < afish_number:
        afish_battery.append(uniform(40, 100))

    while len(amussel_battery) < amussel_number:
        amussel_battery.append(uniform(5, 60))

    print "battery aPad " + str(apad_battery)
    print "battery aFish " + str(afish_battery)
    print "battery aMussel " + str(amussel_battery)

    # write into scene specification file (swarm_test.xml)
    fileOut = open(rospack.get_path('subcultron_launch') + '/data/simulation/' + sceneSpecFile, 'w')
   
    # remove file content
    fileOut.seek(0)
    fileOut.truncate()
    fileOut.write("<?xml version=\"1.0\"?>\n")
    fileOut.write("<!DOCTYPE UWSimScene SYSTEM \"UWSimScene.dtd\" >\n")
    
    tree = xml.etree.ElementTree.parse(rospack.get_path('subcultron_launch') + '/data/simulation/' + sceneSpecTemplate)
    xml_tree_root = tree.getroot()
    
    fill_up_simulation_spec_file(xml_tree_root, apad_number, apad_positions, apad_first_index, afish_number,
                                 afish_positions, afish_first_index, amussel_number, amussel_positions,
                                 amussel_first_index)
    
    indent(xml_tree_root)
    tree.write(fileOut)
    
    fileOut.close()
    
    # write into launch file
    fileOut = open(rospack.get_path('subcultron_launch') + '/launch/simulation/' + launchFile, 'w')
   
    # remove file content
    fileOut.seek(0)
    fileOut.truncate()
       
    tree = xml.etree.ElementTree.parse(rospack.get_path('subcultron_launch') + '/launch/simulation/' +
                                       launchFileTemplate)
    xml_tree_root = tree.getroot()
    
    fill_up_launch_file(xml_tree_root, apad_number, apad_positions, apad_first_index, apad_battery, afish_number,
                        afish_positions, afish_first_index, afish_battery, amussel_number, amussel_positions,
                        amussel_first_index, amussel_battery)
    
    indent(xml_tree_root)
    tree.write(fileOut)
     
    fileOut.close()
