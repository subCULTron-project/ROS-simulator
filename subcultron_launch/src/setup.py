#!/usr/bin/env python

"""!
@package Initializes ROS node for high level mission control.
@author UNIZG-FER <barbara.arbanas@fer.hr>
"""

import sys
import rospkg
import getopt

from copy import deepcopy
from random import uniform
from auv_msgs.msg import NED
import xml.etree.ElementTree

rospack = rospkg.RosPack()

########################################

sceneSpecTemplate = "swarm_test_raw.xml"
sceneSpecFile = "swarm_test.xml"

launchFileTemplate = "standard_simulation_raw.launch"
launchFile = "standard_simulation.launch"

########################################

controller_file_aPad = []
controller_file_aFish = []
controller_file_aMussel = []

simulation_spec_file_aPad = []
simulation_spec_file_aFish = []
simulation_spec_file_aMussel = []

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
        name = "apad" + str(first_index_pad + i)
        tmp.find("name").text = name
        tmp.find("imu").find("name").text = "imu" + str(first_index_pad + i)
        tmp.find("gpsSensor").find("name").text = "GPSSensor" + str(first_index_pad + i)
        root.insert(size - 1, tmp)    

    vehicle_root = xml.etree.ElementTree.parse(rospack.get_path('afish') + '/data/simulation/' + 'vehicle.xml').getroot()
    size = len(root)
    
    for i in range(n_fish):
        tmp = deepcopy(vehicle_root)
        name = "afish" + str(first_index_fish + i)
        tmp.find("name").text = name
        tmp.find("imu").find("name").text = "imu" + str(first_index_fish + i)
        tmp.find("gpsSensor").find("name").text = "GPSSensor" + str(first_index_fish + i)
        root.insert(size - 1, tmp)  
    
    vehicle_root = xml.etree.ElementTree.parse(rospack.get_path('amussel') + '/data/simulation/' + 'vehicle.xml').getroot()

    for i in range(n_mussel):
        tmp = deepcopy(vehicle_root)
        name = "amussel" + str(first_index_mussel + i)
        tmp.find("name").text = name
        tmp.find("imu").find("name").text = "imu" + str(first_index_mussel + i)
        tmp.find("gpsSensor").find("name").text = "GPSSensor" + str(first_index_mussel + i)
        root.insert(size - 1, tmp)    

    root.append(xml.etree.ElementTree.Element("rosInterfaces"))    
    for i in range(n_pad):
        name = "apad" + str(first_index_pad + i)
        tmp = xml.etree.ElementTree.Element("ROSOdomToPAT")
        tmp.append(xml.etree.ElementTree.Element("topic"))
        tmp[-1].text = name + "/uwsim_hook"
        
        tmp.append(xml.etree.ElementTree.Element("vehicleName"))
        tmp[-1].text = name
        
        root.find("rosInterfaces").append(tmp)

    for i in range(n_fish):
        name = "afish" + str(first_index_fish + i)
        tmp = xml.etree.ElementTree.Element("ROSOdomToPAT")
        tmp.append(xml.etree.ElementTree.Element("topic"))
        tmp[-1].text = name + "/uwsim_hook"
        
        tmp.append(xml.etree.ElementTree.Element("vehicleName"))
        tmp[-1].text = name
        
        root.find("rosInterfaces").append(tmp)
          
    for i in range(n_mussel):
        name = "amussel" + str(first_index_mussel + i)
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

    for i in range(n_pad):
        name = 'apad' + str(first_index_pad + i)
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
        group.append(xml.etree.ElementTree.Element("include", {"file": "$(find " + simulation_spec_file_aPad[0] + ")" +
                                                   simulation_spec_file_aPad[1]}))
        group[-1].append(xml.etree.ElementTree.Element("arg", {"name": "battery_level",
                                                               "value": str(battery_pad[i])}))

        # Load the controllers
        group.append(xml.etree.ElementTree.Element("include", {"file": "$(find apad)/data/control/"
                                                                       "control_standard.xml"}))

        # Load visualization
        group.append(xml.etree.ElementTree.Element("include", {"file": "$(find apad)/data/simulation/"
                                                                       "visualization_standard.xml"}))
        group[-1].append(xml.etree.ElementTree.Element("arg", {"name": "hook_sel",
                                                               "value": "apad" + str(first_index_pad + i) +
                                                                        "/uwsim_hook"}))

        if outputToScreen:
            group.append(xml.etree.ElementTree.Element("node", {"pkg": controller_file_aPad[0],
                                                                "type": controller_file_aPad[1],
                                                                "name": "scenario_controller", "output": "screen"}))
            group.append(xml.etree.ElementTree.Element("node", {"pkg": "apad", "type": "action_server.py",
                                                                "name": "action_server", "output": "screen"}))
        else:
            group.append(xml.etree.ElementTree.Element("node", {"pkg": controller_file_aPad[0],
                                                                "type": controller_file_aPad[1],
                                                                "name": "scenario_controller"}))
            group.append(xml.etree.ElementTree.Element("node", {"pkg": "apad", "type": "action_server.py",
                                                                "name": "action_server"}))

        root.append(group)

    for i in range(n_fish):
        name = 'afish' + str(first_index_fish + i)
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
        group.append(xml.etree.ElementTree.Element("include", {"file": "$(find " + simulation_spec_file_aFish[0] + ")" +
                                                   simulation_spec_file_aFish[1]}))
        group[-1].append(xml.etree.ElementTree.Element("arg", {"name": "battery_level",
                                                               "value": str(battery_fish[i])}))

        # Load the controllers
        group.append(xml.etree.ElementTree.Element("include", {"file": "$(find afish)/data/control/"
                                                                       "control_standard.xml"}))

        # Load visualization
        group.append(xml.etree.ElementTree.Element("include", {"file": "$(find afish)/data/simulation/"
                                                                       "visualization_standard.xml"}))
        group[-1].append(xml.etree.ElementTree.Element("arg", {"name": "hook_sel",
                                                               "value": "afish" + str(first_index_fish + i) +
                                                                        "/uwsim_hook"}))

        if outputToScreen:
            group.append(xml.etree.ElementTree.Element("node", {"pkg": controller_file_aFish[0],
                                                                "type": controller_file_aFish[1],
                                                                "name": "scenario_controller", "output": "screen"}))
        else:
            group.append(xml.etree.ElementTree.Element("node", {"pkg": controller_file_aFish[0],
                                                                "type": controller_file_aFish[1],
                                                                "name": "scenario_controller"}))
    root.append(group)

    for i in range(n_mussel):
        name = 'amussel' + str(first_index_mussel + i)
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
        group.append(xml.etree.ElementTree.Element("include", {"file": "$(find " + simulation_spec_file_aMussel[0] +
                                                                       ")" + simulation_spec_file_aMussel[1]}))
        group[-1].append(xml.etree.ElementTree.Element("arg", {"name": "battery_level",
                                                               "value": str(battery_mussel[i])}))

        # Load the controllers
        group.append(xml.etree.ElementTree.Element("include", {"file": "$(find amussel)/data/control/"
                                                                       "control_standard.xml"}))

        # Load visualization
        group.append(xml.etree.ElementTree.Element("include", {"file": "$(find amussel)/data/simulation/"
                                                                       "visualization_standard.xml"}))
        group[-1].append(xml.etree.ElementTree.Element("arg", {"name": "hook_sel",
                                                               "value": "amussel" + str(first_index_mussel + i)
                                                                        + "/uwsim_hook"}))

        if outputToScreen:
            group.append(xml.etree.ElementTree.Element("node", {"pkg": controller_file_aMussel[0],
                                                                "type": controller_file_aMussel[1],
                                                                "name": "scenario_controller", "output": "screen"}))
        else:
            group.append(xml.etree.ElementTree.Element("node", {"pkg": controller_file_aMussel[0],
                                                                "type": controller_file_aMussel[1],
                                                                "name": "scenario_controller"}))

        root.append(group)
       
if __name__ == "__main__":

    config_file = ''
    err = 0
    try:
        opts, args = getopt.getopt(sys.argv[1:], "c:", ["config-file="])
        if len(opts) != 1:
            err = 1
    except getopt.GetoptError:
        err = 1

    if err == 0:
        for opt, arg in opts:
            if opt in ('-c', "--config-file"):
                config_file = arg
                print "\nReading from config file '" + config_file + "' ...\n"

    if len(config_file) == 0 or err:
        print "USAGE: python setup.py -c <config_file_path>"
        print "or     python setup.py --config-file=<config_file_path>"
        sys.exit(2)

    config = xml.etree.ElementTree.parse(config_file).getroot()

    for child in config:
        if child.tag == 'vehicles':

            if child.find('aPadNumber') is None:
                apad_number = 0
            else:
                apad_number = int(child.find('aPadNumber').text)
            if child.find('aFishNumber') is None:
                afish_number = 0
            else:
                afish_number = int(child.find('aFishNumber').text)
            if child.find('aMusselNumber') is None:
                amussel_number = 0
            else:
                amussel_number = int(child.find('aMusselNumber').text)

            start_indices = child.find('startIndices')
            if start_indices is None:
                apad_first_index = 0
                afish_first_index = 0
                amussel_first_index = 0
            else:
                if start_indices.find('aPad') is None:
                    apad_first_index = 1
                else:
                    apad_first_index = int(start_indices.find('aPad').text)
                    if apad_first_index < 0:
                        print "aPad starting index should be >= 0!"
                        sys.exit(2)

                if start_indices.find('aFish') is None:
                    afish_first_index = 1
                else:
                    afish_first_index = int(start_indices.find('aFish').text)
                    if afish_first_index < 0:
                        print "aFish starting index should be >= 0!"
                        sys.exit(2)

                if start_indices.find('aMussel') is None:
                    amussel_first_index = 1
                else:
                    amussel_first_index = int(start_indices.find('aMussel').text)
                    if amussel_first_index < 0:
                        print "aMussel starting index should be >= 0!"
                        sys.exit(2)

            position_file = 0
            apad_positions = []
            afish_positions = []
            amussel_positions = []

            if child.find('positions') is None:
                north_range_pad = [-100, 100]
                east_range_pad = [-100, 100]

                north_range_fish = [-100, 100]
                east_range_fish = [-100, 100]

                north_range_mussel = [-100, 100]
                east_range_mussel = [-100, 100]

                # generate random positions
                posID_pad = []
                posID_fish = []
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
            else:
                position_file = 1
                file_path = child.find('positions').text
                f = open(file_path)
                lines = f.readlines()
                f.close()
                if len(lines[0].split(' ')) != 3:
                    print "error in position file definition!"
                    sys.exit(2)
                pad_num = int(lines[0].split(' ')[0])
                fish_num = int(lines[0].split(' ')[1])
                mussel_num = int(lines[0].split(' ')[2])

                if pad_num != apad_number or fish_num != afish_number or mussel_num != amussel_number:
                    print "error in position file definition, wrong number of robots defined in line 1!"
                    sys.exit(2)

                if len(lines[1:]) < apad_number + afish_number + amussel_number:
                    print "error in position file definition, too few position entries!"
                    sys.exit(2)

                for line in lines[1:]:
                    # skip empty lines (only newline)
                    if len(line) <= 1:
                        continue

                    pos = line.split(' ')
                    if len(pos) != 2:
                        print "error in position file definition, not enough parameters!"
                        print line
                        sys.exit(2)
                    if len(apad_positions) < apad_number:
                        apad_positions.append(NED(float(pos[0]), float(pos[1]), 0))
                    elif len(afish_positions) < afish_number:
                        afish_positions.append(NED(float(pos[0]), float(pos[1]), 0))
                    elif len(amussel_positions) < amussel_number:
                        amussel_positions.append(NED(float(pos[0]), float(pos[1]), 0))

                if len(apad_positions) < apad_number or len(afish_positions) < afish_number or len(amussel_positions) \
                        < amussel_number:
                    print "error in position file definition, too few position entries!"
                    sys.exit(2)

        battery_file = 0
        apad_battery = []
        afish_battery = []
        amussel_battery = []

        if child.find('battery') is None:

            # generate random battery states
            while len(apad_battery) < apad_number:
                apad_battery.append(uniform(5, 100))

            while len(afish_battery) < afish_number:
                afish_battery.append(uniform(5, 100))

            while len(amussel_battery) < amussel_number:
                amussel_battery.append(uniform(5, 100))

        else:
            file_path = child.find('battery').text
            f = open(file_path)
            lines = f.readlines()
            f.close()
            if len(lines[0].split(' ')) != 3:
                print "error in battery file definition!"
                sys.exit(2)
            pad_num = int(lines[0].split(' ')[0])
            fish_num = int(lines[0].split(' ')[1])
            mussel_num = int(lines[0].split(' ')[2])

            if pad_num != apad_number or fish_num != afish_number or mussel_num != amussel_number:
                print "error in battery file definition, wrong number of robots defined in line 1!"
                sys.exit(2)

            if len(lines[1:]) < apad_number + afish_number + amussel_number:
                print "error in battery file definition, too few position entries!"
                sys.exit(2)

            for line in lines[1:]:
                # skip empty lines (only newline)
                if len(line) <= 1:
                    continue

                bat = line.split(' ')
                if len(apad_battery) < apad_number:
                    apad_battery.append(float(bat[0]))
                elif len(afish_battery) < afish_number:
                    afish_battery.append(float(bat[0]))
                elif len(amussel_battery) < amussel_number:
                    amussel_battery.append(float(bat[0]))

            if len(apad_battery) < apad_number or len(afish_battery) < afish_number or len(amussel_battery) \
                    < amussel_number:
                print "error in battery file definition, too few position entries!"
                sys.exit(2)

            if max(apad_battery) > 100 or max(afish_battery) > 100 or max(amussel_battery) > 100 or min(apad_battery) \
                    < 0 or min(afish_battery) < 0 or min(amussel_battery) < 0:
                print "error in battery file definition, values out of range [0,100]!"
                sys.exit(2)

        print ""
        print "aPad number"
        print apad_number
        if apad_number > 0:
            if position_file == 0:
                print "north range"
                print north_range_pad
                print "east range"
                print east_range_pad
            print "first index"
            print apad_first_index

        print ""
        print ""
        print "aFish number"
        print afish_number
        if afish_number > 0:
            if position_file == 0:
                print "north range"
                print north_range_fish
                print "east range"
                print east_range_fish
            print "first index"
            print afish_first_index

        print ""
        print ""
        print "aMussel number"
        print amussel_number
        if amussel_number > 0:
            if position_file == 0:
                print "north range"
                print north_range_mussel
                print "east range"
                print east_range_mussel
            print "first index"
            print amussel_first_index
        print ""
        print "battery states"
        print apad_battery
        print afish_battery
        print amussel_battery

        # write into scene specification file (swarm_test.xml)
        fileOut = open(rospack.get_path('subcultron_launch') + '/data/simulation/' + sceneSpecFile, 'w')

        # remove file content
        fileOut.seek(0)
        fileOut.truncate()
        fileOut.write("<?xml version=\"1.0\"?>\n")
        fileOut.write("<!DOCTYPE UWSimScene SYSTEM \"UWSimScene.dtd\" >\n")

        tree = xml.etree.ElementTree.parse(
            rospack.get_path('subcultron_launch') + '/data/simulation/' + sceneSpecTemplate)
        xml_tree_root = tree.getroot()

        fill_up_simulation_spec_file(xml_tree_root, apad_number, apad_first_index, afish_number, afish_first_index,
                                     amussel_number, amussel_first_index)

        indent(xml_tree_root)
        tree.write(fileOut)

        fileOut.close()

        if child.find('simulation_spec_apad') is None and apad_number > 0:
            print "no simulation specification file defined for aPad!"
            sys.exit(2)
        else:
            tmp = child.find('simulation_spec_apad')
            simulation_spec_file_aPad.append(tmp.find('pkg').text)
            simulation_spec_file_aPad.append(tmp.find('path').text)

        if child.find('simulation_spec_afish') is None and afish_number > 0:
            print "no simulation specification file defined for aFish!"
            sys.exit(2)
        else:
            tmp = child.find('simulation_spec_afish')
            simulation_spec_file_aFish.append(tmp.find('pkg').text)
            simulation_spec_file_aFish.append(tmp.find('path').text)

        if child.find('simulation_spec_amussel') is None and amussel_number > 0:
            print "no simulation specification file defined for aMussel!"
            sys.exit(2)
        else:
            tmp = child.find('simulation_spec_amussel')
            simulation_spec_file_aMussel.append(tmp.find('pkg').text)
            simulation_spec_file_aMussel.append(tmp.find('path').text)

        if child.find('controller_apad') is None and apad_number > 0:
            print "no simulation controller defined for aPad!"
            sys.exit(2)
        else:
            tmp = child.find('controller_apad')
            controller_file_aPad.append(tmp.find('pkg').text)
            controller_file_aPad.append(tmp.find('fname').text)

        if child.find('controller_afish') is None and afish_number > 0:
            print "no simulation controller defined for aFish!"
            sys.exit(2)
        else:
            tmp = child.find('controller_afish')
            controller_file_aFish.append(tmp.find('pkg').text)
            controller_file_aFish.append(tmp.find('fname').text)

        if child.find('controller_amussel') is None and amussel_number > 0:
            print "no simulation controller defined for aMussel!"
            sys.exit(2)
        else:
            tmp = child.find('controller_amussel')
            controller_file_aMussel.append(tmp.find('pkg').text)
            controller_file_aMussel.append(tmp.find('fname').text)

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