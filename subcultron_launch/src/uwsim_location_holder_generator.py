
def generate(pad, fish, mussel):
    
    file = open("uwsim_location_holder.py", 'w')
    
    file.write("#!/usr/bin/python\n\"\"\"\n\"\"\"\n\n")
    file.write("__author__ = \"barbanas\"\n")
    file.write("from geometry_msgs.msg import Point\n")
    file.write("from auv_msgs.msg import NavSts\n")
    file.write("from misc_msgs.srv import GetTrustInfo, GetChargeInfo\n")
    file.write("from std_srvs.srv import Empty\n")
    file.write("import numpy as np\n")
    file.write("from math import sqrt, pow\n")
    file.write("import os\n")
    file.write("import rospy\n\n")
    
    file.write(
    "class UWSimLocationHolder(object):\n\n" +
    "    def __init__(self):\n\n")
    
    file.write("        self.communicationRange = 5\n\n")
    file.write("        self.namespaces = []\n")
    file.write("        self.index = 0\n\n")

    file.write("        self.connMatrix = np.zeros([" + str(fish + mussel) + ","  + str(fish + mussel) + "])\n")
    file.write("        self.positions = []\n")
    file.write("        for i in range(" + str(fish + mussel) + "):\n" +
               "            self.positions.append(Point())\n")
    file.write("\n")
    for i in range(fish):
        file.write("        self.afish" + str(i + 1) + "locSub = rospy.Subscriber('/afish" + str(i + 1) + "/position', NavSts, self.afish" + str(i + 1) + "position_cb)\n")
        file.write("        self.namespaces.append('/afish" + str(i + 1) + "/')\n")
    file.write("\n")
    for i in range(mussel):
        file.write("        self.amussel" + str(i + 1) + "locSub = rospy.Subscriber('/amussel" + str(i + 1) + "/position', NavSts, self.amussel" + str(i + 1) + "position_cb)\n")
        file.write("        self.namespaces.append('/amussel" + str(i + 1) + "/')\n")
    file.write("\n")

    for i in range(pad):
        file.write("        self.namespaces.append('/apad" + str(i + 1) + "/')\n")
    file.write("\n")
    
    file.write("        rospy.Timer(rospy.Duration(0.2), self.calculate_connectivity_matrix)\n\n")
    
    file.write("        rospy.Service('get_connectivity_vectors', GetTrustInfo, self.get_conn_vectors_srv)\n\n")

    file.write("        rospy.Service('log_charge_info', Empty, self.log_charge_info_srv)\n\n")
    
    file.write("        rospy.spin()\n\n")
    # functions
    
    for i in range(fish):
        file.write("    def afish" + str(i + 1) + "position_cb(self, msg):\n" +
                   "        self.positions[" + str(i) + "].x = msg.position.north\n" +
                   "        self.positions[" + str(i) + "].y = msg.position.east\n" +
                   "        self.positions[" + str(i) + "].z = msg.position.depth\n" 
                   )
        file.write("\n")
    file.write("\n")
    
    for i in range(mussel):
        file.write("    def amussel" + str(i + 1) + "position_cb(self, msg):\n" +
                   "        self.positions[" + str(fish + i) + "].x = msg.position.north\n" +
                   "        self.positions[" + str(fish + i) + "].y = msg.position.east\n" +
                   "        self.positions[" + str(fish + i) + "].z = msg.position.depth\n" 
                   )
        file.write("\n")
    file.write("\n")
    
    file.write("    def calculate_connectivity_matrix(self, event):\n" +
               "        for i in range(" + str(fish + mussel) + "):\n" +
               "            for j in range(i + 1, " + str(fish + mussel) + "):\n" +
               "                if self.distance(self.positions[i], self.positions[j]) <= self.communicationRange:\n" +
               "                    self.connMatrix[i][j] = self.connMatrix[j][i] = 1\n" +
               "                else:\n" +
               "                    self.connMatrix[i][j] = self.connMatrix[j][i] = 0\n" 
                   )
    file.write("\n")
    
    file.write("    def distance(self, p1, p2):\n" +
               "        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2))\n")
    file.write("\n")
    
    file.write("# service functions\n")
    
    
    file.write("    def get_conn_vectors_srv(self, req):\n" +
               "        if req.index >= " + str(fish) + ":\n" +
               "            return {}\n" +
               "        return {'A': self.connMatrix[req.index][:" + str(fish) + "], 'b': self.connMatrix[req.index][" + str(fish) + ":] }\n")
    file.write("\n")

    file.write("    def log_charge_info_srv(self, req):\n\n" +
               "        username = os.environ[\"USER\"]\n" +
               "        print os.path.join(\"/home/\", username, \"Desktop/cbm_subc\", \"state\" + str(self.index))\n" +
               "        f = open(os.path.join(\"/home/\", username, \"Desktop/cbm_subc\", \"state\" + str(self.index)), 'w')\n" +
               "        self.index += 1\n\n" +
               "        for ns in self.namespaces:\n" +
               "            charge_info = rospy.ServiceProxy(ns + 'get_charge_info', GetChargeInfo)\n" +
               "            try:\n" +
               "                c = charge_info()\n" +
               "                f.write(ns + \"\\n\")\n" +
               "                f.write(str(c.battery_level) + \"\\n\")\n" + 
               "                f.write(str(c.x) + ' ' + str(c.y) + \"\\n\")\n" + 
               "                f.write(c.type + \"\\n\")\n" + 
               "            except rospy.ServiceException as exc:\n" +
               "                print(\"Service did not process request: \" + str(exc))\n" +
               "        f.close()\n" +
               "        return True\n")
    file.write("\n")
    
    file.write("if __name__ == \"__main__\":\n\n" +
    "    rospy.init_node(\"uwsim_location_holder\")\n" +
    "    try:\n" +
    "        controller = UWSimLocationHolder()\n" +
    "    except rospy.ROSInterruptException:\n" +
    "        pass")