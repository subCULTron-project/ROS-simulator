#Parsing options

uint8 method # Mission file passing method
uint8 FILENAME = 0
uint8 STRING = 1

string missionData# mission xml file name or xml string

# Mission options
bool relative # Start mission from given or current position if true
auv_msgs/NED startPosition # Start position in NED frame


 

