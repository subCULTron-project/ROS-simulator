uint8 Surge=0
uint8 Sway=1
uint8 Heave=2
uint8 Roll=3
uint8 Pitch=4
uint8 Yaw=5

uint8 X=0
uint8 Y=1
uint8 Z=2
uint8 K=3
uint8 M=4
uint8 N=5

#Controller parent name - for multi-controller nodes
string parent
#Controller unique name
string name
#Used tau
int8[6] used_tau
#Used velocity
int8[6] used_nu