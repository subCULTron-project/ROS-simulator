#The available modes
int32 Stop=0
int32 Manual=1
int32 GoToPoint=2
int32 StationKeeping=3
int32 CirclePoint=4
int32 HeadingControl=5
int32 HeadingSurge=6

int32 mode
geometry_msgs/PointStamped ref_point
#Circle info
float32 radius
float32 surge
float32 yaw
