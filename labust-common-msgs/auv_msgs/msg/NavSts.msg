# Sent by the navigator at 5-10 Hz.

Header 	header

DecimalLatLon global_position
DecimalLatLon origin

# NED position and altitude in metres. 
NED position
float32 altitude

# Body velocities in metres/sec.
geometry_msgs/Point body_velocity
# Body velocities relative to ground in metres/sec.
geometry_msgs/Point gbody_velocity

# Orientation and orientation rate are in radians and radians/sec
RPY orientation
RPY orientation_rate

# Variances for position and orientation
NED position_variance
RPY orientation_variance

# Status and associated constants (flags)
uint8 status
uint8 STATUS_FAULT = 0
uint8 STATUS_LOCAL_FRAME_OK = 1
uint8 STATUS_GLOBAL_FRAME_OK = 2
uint8 STATUS_POSITION_OK = 3
uint8 STATUS_ESTIMATION_ERROR_OK  = 8
uint8 STATUS_WATER_VELOCITY_OK = 16
uint8 STATUS_GROUND_VELOCITY_OK = 32
uint8 STATUS_VELOCITY_OK = 48
uint8 STATUS_ALL_OK = 59