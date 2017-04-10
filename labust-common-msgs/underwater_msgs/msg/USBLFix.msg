# The USBL fix message
# Valid range only
int32 RANGE_ONLY = 0
# Valid azimuth
int32 AZIMUTH_ONLY = 1
# Valid position (usually assumes valid range and azimuth)
int32 FULL_FIX = 2

# Header information
Header header

# Beacon ID
int32 beacon
# Fix type
int32 type

# Relative position in NED
geometry_msgs/Point relative_position
# Absolute position + speed/etc. estimates
auv_msgs/NavSts position

# Range to target
float32 range
# Bearing to target
float32 bearing
# Elevation to target
float32 elevation
# Sound speed use for calculation
float32 sound_speed


