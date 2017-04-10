# Standard message header
Header header

# The modem data to transmit
string message

# The acoustic transmission duration 
# This is hardware specific, generally =(nbits/bps) in seconds
float64 duration

# The acoustic transmission strength
# This is hardware specific
float64 gain

# Sender into medium
# This is used to identify data 
# sender on the same topic
int32 sender

# Intended receiver 
# This is used for data coming from
# the medium to filter messages on 
# the same topic
int32 receiver

# The acoustic strength at the receiver level
# This is medium specific attenuation
float64 snr

# Position of the sender
auv_msgs/NavSts position

# Range to the sender
float64 range
# Bearing towards the sender
float64 azimuth
# Elevation towards the sender
float64 elevation  

# The node_id that hears this transmission
int32 listener_id

# The conflict flag if two receptions occured at the same time
bool conflict