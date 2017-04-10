# The controller must not publish anything when disabled
uint8 DISABLED = 0
# The controller takes the manual reference input 
uint8 MANUAL = 1
# The controller takes the higher level control input
uint8 EXTERNAL = 2
# The controller has passive tracking of output - preferred over DISABLE during idle operation
uint8 TRACKING = 3

# Topic names to which to subscribe on change
string manual_topic
string external_topic
string state_topic
string tracking_topic

# Desired controller state
uint8 state

