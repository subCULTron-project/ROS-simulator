# The device sends data and performs ranging
# The device received some data
int32 RECEIVED = 0
# The devices performs ranging with sending data
int32 RANGING_DATA = 1
# The device sends data without ranging
int32 SEND_DATA = 2
# The device adds data in the buffer for auto-reply
int32 SET_REPLY = 3
# The device performs only ranging
int32 RANGING = 4

# Data header
Header header
# Message priority (Note: SET_REPLY actions are executed immediately)
uint8 priority
# Modem payload data
uint8[] payload
# Payload bits - if not divided by 8 the last n < 8 bits 
# for the last byte will be extracted (Note: not used by Seatrac)
int32 nbits
# Desired action
int32 action
# Acoustic receiver (Note: put 0 for broadcasts)
int32 receiver
# Acoustic sender
int32 sender

