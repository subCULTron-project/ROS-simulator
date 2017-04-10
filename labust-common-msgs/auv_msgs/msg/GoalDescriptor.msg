# Contains values common to all waypoint request and status messages.
# Waypoint goals with higher priority trump lower priority goals. For
# goals of the same priority, the pilot should follow the latest
# received.

# Name of the requesting node, used with id to identify the request.
string requester

# Goal ID for the request
uint32 id

uint32 priority
uint32 PRIORITY_LOW = 0
uint32 PRIORITY_NORMAL = 10
uint32 PRIORITY_AVOID_OBSTACLE = 20
uint32 PRIORITY_EMERGENCY = 30
uint32 PRIORITY_MANUAL_OVERRIDE  = 40
