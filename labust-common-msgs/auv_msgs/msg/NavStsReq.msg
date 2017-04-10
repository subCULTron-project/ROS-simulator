# Local frame (absolute) waypoint request.
Header header

DecimalLatLon global_position
DecimalLatLon origin

# Common waypoint details
GoalDescriptor goal

# NED position and altitude in metres. 
NED position
float32 altitude

# Body velocities in metres/sec, for feed forward loops.
geometry_msgs/Point body_velocity
# Body velocities relative to ground in metres/sec.
geometry_msgs/Point gbody_velocity

# Orientation and orientation rate are in radians and radians/sec
RPY orientation
RPY orientation_rate

# Tolerances are in local frame, zero indicates pilot default should be used.
geometry_msgs/Vector3 position_tolerance
RPY orientation_tolerance

# Axes of control to disable, in body frame.
Bool6Axis disable_axis


