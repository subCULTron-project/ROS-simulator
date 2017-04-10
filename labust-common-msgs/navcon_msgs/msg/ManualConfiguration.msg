## DOF accessor
int32 DISABLED=-1
int32 X=0
int32 Y=1
int32 Z=2
int32 K=3
int32 M=4
int32 N=5

## Joystick remap options
# Which axes to use for DOF. Use -1 to disable DOF
int32[] axes_map
# What scalling to apply in order to get the joystick values in [-1,1]
float32[] scale_map
# Use integration with time constant, e.g. for hat(pov) axes. 
# For zero or negative the integration is ignored.
float32[] integrated

## Effort generator options
float32[] maximum_effort

## Speed generator options 
float32[] maximum_nu

## Position generator options 
float32[] maximum_speed
# Sampling time for state integration (if zero, defaults to 0.1)
float64 sampling_time
# Feed-forward option. If true will also set the body_velocities for feed-forward.
bool use_ff

## The joystick topic that should be used
string topic
