#!/bin/bash
rosservice call /apad1/FADP_enable "enable: true" 
rosservice call /apad1/VelCon_enable "enable: true" 

rosservice call /apad1/ConfigureVelocityController "ControllerName: 'FADP' 
desired_mode: [2, 2, 0, 0, 0, 0]" 

rostopic pub /apad1/stateRef auv_msgs/NavSts "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
global_position: {latitude: 0.0, longitude: 0.0}
origin: {latitude: 0.0, longitude: 0.0}
position: {north: 10.0, east: 10.0, depth: 0.0}
altitude: 0.0
body_velocity: {x: 0.0, y: 0.0, z: 0.0}
gbody_velocity: {x: 0.0, y: 0.0, z: 0.0}
orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
orientation_rate: {roll: 0.0, pitch: 0.0, yaw: 0.0}
position_variance: {north: 0.0, east: 0.0, depth: 0.0}
orientation_variance: {roll: 0.0, pitch: 0.0, yaw: 0.0}
status: 0" 

