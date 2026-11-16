[Steering]
+-15 degrees, controller can only give a constant movement from any angle to the left and right, to stop the turning you have to give a stop command which stops the movement and locks the wheel into the angle which it recieves the stop command. Response time is 200ms, meaning when you give the first command and another command after, it acknowledges it after 200ms.bbefore 200ms the acceptance of the second command is uncertain, can be dropped or accepted. 

we have to test, but it's possible that one of the steering directions (right or left) is faster than turning hte other direction (clockwise may be faster than anticlockwise, vise versa) 

[Encoder]
Encoder gives angle of the steering wheel, turning.
Tick encoder, not absolute, and we don't know the absolute angle of wheel ever because the encoder is on the steering wheel not the wheel. Values come through serial passed through an ESP32

[GPS and Odom]
taken through a vectornav, very accurate GPS and Odom.

[Stereo Camera]
Intel realsense, attatched to the front of the kart near the top of the windshield. Used for segmenting the road

[3d Lidar]
LiDAR Mid360 attached to the roof of the kart. for obstacle detection and avoidance mostly. 

[Speed]
Constant speed of 15 kmph, Acceleration and decelartion to be calculated. 

[Remote control]
Controlled by serial inputs, we'll attach the controls in this directory from the repo. 

[Environment]
Ros2 Jazzy, Ubuntu 24 on a jetson agx orin. 

[RL Controller]
Goal: an intelligent controller that works around the kart's drive-by-wire drawbacks instead of
assuming an ideal steering actuator. The drawbacks it has to absorb:
- Steering is bang-bang: only LEFT / RIGHT (constant rate) / STOP-and-hold, no position command.
- ~200 ms command latency; a second command inside that window may be dropped or accepted, and
  the controller gets no acknowledgement either way (only the encoder shows what happened).
- Wheel keeps moving for a while after STOP (overshoot), and the rate may differ left vs right.
- Encoder is relative and on the steering wheel, not the road wheels: zero is lost on every
  ESP32 power cycle and there is backlash between encoder angle and real wheel angle.
- Throttle is open loop (target 15 km/h), steering limited to +-15 deg.
The policy should plan its steering pulses ahead (lead the stop, avoid commands it can't count on),
keep a belief of the true wheel angle from encoder + VectorNav (yaw rate / speed), and track the
lane / path from the RealSense segmentation while avoiding Mid360 obstacles.

Plan: kart_testing_modules measures the actuator (p02-p08 parked, p09-p11 driving) -> fit an
actuator model (delay, drop probability vs command spacing, rate per direction, overshoot,
backlash) -> put that model in the Isaac Sim digital twin in place of the ideal steering in
cart_controller.py, with domain randomisation around the measured values -> train there ->
validate by replaying recorded p11 command sequences through the model -> deploy as a ROS 2
node on the Orin.
