# Raspberry Pi Pico Source Code
This folder contains the source code (firmware) for the two Raspberry Pi Picos that are on the robot's main board.<br>
The firmware is written in C++ and uses the micro-ROS library to handle communications.<br>
Pre-built binaries can be found in the `build` directory.<br>

<br>

## Important Note #1:
_The motor encoder implementation may look unconventional compared to other robots.<br>
Two different methods are utilized when measuring motor speed:<br>_
<br>

#### First Method
_This method measures the time between each encoder pulse to determine the RPM.<br>
The problem with this method is that when the motor stops, so does RPM calculation (I explain why this is a problem below).
This is because the variables that contain the time at which the last and current pulses were received are only updated when a pulse is received from the encoder,
or rather, they can **only** be updated when an encoder pulse is received.<br>
<br>
This method produces very accurate and stable RPM readings even with a low-resolution encoder (such as the ones found on the Namiki 22CL-3501PG motors that only send 160 pulses per gearbox shaft rotation);<br>
**However, since RPM calculations stop when the encoders stop sending pulses, RPM readings get stuck at a low value (5~18) even though the motor isn't rotating.**<br>
<br>_

#### Second Method
_This method measures the number of pulses received within a pre-set time period (referred to as the sample time) and then divides 
the number of pulses received by the sample time to get the (average) time between each encoder pulse.<br>
The problem with this method is that its measurements can be a bit choppy/inaccurate/unstable with low-resolution encoders.
This is because there might not be enough pulses sent by the encoder during a single sample period to obtain an accurate RPM reading.<br>
<br>
**Unlike the first method, however, this method can measure 0 RPM without any issues as its RPM calculations do not rely on encoder pulses and are performed on a fixed timer.**<br>
So if no pulses are received during a sample period, the measured RPM will drop to zero, or at least very close to zero (we can round to zero as it will never go down to absolute zero)._<br>
<br>

#### The Implementation
_Usually, I would only use the second method for measuring motor speed; However, because the Namiki 22CL-3501PG motors that I'm using have low-resolution encoders, I cannot reliably
and accurately measure motor speeds by only using the second method.<br>
<br>
In the current implementation, motor speed is measured using the first method as long as the speed measured by the second method is above 10 RPM. When the second method's reading drops below 10 RPM,
the program starts using the second method's measurement. Thanks to this implementation, we get accurate readings whilst also being able to measure 0 RPM._
