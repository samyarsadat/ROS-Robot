<h1 align="center">The ROS Robot project</h1>

<p align="center">
	<br>
	<a href="https://www.ros.org"><img src="https://github.com/samyarsadat/ROS-Robot/raw/stage-1/Assets/Logos/ROS_logo.svg"></a><br>
	<br>
	<br>
	<a href="https://github.com/samyarsadat/ROS-Robot/blob/dev/LICENSE"><img src="https://img.shields.io/github/license/samyarsadat/ROS-Robot?color=blue"></a>
	|
	<a href="https://github.com/samyarsadat/ROS-Robot#this-project-will-be-completed-in-stages"><img src="https://img.shields.io/badge/Current_Stage-One-red"></a>
	|
	<a href="https://github.com/samyarsadat/ROS-Robot/issues"><img src="https://img.shields.io/github/issues/samyarsadat/ROS-Robot"></a>
	<br><br>
</p>

----
### Disclaimer: Robot is in development.
The ROS Robot project.
<br>
<br>
<br>
<img src="https://github.com/samyarsadat/ROS-Robot/raw/stage-1/Assets/Renders/GitHub_Render_1_Edited.png">
This is a 3D render of the CAD designs.
<br>
<br>

## Overview

### Stages
***This project will be completed in stages.***<br>
*The project is currently at stage 1.*<br>
<br>
<br>

#### Stage 1 (Humble Beginnings / Follow Me)
The robot will follow an object with image processing and a Raspberry Pi camera.<br>
<br>

#### Stage 2 (S.L.A.M. It Shut)
A LIDAR sensor will be added.<br>
The robot will be able to map and navigate its environment.<br>
<br>

#### Stage 3 (Samyarm 1)
A robotic arm will be added for object manipulation.<br>
<br>

#### Stage 4 (Back To The Shipyard)
After stage 3 is completed, I plan to re-design some parts of the robot chassis and PCB.<br>
This will improve upon the current design and allow for future stages and expansions.<br>
Most of the wiring and general design will be kept the same.<br>
This will only improve upon the current design.<br>
<br>

#### Future stages
More stages may be added as the project progresses.

<br>

### System Architecture and Mechanical Design Overview
#### System Architecture
The general architecture of the robot's electronics system consists of a main computer *(the Raspberry Pi 4B)* and two, less powerful microcontrollers *(the Raspberry Pi Pico/RP2040)*.
The Pi 4 handles image processing, mapping & navigation, and any other type of resource-intensive processing whilst the less-powerful Picos handle I/O for motors, sensors, LEDs, etc.<br>
<br>
ROS is used to handle communications between multiple nodes either on the same machine (i.e. a mapping node and a navigation node running on the Pi 4) or between external nodes (i.e. the two Raspberry Pi Picos running microROS). 
Both Raspberry Pi Picos are connected to the Pi 4 via USB cables.<br>
<br>
More details regarding the electronics design <a href="https://github.com/samyarsadat/ROS-Robot/tree/stage-1/Circuit%20diagrams%20%26%20PCB%20files">here</a>.<br>
<br>

#### Mechanical Design Overview
The mechanical design of the robot is quite simple. 
The chassis of the robot is 3D printed in two halves using PLA filament (more details regarding 3D printing <a href="https://github.com/samyarsadat/ROS-Robot/tree/stage-1/CAD%20files/STL%20files">here</a>) and robot uses four geared DC motors (Namiki 22CL-3501PG) in a differential drive configuration.<be>

<br>

## Contact
You can contact me via e-mail.<br>
E-mail: samyarsadat@gigawhat.net<br>
<br>
If you think that you have found a bug or issue please report it <a href="https://github.com/samyarsadat/ROS-Robot/issues">here</a>.

<br>

## Contributing
Please take a look at <a href="https://github.com/samyarsadat/ROS-Robot/blob/dev/CONTRIBUTING.md">CONTRIBUTING.md</a> for contributing.

<br>

## Credits
| Role           | Name                                                             |
| -------------- | ---------------------------------------------------------------- |
| Lead Developer | <a href="https://github.com/samyarsadat">Samyar Sadat Akhavi</a> |
| CAD Design     | <a href="https://github.com/samyarsadat">Samyar Sadat Akhavi</a> |
| PCB Design     | <a href="https://github.com/samyarsadat">Samyar Sadat Akhavi</a> |

<br>
<br>

Copyright © 2022-2023 Samyar Sadat Akhavi.

