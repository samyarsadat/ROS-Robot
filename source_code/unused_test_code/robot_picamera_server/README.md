## Robot PiCamera2 Server
This was a test program written to expose a Raspberry Pi Camera through an HTTP server using PiCamera2 and FastAPI. 
It is no longer necessary, as I have chosen a different method for getting and publishing images from the Raspberry Pi Camera V3.
<br><br>
The Raspberry Pi Camera V3 only supports the new camera stack, which uses `libcamera`, and it also uses the Sony IMX708 image sensor. 
The Linux kernel version used by Ubuntu 22.04 does not support the IMX708. This means that it would be highly impractical to attempt 
to use the camera directly with Ubuntu 22.04. As such, I decided to run Raspberry Pi OS as the host operating system of the robot's 
on-board Raspberry Pi 5 and to run all ROS-related nodes in Docker containers running Ubuntu 22.04. I would then run this server on 
the host OS and have a separate node inside of a Docker container, which would then use the HTTP API provided by the server to get 
images (or a stream of images) from the camera and publish them to ROS topics. This proved highly inefficient, especially for 
uncompressed image streams. I have decided to publish the code for the server here regardless, hoping that it may be of some use to someone.
<br><br>
As for the robot itself, I have decided to upgrade from ROS 2 Humble Hawksbill to ROS 2 Jazzy Jalisco, the next LTS release of ROS 2, 
allowing me to use Ubuntu 24.04 instead of 22.04. Ubuntu 24.04 fully supports the IMX708 as well as the new `libcamera`-based camera 
stack from Raspberry Pi. Granted, this will require numerous changes to Dockerfiles amongst other configuration files, both for the 
ROS Robot and the ROS Remote. However, I don't see myself as having much of a choice in this matter, I'm afraid.