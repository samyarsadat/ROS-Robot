These scripts are for setting up, running, and updating all the ROS packages that will
run on the robot's on-board Raspberry Pi. I'm using a Raspberry Pi 5 8GB with the Raspberry Pi
AI Kit and the Raspberry Pi Camera V3. I'm also running Raspberry Pi OS on the Pi.<br>
<br>
You should first run the setup script, then you can make the `run_driver.bash` script execute on
startup using `systemd`, although Docker Compose should automatically start the container if you
haven't manually stopped it since you last started it.