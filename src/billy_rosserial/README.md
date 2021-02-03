# README.md

This file is about good practices and launching process

## Raspberry pi:

Connect the raspberry to mouse, keyboard and screen

Download ubiquity robot os for RASP

Follow this link: https://downloads.ubiquityrobotics.com/pi.html
Default password is "ubuntu"

Disable automatic wifi broadcast:

Follow this link: https://forum.ubiquityrobotics.com/t/how-do-i-disable-wifi-access-point/33/2

Clone billy_ros GitHub repository:

git clone https://github.com/TeamBilly/billy_ros.git

Go to your RASP .bashrc and add following
```bash
export ROS_IP=IP_OF_YOUR_RASP
export ROS_MASTER_URI=http://IP_OF_YOUR_PC:11311
```
IP_OF_YOUR_RASP can be found by typing "ifconfig" into the RASP terminal and read the first IP from the last section
IP_OF_YOUR_PC can be found by typing "ifconfig" into the PC terminal and read the first IP from the last section

Disconnect mouse, keyboard and screen

Place the RASP on robot:
	
**HIGHLY RECOMMANDED:**
Verify that the power supply from the energy PCB is 5V is indeed 5V.
(You can analog read thanks to Arduino)

Plug Arduino USB cable to RASP
Plug webcam USB cable to RASP
Plug RPlidar USB cable to RASP

Plug power cable on RASP's GPIO
Follow this link: https://cdn.shopify.com/s/files/1/0176/3274/files/4_50e31aa7-9061-424e-be17-cdcd7f303b57_1024x1024.png?v=1561718548

Don't turn master switch ON yet


## Computer
Go to your PC .bashrc and add following
```bash
export ROS_IP=IP_OF_YOUR_PC
export ROS_MASTER_URI=http://IP_OF_YOUR_PC:11311
```

Clone billy_ros GitHub repository:
git clone https://github.com/TeamBilly/billy_ros.git

Turn master switch ON.

Go to terminal:
```bash
roscore
```

New terminal:
```bash
ssh ubuntu@IP_OF_YOUR_RASP
killall -9 roscore
roscore
```

New terminal:
```bash
ssh ubuntu@IP_OF_YOUR_RASP
roslaunch billy_rosserial billy_rosserial_rasp.launch
```
Several short sound from the motor should be heared when connection is made

New terminal:
```bash
roslaunch billy_rosserial billy_rosserial_pc.launch
```
The robot should make a 180° no scope

You can now send any message on cmd_vel topic, the robot should move accordingly


## Shuting down

**DO NOT TURN THE MASTER SWITCH OFF YET**

Kill both process on the terminal controling the RASP (CTRL+C) and exit one using "exit"

On the remaining on shut down the RASP:
```bash
sudo shutdown -P now
```

**WAIT FOR THE RASP RED LED TO BE CONTINUSLY ON BEFORE TURNING MASTER SWITCH OFF**

Kill remaining processes on your PC

Done :)
