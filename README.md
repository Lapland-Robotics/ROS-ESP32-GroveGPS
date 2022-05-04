# ROS-ESP32-GroveGPS
## Directory contents

**Carla_Rosbridge**
- Python code for Carla and HUD GPS data (lines 319-320, 410-430, 446-492)
- File path: *opt/carla-ros-bridge/melodic/lib/carla_manual_control/carla_manual_control.py*


**GPS**
 - esproswifi (ESP32 and Grove GPS code)
 - PythonTests (rosserial connection and data transfer test scripts, original and modified)


**Libraries**
 - espsoftwareserial
 - TinyGPS


## Importing libraries to Arduino IDE environment

**Steps**
1) Extract folders from zip-files
2) Move extracted folders inside Arduino libraries folder


**Libraries folder path depending on operating system**

- Windows: *Documents -> Arduino -> libraries*
- Ubuntu: *Home -> Arduino -> libraries*


## ROS server environment variables

1. export ROS_MASTER_URI=http://<YOUR_IP_ADDRESS>:11311

2. export ROS_HOSTNAME=<YOUR_IP_ADDRESS>


## Commands for running ROS server

1. roscore
2. rosrun rosserial_python serial_node.py tcp
3. rostopic echo <rostopic_name>
