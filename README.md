# Optimus-AMR
Differential drive bot using oriental motors(BLV620km-gfs)

## Motor Driver Connections
![Motor Driver Connection Diagram](Documents/images/Motor_driver_connection.jpg)

Connections required: <br />
CN1: connected to 24V DC power source <br />
CN2, CN3, CN8 : connected to motor <br />
CN5 : connected to RS485-USB converter via ethernet cable. <br />

##  Connection to RS485-USB conerter
[RS485-USB Converter used](https://www.waveshare.com/usb-to-rs485.htm)

![CN5 connection diagram](Documents/images/RS485_connection.jpg)
![Ethernat cable wiring diagram](Documents/images/RJ45-Pinout-T568B.jpg)
![RS485-USB converter](Documents/images/RS485-USB_connector.jpg)

connections at converter end: <br />
GND(converter) - wire2{orange}(Ethernet cable) <br />
A+(converter) - wire3{green striped}(Ethernet cable) <br />
B-(converter) - wire6{green}(Ethernet cable) <br />
## CIRCUIT DIAGRAM FOR AMR
![Circuit Diagram](Documents/images/aMR_circuit_diagram.jpg)
## AMR SETUP
Step1 : Turn ON the MCB placed on the left side of AMR.
<!--![Circuit Diagram](Documents/images/MCB_placement.jpg)-->
Step2 : Turn ON IPC placed at the back of AMR.
<!--![Circuit Diagram](Documents/images/IPC_power.jpg)-->
Step3 : Remotely connect to mira-amr over SSH.(refer to WIFI setup section in readme)
      
      ssh mira-amr@192.168.193.220  

pwd : FSM@2024

Step4 : Run the following command in host terminal to activate the controllers .

      arise_mira

This command will load the robot description as well as activate the differential drive controller , LIDAR and Camera .
      
Step5 : For Teleoperation of the AMR use command in user terminal

      ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=diffbot_base_controller/cmd_vel_unstamped

Note : Make sure the host and user are sharing ros2 topics.
<!--(both systems must have same ROS_DOMAIN and RMW_IMPLEMENTATION-->


<video src="https://github.com/Anindya-1/Optimus-AMR/blob/main/Documents/video/document_6102519452346093974.mp4" width = "180"></video>
[![Watch the video]()]([https://youtu.be/vt5fpE0bzSY](https://github.com/Anindya-1/Optimus-AMR/blob/main/Documents/video/document_6102519452346093974.mp4))


