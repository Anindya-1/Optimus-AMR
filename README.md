# Optimus-AMR
Differential drive bot using oriental motors(BLV620km-gfs)

## Motor Driver Connections
![Motor Driver Connection Diagram](Documents/images/Motor_driver_connection.jpg)

Connections required:
CN1: connected to 24V DC power source
CN2, CN3, CN8 : connected to motor
CN5 : connected to RS485-USB converter via ethernet cable.

##  Connection to RS485-USB conerter
[RS485-USB Converter used](https://www.waveshare.com/usb-to-rs485.htm)

![CN5 connection diagram](Documents/images/RS485_connection.jpg)
![Ethernat cable wiring diagram](Documents/images/RJ45-Pinout-T568B.jpg)
![RS485-USB converter](Documents/images/RS485-USB_connector.jpg)

connections at converter end: <br />
GND(converter) - wire2{orange}(Ethernet cable) <br />
A+(converter) - wire3{green striped}(Ethernet cable) <br />
B-(converter) - wire6{green}(Ethernet cable) <br />

