# Ros node for the kitt car embedded system

The following are the dependencies for kitt\_teensy\_ros\_node.ino
* Arduino IDE / Teensyduino
	* Follow instructions for [Arduino and Teensyduino installation](https://www.pjrc.com/teensy/td_download.html).
* rosserial
	* compile this from sources, because the debian version (using apt-get install) does not support Teensy 3.5 and above ([This post](https://forum.pjrc.com/threads/40418-rosserial_arduino-for-Teensy) discusses the problem).
	* To compile from sources, follow the instructions in the [rosserial wiki](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup), but make sure to follow Step 2.1.2 (Installing from Source onto the ROS workstation) instead of Step 2.1.1
* LSM6
	* This is an Arduino library, but **should not** be installed using the arduino library manager.
	* Clone the [Teensy compatible LSM6 sources](https://github.com/Teensy-Compatible-Libraries/lsm6-arduino.git) into the Arduino/libraries directory.
* LIS3MDL
	* This is again, an Arduino library, and **should not** be installed using the arduino library manager.
	* Clone the [Teensy compatible LIS3MDL sources](https://github.com/Teensy-Compatible-Libraries/lis3mdl-arduino.git) into the Arduino/libraries directory.
