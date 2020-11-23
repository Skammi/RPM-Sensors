# RPM-Sensors

An Arduino program to read optical wheel RPM sensors and make them availble over the I2C interface.
The program is based on simple optical sensors with an encoder. The program is using interrupts so the main loop is empty. There are 2 Sensors and the results can be requested over the I2C interface.
I implemeted this on an Arduino Nano to interface to the I2C bus on a Jetson Nano. Initial test where done using an RPi.

For a Jeson/RPi libray module to get the results See: [Ros-cpp-for-Jetbot](https://github.com/Skammi/ROS-cpp-for-Jetbot)
This project includes both the library module and a ROS-node to publish the RPM's


