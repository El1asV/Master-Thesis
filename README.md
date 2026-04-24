This repository contains the code used in my master's project. The encoder.ino file is code for an Arduino Uno which reads position data of a RLS AksIM-2 SPI encoder and transmits it over serial.
The position_controller.py file is a program for running position control on an ODrive S1 with the encoder data from the AksIM-2 as the regulated axis.
