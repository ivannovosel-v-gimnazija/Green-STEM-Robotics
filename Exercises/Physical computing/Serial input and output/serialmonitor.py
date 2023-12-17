# Green STEM - Robotics
# Exercise: Serial input and output
# Author: Ivan Novosel
# Date: 202-12-17
# Example code on how to make a serial monitor for Arduino, and simmilar devices
# This uses pyserial, please install it before using
# https://pyserial.readthedocs.io/en/latest/pyserial.html#installation
#
# This program reads from an UART enabled device and writes into a file
# You can modify the code to your specific need
# If will try to connect to a device, if it fails the program gives recommendations
# on how to fix the problem with connection.
# In case of error when reading it will retry to gain connection again
# The program ends by pressing Ctrl+C

from serial import *
from serial.tools.list_ports import comports
from time import *

# change this to your current settings set up in Arduino IDE
port = 'COM9'
rate = 9600

try:
    arduino = Serial(port=port, baudrate=rate, timeout=0.1)
    arduino.flushInput()
except SerialException:
    print(f"There is no Arduino connected on port {port}")
    print(f"Change the selected port in the codem and check if the USB cable is connected.")
    print(f"If Serial monitor in Arduino IDE is working shut it down!")
    print(f"The list of ports with detected devices:")
    for port in comports():
        print(port.description)
    exit()

# the name and location of the file where we will write the received data
name = 'data.csv'

with open(name,"a") as file:
    # this will be the header row in the CSV file, change it according to your data
    file.write("Timestamp, header 1, header 2...\n")
    while True:
        try:
            message = arduino.readline().decode().strip()
            h,m,s = localtime().tm_hour, localtime().tm_min, localtime().tm_sec
            file.write(f"{h}:{m}:{s},{message}\n")
            print(f"{h}:{m}:{s},{message}")
        except SerialException:
            print("Error reading from device!")
            arduino.close()
            working = False
            while not working:
                try:
                    arduino = Serial(port=port, baudrate=rate, timeout=0.1)
                    working = True
                    arduino.flushInput()
                except SerialException:
                    print(f"No Arduino connected to {port}")
                    print("Checking again in 1 second...")
                    sleep(1)
        except KeyboardInterrupt:
            print("CTRL+C pressed - exiting!")
            arduino.close()
            break