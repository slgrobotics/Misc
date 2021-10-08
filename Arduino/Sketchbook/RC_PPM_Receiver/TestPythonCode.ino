/*
 * 

#!/usr/bin/python

import sys
import smbus
from time import sleep

# read 8 RC channels from the RC receiver
# Teensy 3.2 runs RC_PPM_Receiver code

# this is part of the PX4 Lawnmower project https://github.com/slgrobotics/PX4-Autopilot



DEVICE_ADDRESS = 0x48      #7 bit address (will be left shifted to add the read write bit)
DEVICE_REG = 0x00

bus = smbus.SMBus(1)    # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)

count = 0

while True:
    try:
        # read int16 values for 8 channels
        # and extra value for milliseconds since last R/C signal read:
        block = bus.read_i2c_block_data(DEVICE_ADDRESS, 0, 18)
        # print(block)

        blockInt = []

        for i in range(9):
            blockInt.append(int.from_bytes([block[2*i],block[2*i+1]], byteorder='little'))

        print(count, " | ", blockInt)
        count = count + 1
    except:
        print("Error:", sys.exc_info()[0])

    sleep(0.020)

 * 
 */
