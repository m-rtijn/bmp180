#!/usr/bin/python
"""
Released under the MIT License
Copyright 2015 MrTijn/Tijndagamer
"""

from BMP180 import BMP180

bmp = BMP180(0x77)

print("Temp: " + str(bmp.get_temp()) + " Celcius")
print("Pressure: " + str(bmp.get_pressure()) + " Pascal")
print("Altitude: " + str(bmp.get_altitude()) + " meter")