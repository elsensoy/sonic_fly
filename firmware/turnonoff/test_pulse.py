#!/usr/bin/env python3
"""Send 'p' to the turnonoff Nano over serial and print its reply.

Usage: python3 test_pulse.py [port]   (default port: /dev/ttyUSB0)
Requires: pip install pyserial
"""
import sys
import time
import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"

ser = serial.Serial(port, 115200, timeout=2)
time.sleep(2.5)  # let the Nano finish its reset-on-connect
ser.reset_input_buffer()

ser.write(b"p")
time.sleep(0.5)

print(ser.read(200))
ser.close()
