#! /usr/bin/env python

# Blinking onboard led example
import Adafruit_BBIO.GPIO as gpio
import time
import signal

def signal_handler(sig, frame):
    gpio.cleanup()
    exit()

signal.signal(signal.SIGINT, signal_handler)

# test built in LEDs on BeagleBone Blue
LED =  [ "USR0", "USR1", "USR2", "USR3" ]

for i in LED:
    gpio.setup(i, gpio.OUT)

for i in LED:
    gpio.output(i, gpio.LOW)
    time.sleep(0.5)

