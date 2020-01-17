#! /usr/bin/env python

# Blinking onboard led example
import Adafruit_BBIO.GPIO as gpio
import time
import signal

gpio.cleanup()

def signal_handler(sig, frame):
    gpio.cleanup()
    exit(0)

signal.signal(signal.SIGINT, signal_handler)

gpio.setwarnings(1)

# test built in LEDs on BeagleBone Blue
LED =  [ "USR0" , "USR1", "USR2", "USR3" ]

for i in LED:
    gpio.setup(i, gpio.OUT)

while True:
    for i in LED:
        gpio.output(i, gpio.HIGH)
        time.sleep(0.5)

    for i in LED:
        gpio.output(i, gpio.LOW)
        time.sleep(0.5)

