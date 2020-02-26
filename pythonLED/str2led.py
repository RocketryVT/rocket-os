#! /usr/bin/env python

import Adafruit_BBIO.GPIO as gpio
import time
import signal
import sys
import bitarray

def blink(word):

    ba = bitarray.bitarray()
    ba.frombytes(word.encode("utf-8"))
    l = ba.tolist()

    for led in LEDs:
        gpio.output(led, gpio.LOW)

    for i in range(len(l)/4):
        states = l[i*4:(i+1)*4]
        for led, state in zip(LEDs, states):
            gpio.output(led, state)
        time.sleep(0.03)

    for led in LEDs:
        gpio.output(led, gpio.LOW)


def signal_handler(sig, frame):
    gpio.cleanup()
    exit()


if __name__ == "__main__":

    signal.signal(signal.SIGINT, signal_handler)

    if not sys.argv[1:]:
        print("bro, I need an argument")
        exit()

    LEDs =  [ "USR0", "USR1", "USR2", "USR3" ]
    for led in LEDs:
        gpio.setup(led, gpio.OUT)

    blink(sys.argv[1])


