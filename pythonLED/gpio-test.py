import Adafruit_BBIO.GPIO as gpio
import time
import signal
import sys

def signal_handler(sig, frame):
    gpio.cleanup()
    exit(0)

signal.signal(signal.SIGINT, signal_handler)

P9 = [11, 12, 13, 15, 17, 18, 23, 24, 26, 27, 30, 41]
P8 = [7, 8, 9, 10, 11, 12, 14, 15, 16, 17, 18, 26]
all_pins = ["P9_{}".format(x) for x in P9] + ["P8_{}".format(x) for x in P8]
print(all_pins)

gpio.cleanup()
gpio.setwarnings(1)

for pin in all_pins:
    gpio.setup(pin, gpio.OUT)

while True:

    print("LOW")
    for pin in all_pins:
        gpio.output(pin, gpio.LOW)

    time.sleep(1)

    print("HIGH")
    for pin in all_pins:
        gpio.output(pin, gpio.HIGH)

    time.sleep(1)

