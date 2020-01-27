import Adafruit_BBIO.GPIO as gpio
import time
import signal
import sys

def signal_handler(sig, frame):
    gpio.cleanup()
    exit(0)

signal.signal(signal.SIGINT, signal_handler)

left = "P9_11"
right = "P9_12"
all_pins = [left, right]
print(all_pins)

gpio.cleanup()
gpio.setwarnings(1)

for pin in all_pins:
    gpio.setup(pin, gpio.OUT)

while True:

    print("LOW")
    gpio.output(left, gpio.LOW)
    gpio.output(right, gpio.LOW)
    raw_input("Press Enter to continue...")

    print("LEFT")
    gpio.output(left, gpio.HIGH)
    gpio.output(right, gpio.LOW)

    time.sleep(0.1)

    gpio.output(left, gpio.LOW)
    gpio.output(right, gpio.LOW)

    time.sleep(1)

    print("RIGHT")
    gpio.output(left, gpio.LOW)
    gpio.output(right, gpio.HIGH)

    time.sleep(0.1)

    gpio.output(left, gpio.LOW)
    gpio.output(right, gpio.LOW)

    raw_input("Press Enter to continue...")


