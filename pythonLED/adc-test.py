import Adafruit_BBIO.ADC as adc
import time
import signal
import sys

def signal_handler(sig, frame):
    print("")
    exit(0)

signal.signal(signal.SIGINT, signal_handler)

print("Setting up ADC...")
adc.setup()
print("Done.")

pins = ["AIN{}".format(x) for x in range(0,7)]

while True:
    for pin in pins:
        value = adc.read(pin)*1.8
        print("{}: {:0.3f} V ".format(pin, value)),
    print("\r"),

