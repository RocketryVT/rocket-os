import Adafruit_BBIO.GPIO as GPIO
import time
import signal
import sys

def signal_handler(sig, frame):
    GPIO.cleanup()
    exit(0)

signal.signal(signal.SIGINT, signal_handler)

if len(sys.argv) < 2:
    print("usage:   python simple-led-output.py <pins>")
    print("example: python simple-led-output.py P9_13 P9_15")
    exit()

pins = sys.argv[1:]
print("Testing pins " + ", ".join(pins))

P9 = [11, 12, 13, 15, 17, 18, 23, 24, 26, 27, 30, 41]
P8 = [7, 8, 9, 10, 11, 12, 14, 15, 16, 17, 18, 26]
all_pins = ["P9_{}".format(x) for x in P9] + ["P8_{}".format(x) for x in P8]

for pin in pins:
    if pin not in all_pins:
        print("Error: " + pin + " is not a valid GPIO pin!")
        exit()

GPIO.cleanup()
for pin in pins:
    GPIO.setup(pin, GPIO.OUT)

while True:
    for pin in pins:
        GPIO.output(pin, GPIO.HIGH)
    print("HIGH ")
    time.sleep(1)
    for pin in pins:
        GPIO.output(pin, GPIO.LOW)
    print("LOW ")
    time.sleep(1)

