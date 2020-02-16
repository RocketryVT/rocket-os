import Adafruit_BBIO.GPIO as GPIO
import time
import signal

def signal_handler(sig, frame):
    GPIO.cleanup()
    exit(0)

signal.signal(signal.SIGINT, signal_handler)

pins = ["P9_15", "P9_13"]

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

