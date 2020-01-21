import Adafruit_BBIO.GPIO as GPIO
import time
import signal

def signal_handler(sig, frame):
    GPIO.cleanup()
    exit(0)

signal.signal(signal.SIGINT, signal_handler)

GPIO.cleanup()
GPIO.setup("P9_11", GPIO.OUT)

while True:
    GPIO.output("P9_11", GPIO.HIGH)
    print("HIGH ")
    time.sleep(1)
    GPIO.output("P9_11", GPIO.LOW)
    print("LOW ")
    time.sleep(1)

