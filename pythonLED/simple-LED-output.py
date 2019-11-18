import Adafruit_BBIO.GPIO as GPIO
import time
GPIO.cleanup()
counter = 0;
GPIO.setup("P9_11", GPIO.OUT)

while True:
    time.sleep(1)
    GPIO.output("P9_11", GPIO.HIGH)
    print("HIGH ")
    time.sleep(1)
    GPIO.output("P9_11", GPIO.LOW)
    print("LOW ")


    