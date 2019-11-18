import Adafruit_BBIO.GPIO as GPIO
import time
GPIO.cleanup()
GPIO.setup("P9_11", GPIO.OUT)
GPIO.setup("P9_13", GPIO.OUT)
counter=0;
while True:
    GPIO.output("P9_11", GPIO.HIGH)
    GPIO.output("P9_13", GPIO.LOW)
    print ("G HIGH A LOW")
    time.sleep(1)
    GPIO.output("P9_11", GPIO.LOW)
    GPIO.output("P9_13", GPIO.HIGH)
    print ("G LOW A HIGH")
    time.sleep(1)
    