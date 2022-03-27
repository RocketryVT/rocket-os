#!/usr/bin/env python3

# tcp_server.py

import socket
import select
import sys
from _thread import *
import atexit
import signal
import time
import sched
from datetime import datetime
import bitarray

import rospy
from std_msgs.msg import String, Duration
from rosgraph_msgs.msg import Log

try:
    import Adafruit_BBIO.GPIO as gpio
except:
    gpio = None


def exit_handler():
    rospy.loginfo("Exiting.")
    time = str(datetime.now())
    pineapple_client.publish(level_to_str(2) + " [" + time + "] [" + name + "]: Exiting.")



def signal_handler(sig, frame):
    exit()


def level_to_str(level):

    if level is 1:
        return '[DEBUG]'
    if level is 2:
        return '[INFO] '
    if level is 4:
        return '[WARN] '
    if level is 8:
        return '[ERROR]'
    if level is 16:
        return '[FATAL]'
    return '[?????]'


def get_rosout(msg):
    time = str(datetime.fromtimestamp(msg.header.stamp.to_sec()))
    pineapple_client.publish(level_to_str(msg.level) + " [" + time + "] [" + str(msg.name) + "]: " + msg.msg)


    if gpio:
        led = "USR0"
        gpio.output(led, gpio.HIGH)
        rospy.sleep(0.1)
        gpio.output(led, gpio.LOW)

def last_event(event):
    global last_event_time
    last_event_time = rospy.Time.now().to_sec()


def publish_los(event):
    global last_event_time


    global los_start_time
    global los_condition

    los_duration = rospy.Time.now() - los_start_time
    los_duration_secs = rospy.Time.now().to_sec() - last_event_time
    # Client connections available
    if los_duration_secs < 10:
        print("ahhhhhhhh")
        los_start_time = rospy.Time.now()
        los_duration = rospy.Duration()  # Set default duration of 0 sec & 0 nanosec
        if los_condition:
            rospy.loginfo("Connection with atleast one client restored.")
        los_condition = False

    # No client connections available
    elif not los_condition:
        rospy.logwarn("LOS condition detected!")
        los_condition = True

    pub_los.publish(los_duration)


def blink_leds(message):

    if not gpio:
        return

    word = message.data

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


if __name__ == "__main__":

    # register signal handler and exit handler
    atexit.register(exit_handler)
    signal.signal(signal.SIGINT, signal_handler)

    if gpio:
        LEDs = ["USR0", "USR1", "USR2", "USR3"]
        for led in LEDs:
            gpio.setup(led, gpio.OUT)
            gpio.output(led, gpio.LOW)

    rospy.init_node("tcp_server", log_level=rospy.DEBUG)
    name = rospy.get_name()

    if not gpio:
        rospy.logwarn(
            "Failed to import Adafruit_BBIO.gpio, running in desktop mode")

    last_broadcast = datetime.now()
    los_start_time = rospy.Time.now()
    los_condition = False

    rospy.Subscriber("/rosout", Log, get_rosout)
    rospy.Subscriber("/requested_commands", String, blink_leds)
    rospy.Subscriber("/keep_alive", String, last_event)

    pub_los = rospy.Publisher("/los", Duration, queue_size=10)
    pub_command = rospy.Publisher("/requested_commands", String, queue_size=10)
    pineapple_client = rospy.Publisher("/client_out", String, queue_size=10)
    last_event_time = rospy.Time.now().to_sec() - 99
    
    rospy.Timer(rospy.Duration(1), publish_los)

    rospy.spin()
