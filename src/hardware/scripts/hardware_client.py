#!/usr/bin/env python

# hardware_client.py

import random
import rospy
from hardware.msg import MotorCommand, SolenoidCommand

def main():
    rospy.init_node("hardware_client")

    motor_cmd = rospy.Publisher("motor_command",
        MotorCommand, queue_size=100)
    solenoid_cmd = rospy.Publisher("solenoid_command",
        SolenoidCommand, queue_size=100)

    rate = rospy.Rate(1);
    while not rospy.is_shutdown():

        cmd = MotorCommand();
        cmd.command = int(random.random()*3);
        motor_cmd.publish(cmd);
        cmd = SolenoidCommand();
        cmd.command = int(random.random()*2);
        solenoid_cmd.publish(cmd);

        rate.sleep();

if __name__ == '__main__':
    main()
