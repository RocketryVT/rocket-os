std_msgs/Header header
string source # name of the node issuing the command
uint8 command # the command itself
uint8 priority # the priority of the command, where higher priority is given preference
duration pulse # the duration of a pulse, if commanded

# for all drivers
uint8 RELEASE=0 # releases the node's control over the driver

# DC motor commands
uint8 MOTOR_STOP=1
uint8 MOTOR_CLOSE=2
uint8 MOTOR_OPEN=3
uint8 MOTOR_PULSE_CLOSE=4 # closes for duration defined above, then stops
uint8 MOTOR_PULSE_OPEN=5 # opens for duration defined above, then stops

# solenoid commands
uint8 SOLENOID_INACTIVE=6 # closes and deactivates cycle
uint8 SOLENOID_ACTIVE=7 # activates open/close cycle

# ematch commands
uint8 EMATCH_LOCK=8 # prevents others from firing the ematch
uint8 EMATCH_UNLOCK=9 # allows others to fire the ematch
uint8 EMATCH_FIRE=10 # fires it, duh
