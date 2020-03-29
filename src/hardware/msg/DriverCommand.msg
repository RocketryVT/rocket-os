std_msgs/Header header
string source # name of the node issuing the command
uint8 command # the command itself
uint8 priority # the priority of the command, where higher priority is given preference
duration pulse # the duration of a pulse, if commanded

# for all drivers
uint8 RELEASE=0 # releases the node's control over the driver

# DC motor commands
uint8 STOP=1
uint8 CLOSE=2
uint8 OPEN=3
uint8 PULSE_CLOSE=4 # closes for duration defined above, then stops
uint8 PULSE_OPEN=5 # opens for duration defined above, then stops

# solenoid commands
uint8 INACTIVE=1 # closes and deactivates cycle
uint8 ACTIVE=2 # activates open/close cycle

# ematch commands
uint8 LOCK=1 # prevents others from firing the ematch
uint8 FIRE=2 # fires it, duh
