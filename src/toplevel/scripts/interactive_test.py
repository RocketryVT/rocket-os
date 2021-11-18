#! /usr/bin/env python


'''
Interactive Test: ??????????????????????

				  Available Actions: 
					+ 
				
				  Data Managed:
					  + 

				  Num of Functions: 5
'''

import rospy
from std_msgs.msg import String, Bool, Empty
import sys

question_index = 0
expected_response = None
status = False # whether the test has passed

def publish_status(frame):

	'''
		Publishes the status of a test
		
		True: Test Passed
		False: Test did not pass
		
		@param frame: ??????????????
	'''

    publisher.publish(status)

def receive_reset_request(message):

	'''
		Resets the satus variable to False
		
		@param messaage: ??????????????????
	'''

    global status
    status = False
    rospy.loginfo("Recieved reset request.")

def idiot_test(index):

	'''
		??????????????????????????????
	'''

    if len(questions) <= index:
        rospy.loginfo("Test passed!")
        global status
        global question_index
        status = True
        question_index = 0
    else:
        rospy.loginfo("Q" + str(index+1) + ": " + questions[index][0])
        global expected_response
        expected_response = questions[index][1]

def verify_response(resp):

	'''
		Verifies that the given Response parameter matches the expected_response
		
		@param resp: response to be verified
		
	'''

    print("Verifying response")
    global question_index
    if expected_response == resp:
        rospy.loginfo("Correct response recieved!")
        question_index = question_index + 1
        idiot_test(question_index - 1)
    else:
        question_index = 0
        rospy.loginfo("Incorrect response. Ending check.")


def get_command(message):

	'''
		???????????????????????
	'''

    global question_index
    command = message.data

    if command == name_of_test and status:
        rospy.loginfo("Cannot perform test again: " + name_of_test)

    elif command == name_of_test and not question_index:
        rospy.loginfo("Beginning interactive test: " + name_of_test)
        idiot_test(question_index)
        question_index = 1

    elif (command == "yes" or command == "no") and question_index:
        verify_response(command)

    elif question_index:
        question_index = 0
        rospy.logwarn("Non-yes/no command recieved. Ending interactive test: " + name_of_test)


sys.argv = rospy.myargv(argv=sys.argv)
sys.argv.pop(0)

#Initialize Node
rospy.init_node("interactive_test")
name = rospy.get_name()

#Set Subscriptions
rospy.Subscriber("/commands", String, get_command)
rospy.Subscriber(name + "/reset", Empty, receive_reset_request)

#Set Publisher
publisher = rospy.Publisher(name + "/status", Bool, queue_size=10)

if len(sys.argv) < 3:
    rospy.logerr("Requires at least 3 arguments: <name> <Q1> <A1> ...")
    exit()

if len(sys.argv) % 2 == 0:
    rospy.logerr("Requires an odd number of args: <name> <Q1> <A1> ...")
    exit()

name_of_test = sys.argv[0]

questions = []
for i in range(1, len(sys.argv[1:]), 2):
    questions.append((sys.argv[i], sys.argv[i+1]))

#???????????????????????????
rospy.Timer(rospy.Duration(1), publish_status)
rospy.spin()

