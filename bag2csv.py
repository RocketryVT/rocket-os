#!/usr/bin/env python

'''
This script saves each topic in a bagfile as a csv.

Written by Nick Speal in May 2013 at McGill University's Aerospace Mechatronics Laboratory. Bugfixed by Marc Hanheide June 2016.
www.speal.ca

Supervised by Professor Inna Sharf, Professor Meyer Nahon

'''

import rosbag, sys, csv
import time
import string
import os #for file management make directory
import shutil #for file management, copy file

overwrite = False

if len(sys.argv) < 2:
    "usage: ./{} log1.bag log2.bag...".format(sys.argv[0])
    exit()

for arg in sys.argv:
    if arg == "--overwrite":
        overwrite = True
        sys.argv.remove(arg)

listOfBagFiles = sys.argv[1:]
numberOfFiles = len(listOfBagFiles)
count = 0
for bagFile in listOfBagFiles:
    if os.path.isdir(bagFile):
        continue

    print("Reading {}".format(bagFile))
    bag = rosbag.Bag(bagFile)
    bagContents = bag.read_messages()
    bagName = bag.filename

    folder = string.rstrip(bagName, ".bag")
    try:
        os.makedirs(folder)
    except:
        if overwrite:
            print("CSV files already exist for this bag -- overwriting")
            shutil.rmtree(folder)
            os.makedirs(folder)
        else:
            continue

    listOfTopics = []
    for topic, msg, t in bagContents:
        if topic not in listOfTopics:
            listOfTopics.append(topic)

    for topicName in listOfTopics:
        filename = folder + '/' + string.replace(topicName, '/', '_slash_') + '.csv'
        with open(filename, 'w+') as csvfile:
            filewriter = csv.writer(csvfile, delimiter = ',')
            firstIteration = True
            row = 1
            number_of_messages = bag.get_message_count(topicName)
            for subtopic, msg, t in bag.read_messages(topicName):
                print "Writing topic {}: {}%\r".format(topicName, 100*row/number_of_messages),
                msgString = str(msg)
                msgList = string.split(msgString, '\n')
                instantaneousListOfData = []
                for nameValuePair in msgList:
                    splitPair = nameValuePair.split(':', 1)
                    for i in range(len(splitPair)):
                        splitPair[i] = splitPair[i].strip()
                    instantaneousListOfData.append(splitPair)
                if firstIteration:    # header
                    headers = ["timestamp"]
                    for pair in instantaneousListOfData:
                        headers.append(pair[0])
                    filewriter.writerow(headers)
                    firstIteration = False
                values = [str(t)]
                for pair in instantaneousListOfData:
                    if len(pair) > 1:
                        values.append(pair[1])
                filewriter.writerow(values)
                row += 1
            print("")
    bag.close()
print("Done");
