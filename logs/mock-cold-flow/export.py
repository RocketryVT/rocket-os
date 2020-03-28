#! /usr/bin/env python3

import rosbag
import sys
from datetime import datetime

bag = rosbag.Bag(sys.argv[1])
for topic, msg, t in bag.read_messages():
    time = str(datetime.fromtimestamp(t.to_sec()))
    print("[{}] [{}]:\n{}\n".format(time, topic, msg))
bag.close()
