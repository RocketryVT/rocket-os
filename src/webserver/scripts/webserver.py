#!/usr/bin/env python

# webserver.py

from BaseHTTPServer import HTTPServer, BaseHTTPRequestHandler
from rospy_message_converter import json_message_converter
import signal, sys, os
import rospy, rospkg, rostopic
import json

rospack = rospkg.RosPack()
package_path = rospack.get_path('webserver')
rospy.init_node('webserver')
client_data = {}

class HttpApi(BaseHTTPRequestHandler):

    def __init__self():
        pass

    def log_message(self, format, *args):
        pass

    def do_HEAD(self):
        pass

    def do_POST(self):

        global client_data

        if self.path == '/update':

            client_data["time"] = rospy.get_time();
            json_dump = json.dumps(client_data)
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(json_dump)
            return

        self.send_response(400)
        self.end_headers()

    def do_GET(self):

        global package_path

        if self.path == '/':

            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(open(package_path + '/html/index.html').read())
            return

        if self.path == '/vis':

            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(open(package_path + '/html/vis.html').read())
            return

        if self.path == '/css/styles.css':

            self.send_response(200)
            self.send_header('Content-Type', 'text/css')
            self.end_headers()
            self.wfile.write(open(package_path + '/css/styles.css').read())
            return

        if self.path == '/css/normalize.css':

            self.send_response(200)
            self.send_header('Content-Type', 'text/css')
            self.end_headers()
            self.wfile.write(open(package_path + '/css/normalize.css').read())
            return

        if self.path == '/scripts/index.js':

            self.send_response(200)
            self.send_header('Content-Type', 'text/javascript')
            self.end_headers()
            self.wfile.write(open(package_path + '/scripts/index.js').read())
            return

        if self.path == '/scripts/three.js':

            self.send_response(200)
            self.send_header('Content-Type', 'text/javascript')
            self.end_headers()
            self.wfile.write(open(package_path + '/scripts/three.js').read())
            return

        self.send_response(404)
        self.end_headers()

# Exit on Ctrl-C.
def ctrl_c(signal, frame):
    sys.exit(0)
signal.signal(signal.SIGINT, ctrl_c)

addr = "0.0.0.0"
port = 7000
rospy.loginfo("Begin server on " + addr + ":" + str(port))
server = HTTPServer((addr, port), HttpApi)

class MessageConverter:
    def __init__(self, topic_name):
        self.topic_name = topic_name
    def callback(self, msg):
        global client_data
        json = json_message_converter.convert_ros_message_to_json(msg)
        client_data[self.topic_name] = json

subscribers = {}
for topic in rospy.get_published_topics():
    topic_name = topic[0];
    if topic_name in subscribers:
        continue;
    rospy.loginfo("Subscribing: " + topic_name);
    message_class = rostopic.get_topic_class(topic[0])[0];
    msgconv = MessageConverter(topic_name);
    subscribers[topic_name] = rospy.Subscriber(topic_name, message_class,
        msgconv.callback)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    server.handle_request()
    rate.sleep()

