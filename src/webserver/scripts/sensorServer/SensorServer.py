#!/usr/bin/env python3

'''
Sensor Server: A Webserver that processes and responds to incomming Get Requests.

			  + Num of Functions: 1
			  + Num of Classes: 1
'''

from http.server import HTTPServer, BaseHTTPRequestHandler
import time
import os 
import random

def arrayUpdate():
    '''
        Keeps a tally of every incoming Get Request ???????????
    '''
    global counter
    global array

    counter += 1
    array.append(counter)


class MyServer(BaseHTTPRequestHandler):
    '''
        A request handler class.
        Subclass of BaseHTTPRequestHandler.
    '''
    def do_GET(self):
        '''
            Redefine BaseHTTPRequestHandler method: do_GET().
            Processes and responds to incomming 'Get Requests'.
        '''
        start_time = time.time() 
        global array
        arrayUpdate()

        self.send_response(200) # Tell Client 'Get Request' was sucessful
        self.send_header("Content-type", "text/html") # Set Header content type to text/html
        self.end_headers() # Write Header to output stream

        # Get directory location of the given file
        dir_path = os.path.dirname(os.path.realpath(__file__))

        # Write contents of MainFile.html
        with open(dir_path + '/MainFile.html', 'rb') as file: 
            self.wfile.write(file.read())

        # Send array containing data
        self.wfile.write(bytes("""
        <script>
        var myArray = """ + str(array) + str(array2) +str(array3) + "</script>", "utf-8"))
        self.wfile.write(bytes("\n","utf-8"))

        self.wfile.write(bytes("</body></html>", "utf-8"))
        self.wfile.write(bytes("\n","utf-8"))
        print("--- %s seconds ---" % (time.time() - start_time))


if __name__ == "__main__":

    # Server Address info
    hostName = "localhost"
    serverPort = 8080

    # Why the hell are we making 12,000 tuples?
    array = [(random.random(),10)]*12000
    array2 = [(random.random(),10)]*12000
    array3 = [(random.random(),10)]*12000
    counter = 0

    # Start webserver
    webServer = HTTPServer((hostName, serverPort), MyServer)
    print("Server started http://%s:%s" % (hostName, serverPort))

    try:
        # Handle requests indefinitely
        webServer.serve_forever()
    except KeyboardInterrupt:
        pass

    webServer.server_close()
    print("Server stopped.")


