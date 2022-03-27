# Python 3 server example
from http.server import BaseHTTPRequestHandler, HTTPServer
import time
import os
import random


def arrayUpdate():
    global counter
    global array

    counter += 1
    array.append(counter)


class MyServer(BaseHTTPRequestHandler):
    def do_GET(self):

        start_time = time.time()
        global array
        arrayUpdate()

        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()

        dir_path = os.path.dirname(os.path.realpath(__file__))

        with open(dir_path + '/MainFile.html', 'rb') as file:
            self.wfile.write(file.read())

        # #Sends array containing data

        self.wfile.write(bytes("""
        <script>
        var myArray = """ + str(array) + str(array2) + str(array3) + "</script>", "utf-8"))
        self.wfile.write(bytes("\n", "utf-8"))

        self.wfile.write(bytes("</body></html>", "utf-8"))
        self.wfile.write(bytes("\n", "utf-8"))
        print("--- %s seconds ---" % (time.time() - start_time))


if __name__ == "__main__":

    hostName = "192.168.1.34"
    serverPort = 9000
    array = [(random.random(), 10)]*12000
    array2 = [(random.random(), 10)]*12000
    array3 = [(random.random(), 10)]*12000
    counter = 0

    webServer = HTTPServer((hostName, serverPort), MyServer)
    print("Server started http://%s:%s" % (hostName, serverPort))

    try:
        webServer.serve_forever()
    except KeyboardInterrupt:
        pass

    webServer.server_close()
    print("Server stopped.")
