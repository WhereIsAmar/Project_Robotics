#!/usr/bin/env python
# from http.server import BaseHTTPRequestHandler, HTTPServer
import SimpleHTTPServer
import SocketServer
import time
import rospy
import os
from std_msgs.msg import String


HOST_NAME = os.getenv('HOST', 'localhost')
PORT_NUMBER = os.getenv('PORT', 8080)


def playback(name):
    name = name.lower()
    rospy.loginfo('publishing %s' % name)
    rospy.Publisher('handshake/play', String, queue_size=1, latch=True).publish(String(name))


def edit(name):
    rospy.loginfo('edit not implemented yet {}'.format(name))


ACTIONS = {
    'playback': playback,
    'edit': edit
}


class Server(SimpleHTTPServer.SimpleHTTPRequestHandler):
    def do_HEAD(self):
        return

    def do_POST(self):
        return

    def do_GET(self):
        rospy.loginfo('Request: %s' % self.path)
        _, action, name = str(self.path).split('/')
        func = ACTIONS[action]
        func(name)
        self.send_response(204)

    def handle_http(self):
        return

    def respond(self):
        return


if __name__ == '__main__':
    rospy.init_node('webserver')
    httpd = SocketServer.TCPServer((HOST_NAME, PORT_NUMBER), Server)
    rospy.loginfo("%s Server Up - %s %s" % (time.asctime(), HOST_NAME, PORT_NUMBER))
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass

    httpd.server_close()
    print(time.asctime(), 'Server DOWN - %s:%s' % (HOST_NAME, PORT_NUMBER))
