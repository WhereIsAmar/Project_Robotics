#!/usr/bin/env python
# from http.server import BaseHTTPRequestHandler, HTTPServer
import SimpleHTTPServer
import SocketServer
import time
import handshake
import rospy
import os


HOST_NAME = os.getenv('HOST')
PORT_NUMBER = 8080


def playback(name):
    limb, hd, _ = handshake.init_sawyer()
    print(limb)
    handshake.playback(hd, limb, name)


def edit(name):
    print('not implemented yet {}'.format(name))


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
        _, action, name = str(self.path).split('/')
        func = ACTIONS[action]
        func(name)
        return

    def handle_http(self):
        return

    def respond(self):
        return


if __name__ == '__main__':
    rospy.init_node('webserver')
    httpd = SocketServer.TCPServer((HOST_NAME, PORT_NUMBER), Server)
    print(time.asctime(), 'Server UP - %s:%s' % (HOST_NAME, PORT_NUMBER))
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    # httpd.server_close()
    print(time.asctime(), 'Server DOWN - %s:%s' % (HOST_NAME, PORT_NUMBER))
