from http.server import BaseHTTPRequestHandler, HTTPServer
import time

def playback(name):
    print('Im going to play {}s handshake from a function.'.format(name))

def edit(name):
    print('not implemented yet {}'.format(name))

ACTIONS = {
    'playback': playback,
    'edit': edit
}

class Server(BaseHTTPRequestHandler):
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


HOST_NAME = 'localhost'
PORT_NUMBER = 8080

if __name__ == '__main__':
    httpd = HTTPServer((HOST_NAME, PORT_NUMBER), Server)
    print(time.asctime(), 'Server UP - %s:%s' % (HOST_NAME, PORT_NUMBER))
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    httpd.server_close()
    print(time.asctime(), 'Server DOWN - %s:%s' % (HOST_NAME, PORT_NUMBER))
