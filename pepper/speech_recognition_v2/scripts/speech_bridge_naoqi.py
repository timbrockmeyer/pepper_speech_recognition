#!/usr/bin/env python

#receives ros topic recognized_speech from the corresponding bridge python script
#and displays it on pepper's tablet via naoqi
#also inputs it into pepper's dialogue system via force_input
#since we can not run naoqi and rospy with the same python version

import qi
import sys
import os
import time
import socket

# NAO_IP = '192.168.1.100'
# NAO_PORT = 9559

class NaoqiInputBridge:

    def __init__(self, ip, port):
        try:
            connection_url = "tcp://" + ip + ":" + str(port)
            self.app = qi.Application(["recognized_speech_force_input", "--qi-url=" + connection_url])
        except RuntimeError:
            print("Can't connect to Naoqi.")
            sys.exit(1)

        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) #avoid bugs with closed sockets
            self.s.bind((socket.gethostname(), 1620))
            self.s.listen(5)
            print("Socket Receiver initialized!")
        except Exception, e:
            print ("Failed to initialize socket receiver with error: ", e)

    def run(self):
        try:
            self.app.start()
            session = self.app.session

            tablet_service = session.service("ALTabletService")
	    dir_path = "/opt/aldebaran/www/apps/"

            ALDialog = session.service("ALDialog")
            ALDialog.setLanguage("English")
            ALDialog.runDialog()

            clientsocket, address = self.s.accept()
            self.s.setblocking(False)
            while True:
                recognized_speech = clientsocket.recv(4096)
                print(recognized_speech)

                # display text on tablet
                with open(os.path.join(dir_path, "temp.html"), "wb") as f:
                    f.write('<html><meta name="viewport" content="width=1280, user-scalable=no" /><body bgcolor="#E6E6FA"><center><h1><font size="7"><br><br><br><br>' + recognized_speech + '</font></h1></center></html>')
                tablet_service.showWebview("http://198.18.0.1/apps/temp.html")

                # force recognized speech as input to pepper dialog system
                ALDialog.forceInput(str(recognized_speech))
            clientsocket.close()
        except Exception, e:
                print "Error was: ", e

if __name__ == "__main__":
    ip = "192.168.1.100" # IP of the robot
    port = 9559
    NaoqiInputBridge(ip, port).run()
