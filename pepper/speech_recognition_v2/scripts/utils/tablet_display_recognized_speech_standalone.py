#not needed for production but may be interesting
#display something on pepper's tablet by creating an html file and launching a webview
#since this is not really documented anywhere it may be a valuable hack to know about

import qi
import sys
import os
import urlparse, urllib

def path2url(path):
    return urlparse.urljoin(
      'file:', urllib.pathname2url(path))

def main(app):
    try:
        session = app.session
        tabletService = session.service("ALTabletService")
	recognized_speech = "DINGOS<br>DINGOS<br>DINGOS<br>DINGOS<br>DINGOS<br>DINGOS<br>DINGOS<br>DINGOS<br>DINGOS<br>DINGOS<br>DINGOS<br>DINGOS<br>DINGOS<br>DINGOS<br>DINGOS<br>"
	dir_path = "/opt/aldebaran/www/apps/speech_recognition_v2"
	with open(os.path.join(dir_path, "temp.html"), "wb") as f:
		f.write('<html><meta name="viewport" content="width=1280, user-scalable=no" /><body bgcolor="#E6E6FA"><center>' + recognized_speech + '</center></html>')
		print(f)
		print(path2url(str(f)))
        	#tabletService.showWebview(path2url(str(f)))
		tabletService.showWebview("http://198.18.0.1/apps/temp.html")#"http://198.18.0.1/apps/speech_recognition_v2/temp.html")
	app.run()
    except Exception, e:
        print "Error was: ", e


if __name__ == "__main__":
    ip = "192.168.1.100" # the IP of the robot
    port = 9559

    try:
        connection_url = "tcp://{}:{}".format(ip, port)
        app = qi.Application(url=connection_url)
        app.start()
    except RuntimeError:
        print("Can't connect to Naoqi.")
        sys.exit(1)
    main(app)
