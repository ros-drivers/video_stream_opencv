#!/usr/bin/env python
# coding=UTF-8

import cv2
import urllib 
import numpy as np

import roslib
roslib.load_manifest('video_stream_opencv')
import signal
import sys
import time
import rospy

from cv_bridge import CvBridge, CvBridgeError
#from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

print(sys.version)
print(sys.argv)

topic_name = 'mjpeg_republisher'
stream_url = 'http://iris.not.iac.es/axis-cgi/mjpg/video.cgi?resolution=320x240'

if len(sys.argv) > 1:
        if sys.argv[1] == "--help" or sys.argv[1] == "-h":
                syntax(sys.argv)
        stream_url   = sys.argv[1]
if len(sys.argv) > 2:
        topic_name   = sys.argv[2]
if len(sys.argv) > 3:
        jpeg_quality = sys.argv[3]
if len(sys.argv) > 4:
        show_gui     = sys.argv[4]




show_gui = True
#stream = urllib.urlopen('http://localhost:8080/frame.mjpg')
#stream = urllib.urlopen('http://vivotek-0:8080/video.mjpg')
#stream = urllib.urlopen('http://iris.not.iac.es/axis-cgi/mjpg/video.cgi?resolution=320x240')
stream = urllib.urlopen(stream_url)

rospy.init_node('mjpeg_stream_to_ros_topic', anonymous=True)
republisher = rospy.Publisher (topic_name + '/compressed', CompressedImage, queue_size = 1)


def signal_handler(sig, frame):
        print('Ctrl+C pressed, exiting...')
        sys.exit(0)

def jpeg_publisher(data):
        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        #msg.format = "rgb8; jpeg compressed bgr8"
        #msg.data = np.array(cv2.imencode('.jpg', self.last_img)[1]).tostring()
        msg.data = data.tostring()
        # Publish new image
        republisher.publish(msg)

def syntax(argv):
        print("")
        print("Syntax:")
        print("-------")
        print("\t", argv[0], "{--help,-h}\t\t\tShows this help")
        print("\t", argv[0], "[http://host:port/mjpeg-stream.mjpg] [mjpeg_topic] [(re)compression] [show_gui]")
        print("")
        print("Examples:")
        print("---------")
        print("\t", argv[0], "http://vivotek-0:8080/video.mjpg /vivotek_0_republished 40")
        print("\t", argv[0], "http://iris.not.iac.es/axis-cgi/mjpg/video.cgi?resolution=320x240 /a_random_ip_camera 90 show_gui")
        print("")
        sys.exit(0)



signal.signal(signal.SIGINT, signal_handler)
print('Press Ctrl+C to quit')
#signal.pause()

bytes = ''
while True:
    bytes += stream.read(1024)
    a = bytes.find('\xff\xd8')
    b = bytes.find('\xff\xd9')
    if a != -1 and b != -1:
        jpg = bytes[a:b+2]
        bytes = bytes[b+2:]
        #print(bytes)

	numpy_data = np.fromstring(jpg, dtype=np.uint8)
	if show_gui:
		i = cv2.imdecode(numpy_data, cv2.IMREAD_COLOR)
		cv2.imshow('img', i)
		if cv2.waitKey(1) == 27:
			exit(0)   

        jpeg_publisher(numpy_data)
