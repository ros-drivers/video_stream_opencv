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

est_image_size = 409600

topic_name   = 'mjpeg_publisher'
stream_url   = 'http://iris.not.iac.es/axis-cgi/mjpg/video.cgi?resolution=320x240'
jpeg_quality = 40
show_gui     = False
verbose      = True

def syntax(argv):
        print("")
        print("Syntax:")
        print("-------")
        print("\t{} {}".format(argv[0], "{--help,-h}\t\t\t\t\t\t\t\t\t\t\t# Shows this help"))
        print("\t{} {}".format(argv[0], "[http://host:port/mjpeg-stream.mjpg] [mjpeg_topic] [(re)compression quality 0-99] [show_gui]"))
        print("")
        print("Examples:")
        print("---------")
        print("\t{} {}".format(argv[0], "http://vivotek-0:8080/video.mjpg /vivotek_0 40"))
        print("\t{} {}".format(argv[0], "http://iris.not.iac.es/axis-cgi/mjpg/video.cgi?resolution=320x240 /a_random_ip_camera 90 show_gui"))
        print("\t{} {}".format(argv[0], "http://vivotek-0:8080/video.mjpg /vivotek_0 0\t\t\t\t\t# if compression quality is 0, only the original topic is published"))
        print("")
        sys.exit(0)



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




#stream = urllib.urlopen('http://localhost:8080/frame.mjpg')
#stream = urllib.urlopen('http://vivotek-0:8080/video.mjpg')
#stream = urllib.urlopen('http://iris.not.iac.es/axis-cgi/mjpg/video.cgi?resolution=320x240')
stream = urllib.urlopen(stream_url)

rospy.init_node('mjpeg_stream_to_ros_topic', anonymous=True)
mjpeg_publisher      = rospy.Publisher (topic_name + '/compressed'		, CompressedImage, queue_size = 1)
low_qual_republisher = rospy.Publisher (topic_name + '_low_qual' + '/compressed', CompressedImage, queue_size = 1)


def signal_handler(sig, frame):
        print('Ctrl+C pressed, exiting...')
        sys.exit(0)

def jpeg_publisher(data, publisher):
        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        #msg.format = "rgb8; jpeg compressed bgr8"
        #msg.data = np.array(cv2.imencode('.jpg', self.last_img)[1]).tostring()
        msg.data = data.tostring()
        # Publish new image
        publisher.publish(msg)



signal.signal(signal.SIGINT, signal_handler)
print('Press Ctrl+C to quit')

bytes = ''
a = b = -1
while True:
    bytes += stream.read(est_image_size)
    if a == -1:
	a = bytes.find('\xff\xd8')
    if b == -1:
	b = bytes.find('\xff\xd9')
    if a != -1 and b != -1:
        jpg = bytes[a:b+2]
        bytes = bytes[b+2:]

	a = b = -1

	numpy_data = np.fromstring(jpg, dtype=np.uint8)
	if show_gui:
		i = cv2.imdecode(numpy_data, cv2.IMREAD_COLOR)
		cv2.imshow('img', i)
		if cv2.waitKey(1) == 27:
			exit(0)   

	if int(jpeg_quality) > 0:
		i = cv2.imdecode(numpy_data, cv2.IMREAD_COLOR)
		retval, jpeg_data = cv2.imencode('.jpg', i, [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)])
		if verbose:
			print("Successful recompression: {} - orig jpeg: {} - recompressed: {}".format(retval, numpy_data.size, jpeg_data.size))
		if retval:
			jpeg_publisher(jpeg_data, low_qual_republisher)

        jpeg_publisher(numpy_data, mjpeg_publisher)
