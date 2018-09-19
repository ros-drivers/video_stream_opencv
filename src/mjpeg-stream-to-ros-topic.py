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
import yaml
import argparse

from cv_bridge import CvBridge, CvBridgeError
#from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo

print(sys.version)
print(sys.argv)

#args.topic_name     = 'mjpeg_publisher'
#args.stream_url     = 'http://iris.not.iac.es/axis-cgi/mjpg/video.cgi?resolution=320x240'
#args.jpeg_quality   = 40
#args.est_image_size = 409600
#args.show_gui       = False
#args.verbose        = True
#args.caminfo_file   = '/home/biagio/.ros/camera_info/vivotek_IB8168_C.yaml'

#def syntax(argv):
#        print("")
#        print("Syntax:")
#        print("-------")
#        print("\t{} {}".format(argv[0], "{--help,-h}\t\t\t\t\t\t\t\t\t\t\t# Shows this help"))
#        print("\t{} {}".format(argv[0], "[http://host:port/mjpeg-stream.mjpg] [mjpeg_topic] [(re)compression quality 0-99] [socket_read_size] [show_gui]"))
#        print("")
#        print("Examples:")
#        print("---------")
#        print("\t{} {}".format(argv[0], "http://vivotek-0:8080/video.mjpg /vivotek_0 40"))
#        print("\t{} {}".format(argv[0], "http://iris.not.iac.es/axis-cgi/mjpg/video.cgi?resolution=320x240 /a_random_ip_camera 90 409600 show_gui\n\t\t\t\t\t\t\t\t# 500k may be the average size of a jpeg full HD frame @ 90% quality, while 8k maybe a 640x480 frame at 40% quality"))
#        print("\t{} {}".format(argv[0], "http://vivotek-0:8080/video3.mjpg /vivotek_0 0 4096\n\t\t\t\t\t\t\t\t# if compression quality is 0, only the original topic is published"))
#        print("")
#        sys.exit(0)
#
#
#
#if len(sys.argv) > 1:
#        if sys.argv[1] == "--help" or sys.argv[1] == "-h":
#                syntax(sys.argv)
#        stream_url     = sys.argv[1]
#if len(sys.argv) > 2:
#        topic_name     = sys.argv[2]
#if len(sys.argv) > 3:
#        jpeg_quality   = int(sys.argv[3])
#if len(sys.argv) > 4:
#        est_image_size = int(sys.argv[4])
#if len(sys.argv) > 5:
#	show_gui_str = sys.argv[5].lower()
#	if show_gui_str == "true" or show_gui_str == "1" or show_gui_str == "yes":
#		show_gui       = True


parser = argparse.ArgumentParser(description='Given a MJPEG HTTP stream, this node publishes a CompressedImage (and its CameraInfo) topics.')
parser.add_argument('stream_url', help='The MJPEG URL', metavar='http://vivotek-0:8080/video3.mjpg',
                              default='http://vivotek-0:8080/video.mjpg')
parser.add_argument('topic_name', help='The output topic', metavar='image',
                              default='vivotek_0')
parser.add_argument('-q', help='The jpeg quality for an optional republish', dest='jpeg_quality', type=int,
                        metavar='40')
parser.add_argument('est_image_size', help='The estimated image size, to optimize recv speed', type=int,
                        metavar='409600')
parser.add_argument('--show_gui', help='Show the images that are being received', dest='show_gui', type=bool,
                        metavar=False, default=False)
parser.add_argument('--verbose', help='Be more verbose', dest='verbose', type=bool,
                        metavar=False, default=False)
parser.add_argument('caminfo_file', help='Camera info file', metavar='~/.ros/camera_info/camera.yaml')

args = parser.parse_args()

print("URL: ", args.stream_url, "Output topic: ", args.topic_name, "Est. image size: ", args.est_image_size, "Camera info file: ", args.caminfo_file, "Jpeg quality: ", args.jpeg_quality, "Show GUI: ", args.show_gui, "Verbose: ", args.verbose)


def parse_calibration_yaml(calib_file):
    with file(calib_file, 'r') as f:
        params = yaml.load(f)

    cam_info = CameraInfo()
    cam_info.header.frame_id = params['camera_name']
    cam_info.height = params['image_height']
    cam_info.width = params['image_width']
    cam_info.distortion_model = params['distortion_model']
    cam_info.K = params['camera_matrix']['data']
    cam_info.D = params['distortion_coefficients']['data']
    cam_info.R = params['rectification_matrix']['data']
    cam_info.P = params['projection_matrix']['data']

    return cam_info



#stream = urllib.urlopen('http://localhost:8080/frame.mjpg')
#stream = urllib.urlopen('http://vivotek-0:8080/video.mjpg')
#stream = urllib.urlopen('http://iris.not.iac.es/axis-cgi/mjpg/video.cgi?resolution=320x240')
stream = urllib.urlopen(args.stream_url)

rospy.init_node('mjpeg_stream_to_ros_topic', anonymous=True)
mjpeg_publisher      = rospy.Publisher (args.topic_name + '/compressed'		, CompressedImage, queue_size = 1)
if args.jpeg_quality > 0:
	low_qual_republisher = rospy.Publisher (args.topic_name + '_low_qual' + '/compressed', CompressedImage, queue_size = 1)

cam_pub  = rospy.Publisher("camera_info", CameraInfo, queue_size = 1)
cam_info = parse_calibration_yaml(args.caminfo_file)

def signal_handler(sig, frame):
        print('Ctrl+C pressed, exiting...')
        sys.exit(0)

def jpeg_publisher(data, publisher, cam_pub = None):
        stamp = rospy.Time.now()
	if cam_pub is not None:
		cam_info.header.stamp = stamp
		# publish the camera info messages first
		cam_pub.publish(cam_info)
        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = stamp
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
    bytes += stream.read(args.est_image_size)
    if a == -1:
	a = bytes.find('\xff\xd8')
    if b == -1:
	b = bytes.find('\xff\xd9')
    if a != -1 and b != -1:
        jpg = bytes[a:b+2]
        bytes = bytes[b+2:]
	#print("----------------------------------------------------------", a, b, len(bytes))

	a = b = -1

	numpy_data = np.fromstring(jpg, dtype=np.uint8)
	if args.show_gui:
		i = cv2.imdecode(numpy_data, cv2.IMREAD_COLOR)
		cv2.imshow('img', i)
		if cv2.waitKey(1) == 27:
			exit(0)   

	if args.jpeg_quality > 0:
		i = cv2.imdecode(numpy_data, cv2.IMREAD_COLOR)
		retval, jpeg_data = cv2.imencode('.jpg', i, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
		if args.verbose:
			print("Successful recompression: {} - orig jpeg: {} - recompressed: {}".format(retval, numpy_data.size, jpeg_data.size))
		if retval:
			jpeg_publisher(jpeg_data, low_qual_republisher)

        jpeg_publisher(numpy_data, mjpeg_publisher, cam_pub)
