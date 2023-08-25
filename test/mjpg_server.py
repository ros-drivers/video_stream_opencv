#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from __future__ import print_function
import argparse
import cv2
import os
import rospy
import sys
import time
from BaseHTTPServer import BaseHTTPRequestHandler
from BaseHTTPServer import HTTPServer
from SocketServer import ThreadingMixIn


VIDEO_PATH = None


class MJPGStreamHandler(BaseHTTPRequestHandler):
    html_suffix = ['.html']
    mjpg_suffix = ['.mjpg', '.avi', '.mov']
    capture = None
    jpeg_quality = 80
    loop_rate = 20.0

    def do_GET(self):
        path, ext = os.path.splitext(self.path)
        if ext in self.html_suffix + self.mjpg_suffix:
            self.send_response(200)
            if ext in self.html_suffix:
                self.html_response()
            elif ext in self.mjpg_suffix:
                self.mjpg_response()
        else:
            self.send_response(404)

    def html_response(self):
        self.send_header('Content-type', 'text/html')
        self.end_headers()

        self.wfile.write('''
        <html>
          <head></head>
          <body>
            <img src="http://127.0.0.1:8080/camera.mjpg"/>
          </body>
        </html>''')

    def mjpg_response(self):
        self.send_header(
            'Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
        self.end_headers()

        if self.capture is None:
            self.capture = cv2.VideoCapture(VIDEO_PATH)

        nframes = self.capture.get(cv2.CAP_PROP_FRAME_COUNT)
        iframe = 0

        while True:
            try:
                ok, img = self.capture.read()
                if not ok:
                    continue
                iframe += 1
                if iframe > nframes:
                    iframe = 0
                    self.capture.open(VIDEO_PATH)
                ok, img = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                if not ok:
                    continue
                jpg = img.tostring()
                self.wfile.write('--jpgboundary')
                self.wfile.write(os.linesep)
                self.send_header('Content-type', 'image/jpeg')
                self.send_header('Content-length', str(len(jpg)))
                self.end_headers()
                self.wfile.write(jpg)
                self.wfile.write(os.linesep)
                time.sleep(1.0 / self.loop_rate)
            except KeyboardInterrupt:
                break


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    pass


def main():
    global VIDEO_PATH
    p = argparse.ArgumentParser()
    p.add_argument('video_path')
    p.add_argument('--port', default=8080, type=int)

    if rospy.myargv() == sys.argv:
        argv = sys.argv
        loginfo = print
    else:
        rospy.init_node('mjpg_server')
        argv = rospy.myargv()
        loginfo = rospy.loginfo


    args = p.parse_args(argv[1:])

    VIDEO_PATH = args.video_path
    if not os.path.exists(VIDEO_PATH):
        raise OSError('video not found: {}'.format(VIDEO_PATH))

    server = None
    try:
        server = ThreadedHTTPServer(('localhost', args.port), MJPGStreamHandler)
        loginfo('Server Started')
        server.serve_forever()
    except Exception as ex:
        loginfo('Got exception: {}'.format(str(ex)))
    finally:
        if server:
            server.socket.close()
        loginfo('Server Ended')


if __name__ == '__main__':
    main()
