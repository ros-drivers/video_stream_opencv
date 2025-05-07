#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@gmail.com>

import argparse
import os

import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import GLib, GObject, Gst, GstRtspServer

Gst.init(None)


class RTSPMediaFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, video_file, **properties):
        super().__init__(**properties)
        self.video_file = video_file

    def do_create_element(self, url):
        # Build the pipeline: read from file, decode, convert video, encode to H264, and pack in RTP H264 payload
        return Gst.parse_launch(' ! '.join([
            f'filesrc location={self.video_file}',
            'decodebin',
            'videoconvert',
            'x264enc speed-preset=ultrafast tune=zerolatency',
            'rtph264pay name=pay0 pt=96',
        ]))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', type=int, default=8554)
    p.add_argument('video', type=str)

    args, _ = p.parse_known_args()

    if not os.path.exists(args.video):
        p.error(f'Video {args.video} does not exist')

    server = GstRtspServer.RTSPServer()
    server.props.service = str(args.port)

    factory = RTSPMediaFactory(args.video)
    factory.set_shared(True)

    mount_points = server.get_mount_points()
    mount_points.add_factory('/', factory)

    server.attach(None)
    print(f'RTSP server is running at rtsp://localhost:{args.port}/')

    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        print('Server interrupted, exiting...')
        loop.quit()


if __name__ == "__main__":
    main()
