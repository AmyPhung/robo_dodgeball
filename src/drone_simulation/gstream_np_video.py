"""
This script is for grabbing the udp stream from the simulated drone for video streaming.

Original gst-launch command:

gst-launch-1.0  -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 sync=false
"""

# # For input into Gst init
# import sys

# # Set up gstreamer
# import gi
# gi.require_version('Gst', '1.0')
# gi.require_version('GstApp', '1.0')
# from gi.repository import Gst, GLib, GstApp

# # Initialize gstreamer
# Gst.init(sys.argv)

# # Set up the pipeline
# # pipeline = Gst.parse_launch("udpsrc port=5600 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264\" ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 sync=false")
# pipeline = Gst.parse_launch("udpsrc port=5600 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264\" ! rtph264depay ! avdec_h264 ! videoconvert ! appsink emit-signals=true")

# # Play the pipeline
# pipeline.set_state(Gst.State.PLAYING)

# help(pipeline)

# # Run mainloop
# loop = GLib.MainLoop()
# try:
#     loop.run()
# except:
#     loop.quit()

# https://github.com/jackersson/gst-python-tutorials/blob/master/launch_pipeline/run_appsink.py

import sys
import traceback
import argparse
import typing as typ
import time
import attr

import numpy as np
import cv2

from gstreamer import GstContext, GstPipeline, GstApp, Gst, GstVideo
import gstreamer.utils as utils

# Converts list of plugins to gst-launch string
# ['plugin_1', 'plugin_2', 'plugin_3'] => plugin_1 ! plugin_2 ! plugin_3
# "udpsrc port=5600 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264\" ! rtph264depay ! avdec_h264 ! videoconvert ! appsink emit-signals=true"
# DEFAULT_PIPELINE = utils.to_gst_string([
#     "udpsrc port=5600 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264\"",
#     "rtph264depay",
#     "avdec_h264",
#     "videoconvert",
#     "appsink emit-signals=True"
# ])

# DEFAULT_PIPELINE = "udpsrc port=5600 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264\" ! rtph264depay ! avdec_h264 ! videoconvert ! appsink emit-signals=true" #videotestsrc num-buffers=10 ! capsfilter caps=\"video/x-raw,format=RGB,width=640,height=480\" ! queue !
DEFAULT_PIPELINE = "udpsrc port=5600 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264\" ! rtph264depay ! avdec_h264 ! videoconvert ! queue ! appsink emit-signals=true"

# print(DEFAULT_PIPELINE)
# sys.exit()

ap = argparse.ArgumentParser()
ap.add_argument("-p", "--pipeline", required=False,
                default=DEFAULT_PIPELINE, help="Gstreamer pipeline without gst-launch")

args = vars(ap.parse_args())

command = args["pipeline"]

def extract_buffer(sample: Gst.Sample) -> np.ndarray:
    """Extracts Gst.Buffer from Gst.Sample and converts to np.ndarray"""

    buffer = sample.get_buffer()  # Gst.Buffer

    print(buffer.pts, buffer.dts, buffer.offset)

    caps_format = sample.get_caps().get_structure(0)  # Gst.Structure

    # GstVideo.VideoFormat
    video_format = GstVideo.VideoFormat.from_string(
        caps_format.get_value('format'))

    w, h = caps_format.get_value('width'), caps_format.get_value('height')
    c = utils.get_num_channels(video_format)

    print("width: ", w, " | height: ", h, " | channels: ", c)
    # 640, 360, -1

    buffer_size = buffer.get_size()
    shape = (h, w, c) if (h * w * c == buffer_size) else buffer_size
    print("shape of array: ", shape)
    array = np.ndarray(shape=shape, buffer=buffer.extract_dup(0, buffer_size),
                       dtype=utils.get_np_dtype(video_format))

    return np.squeeze(array)  # remove single dimension if exists


def on_buffer(sink: GstApp.AppSink, data: typ.Any) -> Gst.FlowReturn:
    """Callback on 'new-sample' signal"""
    # Emit 'pull-sample' signal
    # https://lazka.github.io/pgi-docs/GstApp-1.0/classes/AppSink.html#GstApp.AppSink.signals.pull_sample

    sample = sink.emit("pull-sample")  # Gst.Sample

    if isinstance(sample, Gst.Sample):
        array = extract_buffer(sample)
        # image = np.reshape(array, (640,-1,1) )

        subarray = array[0:640*360]
        image = np.reshape(subarray, (360,640,1))

        if appsink.FIRST_IMAGE:
            # TODO: Reshape array into an image
            if cv2.imwrite('color_img.jpg', image ):
                print("image saved successfully")
            else:
                print("image failed to save")
            appsink.FIRST_IMAGE = False

            # cv2.imshow("image", image)
            # cv2.waitKey(1)

        print(
            "Received {type} with shape {shape} of type {dtype}".format(type=type(array),
                                                                        shape=array.shape,
                                                                        dtype=array.dtype))
        return Gst.FlowReturn.OK

    return Gst.FlowReturn.ERROR


with GstContext():  # create GstContext (hides MainLoop)
    # create GstPipeline (hides Gst.parse_launch)
    with GstPipeline(command) as pipeline:
        appsink = pipeline.get_by_cls(GstApp.AppSink)[0]  # get AppSink
        # subscribe to <new-sample> signal
        appsink.FIRST_IMAGE = True
        # appsink.cap = cv2.VideoCapture(0)

        appsink.connect("new-sample", on_buffer, None)
        while not pipeline.is_done:
            time.sleep(.1)

