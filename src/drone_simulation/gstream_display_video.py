"""
This script is for grabbing the udp stream from the simulated drone for video streaming.

Original gst-launch command:

gst-launch-1.0  -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 sync=false
"""

# For input into Gst init
import sys

# Set up gstreamer
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initialize gstreamer
Gst.init(sys.argv)

# Set up the pipeline
pipeline = Gst.parse_launch("udpsrc port=5600 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264\" ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 sync=false")

# Play the pipeline
pipeline.set_state(Gst.State.PLAYING)

help(pipeline)

# Run mainloop
loop = GLib.MainLoop()
try:
    loop.run()
except:
    loop.quit()