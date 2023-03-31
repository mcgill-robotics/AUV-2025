import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import cv2
import numpy as np

Gst.init(None)

def on_new_sample(sink, appsrc):
    print("new sample")
    sample = sink.emit("pull-sample")
    if sample:
        buf = sample.get_buffer()
        print(buf.get_size())
        result, mapinfo = buf.map(Gst.MapFlags.READ)
        if result:
            caps = sample.get_caps()
            structure = caps.get_structure(0)
            width = int(structure.get_value('width'))
            height = int(structure.get_value('height'))
            channels = 3
            arr = np.ndarray((height, width, channels), buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
            print(buf.extract())

        buf.unmap(mapinfo)


pipeline = Gst.Pipeline()

src = Gst.ElementFactory.make("nvarguscamerasrc", "camera-source")
src.set_property("sensor-id", 0)

caps_filter_1 = Gst.ElementFactory.make("capsfilter", "caps-filter-1")
caps_1 = Gst.Caps.from_string("video/x-raw(memory:NVMM),width=4032,height=3040,framerate=30/1")
caps_filter_1.set_property("caps", caps_1)

nvvidconv_1 = Gst.ElementFactory.make("nvvidconv", "nvvidconv-1")

caps_filter_2 = Gst.ElementFactory.make("capsfilter", "caps-filter-2")
caps_2 = Gst.Caps.from_string("video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1,format=I420")
caps_filter_2.set_property("caps", caps_2)

nvvidconv_2 = Gst.ElementFactory.make("nvvidconv", "nvvidconv-2")

videoconvert = Gst.ElementFactory.make("videoconvert", "video-convert")

sink = Gst.ElementFactory.make("appsink", "app-output")
sink.set_property("emit-signals", True)
sink.set_property("sync", False)
sink.set_property("max-buffers", 1)

pipeline.add(src)
pipeline.add(caps_filter_1)
pipeline.add(nvvidconv_1)
pipeline.add(caps_filter_2)
pipeline.add(nvvidconv_2)
pipeline.add(videoconvert)
pipeline.add(sink)

src.link(caps_filter_1)
caps_filter_1.link(nvvidconv_1)
nvvidconv_1.link(caps_filter_2)
caps_filter_2.link(nvvidconv_2)
nvvidconv_2.link(videoconvert)
videoconvert.link(sink)


sink.connect("new-sample", on_new_sample, src)

pipeline.set_state(Gst.State.PLAYING)

print("started stream")

try:
    loop = GLib.MainLoop()
    loop.run()
except KeyboardInterrupt:
    pipeline.set_state(Gst.State.NULL)
cv2.destroyAllWindows()

