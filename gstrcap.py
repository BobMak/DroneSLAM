from traceback import print_tb

import cv2
import gi
import numpy as np
import cv2, math, time

gi.require_version('Gst', '1.0')
from gi.repository import Gst

from utils import TelloContext


def main():
    # Initialize GStreamer
    Gst.init(None)
    # Define the GStreamer pipeline
    pipeline_str = (
        'udpsrc port=11111 '
        '! video/x-h264, stream-format=(string)byte-stream, width=(int)960, height=(int)720, framerate=(fraction)24/1, skip-first-bytes=2 '
        '! queue ! decodebin ! videoconvert ! video/x-raw,format=BGR '
        '! appsink emit-signals=True'
    )
    # Create the GStreamer pipeline
    pipeline = Gst.parse_launch(pipeline_str)
    # Create an appsink element to retrieve frames
    appsink = pipeline.get_by_name("appsink0")
    # buffer for the latest observations
    img_buff = np.zeros((980, 720, 3), dtype=np.uint8)
    # Function to retrieve frames from appsink
    def on_new_sample(appsink):
        sample = appsink.emit("pull-sample")
        if sample:
            buffer = sample.get_buffer()
            caps_format = sample.get_caps().get_structure(0)
            height = caps_format.get_value('height')
            width = caps_format.get_value('width')
            (result, mapinfo) = buffer.map(Gst.MapFlags.READ)
            if result:
                # Convert gstreamer data to OpenCV format
                buffer.unmap(mapinfo)
                img_buff = np.ndarray(
                    (height, width, 3),
                    buffer=mapinfo.data,
                    dtype=np.uint8
                )
            return Gst.FlowReturn.OK
    # Connect the callback to the appsink
    appsink.connect("new-sample", on_new_sample)
    # Start playing the pipeline
    pipeline.set_state(Gst.State.PLAYING)
    try:
        while True:
            frame = cv2.cvtColor(img_buff, cv2.COLOR_RGB2BGR)

            time.sleep(1)
            # cv2.imshow("cam", frame)
            # if cv2.waitKey(1000) & 0xFF == ord('q'):
            #     break
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print_tb(e.__traceback__)
    # Clean up
    pipeline.set_state(Gst.State.NULL)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    with open("telloid.txt", 'r') as f:
        telloid = f.readline()
    with TelloContext(tello_ssid=telloid) as tc:
        main()