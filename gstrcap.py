import sys
import os
import uuid
from traceback import print_tb
import argparse

import gi
import numpy as np
import cv2, math, time

gi.require_version('Gst', '1.0')
from gi.repository import Gst

from utils import TelloContext, DPTModel

sys.path.append("DPT")
from DPT.run_monodepth import run


# callback for recording the observations
def save_obs(img_buff, ep_uid, sensor_data, timestamp_ns):
    # save the latest observation
    cv2.imwrite(f"data/{ep_uid}/image/{timestamp_ns}.jpg", img_buff)
    # parse sensor data
    data = [x for k, x in sensor_data.items()]
    # log the timestamp
    with open(f"data/{ep_uid}/data.csv", "a") as f:
        f.write(','.join([str(x) for x in [timestamp_ns, *data]])+'\n')


def stream_tello_data(ep_uid, tc, cbs:list=()):
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
    img_buff = np.zeros((3, 720, 960, 3), dtype=np.uint8)
    # create a data folder if it doesn't exist
    os.makedirs(f"data/{ep_uid}/image", exist_ok=True)
    os.makedirs(f"data/{ep_uid}/depth", exist_ok=True)
    # episode meta data log file
    if not os.path.exists("data/episode_meta.csv"):
        with open("data/episode_meta.csv", "w") as f:
            f.write("ep_uid,timestamp\n")
    with open("data/episode_meta.csv", "a") as f:
        f.write(f"{ep_uid},{time.time()}\n")
    # data log file
    with open(f"data/{ep_uid}/data.csv", "w") as f:
        f.write("timestamp,pitch,roll,yaw,vgx,vgy,vgz,templ,temph,tof,h,bat,baro,time,agx,agy,agz\n")

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
                np.roll(img_buff, 1)
                img_buff[0] = np.ndarray(
                    (height, width, 3),
                    buffer=mapinfo.data,
                    dtype=np.uint8
                )
                # save the observation
                timestamp_ns = time.time_ns()
                sensor_data = tc.get_current_state()
                for cb in cbs:
                    cb(img_buff[0], ep_uid, sensor_data, timestamp_ns)
            return Gst.FlowReturn.OK
    # Connect the callback to the appsink
    appsink.connect("new-sample", on_new_sample)
    # Start playing the pipeline
    pipeline.set_state(Gst.State.PLAYING)
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print_tb(e.__traceback__)
    # Clean up
    pipeline.set_state(Gst.State.NULL)
    cv2.destroyAllWindows()
    return ep_uid


def stream_webcam(ep_uid, cam_id, cbs:list=()):
    cap = cv2.VideoCapture(cam_id)
    # create a data folder if it doesn't exist
    os.makedirs(f"data/{ep_uid}/image", exist_ok=True)
    os.makedirs(f"data/{ep_uid}/depth", exist_ok=True)
    # episode meta data log file
    if not os.path.exists("data/episode_meta.csv"):
        with open("data/episode_meta.csv", "w") as f:
            f.write("ep_uid,timestamp\n")
    with open("data/episode_meta.csv", "a") as f:
        f.write(f"{ep_uid},{time.time()}\n")
    start_time = time.time()
    while True:
        print('recording {}s'.format(time.time()-start_time))
        _, img_buff = cap.read()
        # save the observation
        timestamp_ns = time.time_ns()
        sensor_data = dict()
        for cb in cbs:
            cb(img_buff, ep_uid, sensor_data, timestamp_ns)
        # cv2.imshow("frame", img_buff)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    return ep_uid


if __name__ == "__main__":
    args = argparse.ArgumentParser()
    args.add_argument("--type", type=str, default="webcam", choices=["drone", "webcam"])
    args.add_argument("--telloid", type=str, default=None)
    args.add_argument("--cam", type=str, default="/dev/video0")
    args = args.parse_args()
    telloid = args.telloid
    ep_uid = str(uuid.uuid4())
    if args.type == "webcam":
        dpt = DPTModel(
            "dpt_hybrid_nyu-2ce69ec7.pt", "dpt_hybrid_nyu", True,
            output_path=os.path.join("data", ep_uid, "depth"))
        print('starting webcam')
        stream_webcam(ep_uid, args.cam, cbs=[save_obs, dpt.run])
    else:
        with TelloContext(tello_ssid=telloid) as tc:
            dpt = DPTModel(
                 "dpt_hybrid_nyu-2ce69ec7.pt", "dpt_hybrid_nyu", True,
                output_path=os.path.join("data", ep_uid, "depth"))
            stream_tello_data(ep_uid, tc, cbs=[save_obs, dpt.run])  # dpt.run