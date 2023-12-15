import subprocess
import sys
import os
import torch
import cv2
from torchvision.transforms import Compose

from djitellopy import Tello
sys.path.append("DPT")
import util.io
from dpt.models import DPTDepthModel
from dpt.midas_net import MidasNet_large
from dpt.transforms import Resize, NormalizeImage, PrepareForNet


class TelloContext:
    def __init__(self, tello_ssid):
        self.tello_ssid = tello_ssid
        # remember the current wifi
        self.ssid = subprocess.check_output("iwgetid -r", shell=True).decode().strip()
        print(f"current wifi: {self.ssid}")
        # initialize the tello
        self.tello = None

    def __enter__(self):
        # connect to the tello wifi (assuming the connection was previously established)
        ret = subprocess.call(f"nmcli con up {self.tello_ssid}", shell=True, )
        assert ret==0, "failed to connect to tello"
        print(f"Connected to {self.tello_ssid}")
        self.tello = Tello()
        self.tello.send_control_command('command')
        try:
            self.tello.streamon()
        except Exception as e:
            print(f"failed to enable stream from the drone, {e}")
            raise Exception()
        return self

    def get_current_state(self):
        return self.tello.get_current_state()

    def __exit__(self, exc_type, exc_val, exc_tb):
        print('disconnecting from Tello')
        try:
            self.tello.streamoff()
            print("stream disabled")
        except Exception as e:
            print(f"didnt disable tello stream due to {e}")
        # reconnect to the previous wifi
        subprocess.call(f"nmcli con up {self.ssid}", shell=True)


class DPTModel:
    def __init__(self, model_path, model_type="dpt_hybrid", optimize=True, output_path="data"):
        print("initialize")
        # select device
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print("device: %s" % device)
        # load network
        if model_type == "dpt_hybrid":  # DPT-Hybrid
            net_w = net_h = 384
            model = DPTDepthModel(
                path=model_path,
                backbone="vitb_rn50_384",
                non_negative=True,
                enable_attention_hooks=False,
            )
            normalization = NormalizeImage(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
        elif model_type == "dpt_hybrid_nyu":
            net_w = 640
            net_h = 480
            model = DPTDepthModel(
                path=model_path,
                scale=0.000305,
                shift=0.1378,
                invert=True,
                backbone="vitb_rn50_384",
                non_negative=True,
                enable_attention_hooks=False,
            )
            normalization = NormalizeImage(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
        transform = Compose(
            [
                Resize(
                    net_w,
                    net_h,
                    resize_target=None,
                    keep_aspect_ratio=True,
                    ensure_multiple_of=32,
                    resize_method="minimal",
                    image_interpolation_method=cv2.INTER_CUBIC,
                ),
                normalization,
                PrepareForNet(),
            ]
        )
        model.eval()
        if optimize == True and device == torch.device("cuda"):
            model = model.to(memory_format=torch.channels_last)
            model = model.half()
        model.to(device)
        self.model = model
        self.transform = transform
        self.device = device
        self.optimize = optimize
        self.model_type = model_type
        self.output_path = output_path

    def run(self, img_buff, ep_uid, sensor_data, timestamp_ns):
        """Run MonoDepthNN to compute depth maps."""
        img = cv2.cvtColor(img_buff, cv2.COLOR_BGR2RGB)
        img_input = self.transform({"image": img})["image"]
        # compute
        with torch.no_grad():
            sample = torch.from_numpy(img_input).to(self.device).unsqueeze(0)
            if self.optimize == True and self.device == torch.device("cuda"):
                sample = sample.to(memory_format=torch.channels_last)
                sample = sample.half()
            prediction = self.model.forward(sample)
            prediction = (
                torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=img.shape[:2],
                    mode="bicubic",
                    align_corners=False,
                ).squeeze().cpu().numpy()
            )
            if self.model_type == "dpt_hybrid_nyu":
                prediction *= 1000.0

        filename = os.path.join(
            self.output_path, str(timestamp_ns)
        )
        util.io.write_depth(filename, prediction, bits=2, absolute_depth=False)
