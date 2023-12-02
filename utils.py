import subprocess
from djitellopy import Tello


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

    def __exit__(self, exc_type, exc_val, exc_tb):
        print('disconnecting from Tello')
        try:
            self.tello.streamoff()
            print("stream disabled")
        except Exception as e:
            print(f"didnt disable tello stream due to {e}")
        # reconnect to the previous wifi
        subprocess.call(f"nmcli con up {self.ssid}", shell=True)