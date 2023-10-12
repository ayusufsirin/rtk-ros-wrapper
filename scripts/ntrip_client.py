# https://github.com/semuconsulting/pynmeagps
# https://pypi.org/project/pygnssutils/

import serial
import time
from pygnssutils.gnssntripclient import GNSSNTRIPClient, GGALIVE
from pynmeagps import NMEAReader


SERIAL_DEVICE = '/dev/ttyUSB0'
BAUD_RATE = 115200


class RTKModule:
    def __init__(self, serial: serial.Serial):
        self.lat = self.lon = self.alt = self.sep = 0.0
        self.nmr = NMEAReader(serial)

    def get_coordinates(self):
        (_, parsed_data) = self.nmr.read()
        if hasattr(parsed_data, 'msgID') and parsed_data.msgID == 'GGA':
            self.lat = parsed_data.lat
            self.lon = parsed_data.lon
            self.alt = parsed_data.alt
            self.sep = parsed_data.sep

        return None, self.lat, self.lon, self.alt, self.sep


def main():
    with serial.Serial(SERIAL_DEVICE, BAUD_RATE, timeout=3) as ser:
        ntripc_settings = {
            "server": "212.156.70.42",
            "port": 2101,
            "mountpoint": "ORTA_TG20_BROADCASTRTCM",
            "ntripuser": "K0706071701",
            "ntrippassword": "maQjEp",
            "ggainterval": 60,
            "ggamode": GGALIVE,
            "output": ser,
        }

        rtk = RTKModule(serial=ser)
        ntrpc = GNSSNTRIPClient(app=rtk, logtofile=True,)

        streaming = ntrpc.run(**ntripc_settings)

        while True:
            print(f'Connected: {ntrpc.connected}, Streaming: {streaming}')
            time.sleep(2)


if __name__ == '__main__':
    main()
