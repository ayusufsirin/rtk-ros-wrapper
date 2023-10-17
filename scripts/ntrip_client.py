#!/usr/bin/env python3
import serial
import time
import argparse
import rospy

# https://github.com/semuconsulting/pynmeagps
# https://pypi.org/project/pygnssutils/
from pygnssutils.gnssntripclient import GNSSNTRIPClient, GGALIVE
from pynmeagps import NMEAReader


parser = argparse.ArgumentParser(description='NTRIP client for RTK serial devices.')
parser.add_argument('-s', '--serial-port', required=False, help='Serial port for the device (e.g. /dev/ttyUSB0)', default='/dev/ttyUSB1')
parser.add_argument('-b', '--baud-rate', type=int, required=False, help='Baud rate for the serial port (e.g. 115200)', default=115200)

args, _ = parser.parse_known_args()


class RTKModule:
    def __init__(self, serial: serial.Serial):
        self.lat = self.lon = self.alt = self.sep = 0.0
        self.nmr = NMEAReader(serial)

    def get_coordinates(self):
        (raw_data, parsed_data) = self.nmr.read()
        
        rospy.logwarn(f'NMEA: {raw_data}')
        
        if hasattr(parsed_data, 'msgID') and parsed_data.msgID == 'GGA':
            self.lat = parsed_data.lat
            self.lon = parsed_data.lon
            self.alt = parsed_data.alt
            self.sep = parsed_data.sep

        return None, self.lat, self.lon, self.alt, self.sep


def main():
    with serial.Serial(args.serial_port, baudrate=args.baud_rate, timeout=3) as ser:
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
            rospy.logwarn(f'Connected: {ntrpc.connected}, Streaming: {streaming}')
            time.sleep(2)


if __name__ == '__main__':
    main()
