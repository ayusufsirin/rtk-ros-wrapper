#!/usr/bin/env python3
import serial
import struct
import rospy
import argparse
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, UInt32
from geometry_msgs.msg import Twist


parser = argparse.ArgumentParser(description='Read data from a serial device and publish to ROS topics.')
parser.add_argument('-s', '--serial-port', required=False, help='Serial port for the device (e.g. /dev/ttyUSB0)', default='/dev/ttyUSB0')
parser.add_argument('-b', '--baud-rate', type=int, required=False, help='Baud rate for the serial port (e.g. 115200)', default=115200)

args, _ = parser.parse_known_args()


data_fields = {
    0x0001: ("Acceleration(XYZ)", 12, "fff"),
    0x0002: ("Gyro(XYZ)", 12, "fff"),
    0x0004: ("Magnetometer(XYZ)", 12, "fff"),
    0x0008: ("Latitude-Longitude-Elevation", 12, "fff"),
    0x0010: ("Roll-Pitch-Yaw", 12, "fff"),
    0x0020: ("Pressure-Temperature-BaroHeight", 12, "fff"),
    0x0040: ("Time", 4, "f"),
    0x0080: ("Status", 1, "B"),
    0x0100: ("Velocity", 12, "fff"),
    0x0200: ("Scaled-LLh", 12, "iii"),
    0x1000: ("Low-Rate-Messages", 12, "")
}

def process_serial_data(ser: serial.Serial, callback):
    while True:
        # Buffer to hold the current message
        message = bytearray()

        # Read until we find a preamble
        byte = ser.read(1)
        while byte != b'\x3D':
            byte = ser.read(1)

        # We found a preamble, so start a new message
        message += byte

        # Read Message ID (3 bytes)
        for _ in range(3):
            message += ser.read(1)

        # Ignore the last byte of Message ID, as it's set to 0
        message_id = int.from_bytes(message[1:3], 'big')

        data = {
            'Preamble': message[:1].hex(),
            'Message ID': message[1:4].hex(),
        }

        # Parse data fields according to message_id
        data_start = 4
        for bit_position, (field_name, field_length, field_format) in data_fields.items():
            if message_id & bit_position:
                data_end = data_start + field_length
                field_data = ser.read(field_length)
                message += field_data

                # Unpack the data according to the provided format
                unpacked_data = struct.unpack('<' + field_format, field_data) if field_format else field_data.hex()
                data[field_name] = unpacked_data

                data_start = data_end

        # Read CRC (2 bytes)
        for _ in range(2):
            message += ser.read(1)
        data['CRC'] = message[-2:].hex()

        # Call the callback with the parsed data
        callback(data)


# Initialize ROS node
rospy.init_node('my_rtk_node')

# Create publishers
imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
gps_pub = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)
time_pub = rospy.Publisher('device_time', Float32, queue_size=10)
status_pub = rospy.Publisher('status', UInt32, queue_size=10)
velocity_pub = rospy.Publisher('velocity', Twist, queue_size=10)
scaled_llh_pub = rospy.Publisher('scaled_llh', NavSatFix, queue_size=10)


def my_callback(parsed_data):
    # Check if IMU data is present
    if "Acceleration(XYZ)" in parsed_data and "Gyro(XYZ)" in parsed_data:
        # Create and publish IMU message
        imu_msg = Imu()
        imu_msg.header.frame_id = 'map'
        imu_msg.linear_acceleration.x = parsed_data["Acceleration(XYZ)"][0]
        imu_msg.linear_acceleration.y = parsed_data["Acceleration(XYZ)"][1]
        imu_msg.linear_acceleration.z = parsed_data["Acceleration(XYZ)"][2]
        imu_msg.angular_velocity.x = parsed_data["Gyro(XYZ)"][0]
        imu_msg.angular_velocity.y = parsed_data["Gyro(XYZ)"][1]
        imu_msg.angular_velocity.z = parsed_data["Gyro(XYZ)"][2]
#        print("Acceleration")
        imu_pub.publish(imu_msg)

    # Check if GPS data is present
    if "Latitude-Longitude-Elevation" in parsed_data:
        # Create and publish GPS message
        gps_msg = NavSatFix()
        gps_msg.header.frame_id = 'map'
        gps_msg.latitude = parsed_data["Latitude-Longitude-Elevation"][0]
        gps_msg.longitude = parsed_data["Latitude-Longitude-Elevation"][1]
        gps_msg.altitude = parsed_data["Latitude-Longitude-Elevation"][2]
#        print("GPS")
        gps_pub.publish(gps_msg)

    # Check if Time data is present
#    if "Time" in parsed_data:
 #       print("Time")
#        time_msg = Float32()
#        time_msg.data = parsed_data["Time"]
#        time_pub.publish(time_msg)

    # Check if Status data is present
    if "Status" in parsed_data:
 #       print("Status")
        status_msg = UInt32()
        status_msg.data = int.from_bytes(parsed_data["Status"], 'little')
        status_pub.publish(status_msg)

    # Check if Velocity data is present
    if "Velocity" in parsed_data:
        # Create and publish velocity message
        velocity_msg = Twist()
        velocity_msg.linear.x = parsed_data["Velocity"][0]
        velocity_msg.linear.y = parsed_data["Velocity"][1]
        velocity_msg.linear.z = parsed_data["Velocity"][2]
 #       print("Velocity")
        velocity_pub.publish(velocity_msg)

    # Check if Scaled-LLH data is present
    if "Scaled-LLh" in parsed_data:
        # Create and publish Scaled-LLH message
        scaled_llh_msg = NavSatFix()
        scaled_llh_msg.header.frame_id = 'map'
        scaled_llh_msg.latitude = parsed_data["Scaled-LLh"][0]
        scaled_llh_msg.longitude = parsed_data["Scaled-LLh"][1]
        scaled_llh_msg.altitude = parsed_data["Scaled-LLh"][2]
 #       print("Scaled-LLh")
        scaled_llh_pub.publish(scaled_llh_msg)

ser = serial.Serial(args.serial_port, baudrate=args.baud_rate)
process_serial_data(ser, my_callback)
