#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
import simplekml
import math
from datetime import datetime


SCALED_LLH_LL_SCALER = 536870912
SCALED_LLH_H_SCALER = 16384


class GpsToKml:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('gps_to_kml', anonymous=True)

        # Subscribe to the GPS topic
        self.subscriber = rospy.Subscriber('/scaled_llh', NavSatFix, self.callback)

        # Create a KML object
        self.kml = simplekml.Kml()
        self.path_points = []
        self.waypoints_folder = self.kml.newfolder(name="Waypoints")

        # On shutdown, write data to KML
        rospy.on_shutdown(self.save_kml_file)
        
        self.counter = 0

    def callback(self, data):
        print(data)
        
        data.latitude = data.latitude / SCALED_LLH_LL_SCALER * 180 / math.pi
        data.longitude = data.longitude / SCALED_LLH_LL_SCALER * 180 / math.pi
        data.altitude = data.altitude / SCALED_LLH_H_SCALER
        
        # TODO: Remove this constraints after checksum control implementation
        if not (26 <= data.longitude <= 45 and 36 <= data.latitude <= 42): 
            return

        self.counter += 1
        
        # Add latitude and longitude from the received message to the path points
        self.path_points.append((data.longitude, data.latitude, data.altitude))
        
        if self.counter < 20:
            return

        # Create a Placemark for the received GPS coordinates
        placemark = self.waypoints_folder.newpoint(
            name="Waypoint",
            coords=[(data.longitude, data.latitude, data.altitude)]
        )
        self.counter = 0

    def save_kml_file(self):
        # Create a KML LineString with the captured path points
        linestring = self.kml.newlinestring(name="GPS Path", coords=self.path_points)
        
        # Save the KML file
        self.kml.save(rospy.get_param('~output_file', datetime.now().strftime('gps_path_%Y-%m-%d-%H-%M-%S.kml')))

        rospy.loginfo("Saved KML file.")

if __name__ == '__main__':
    try:
        kml_converter = GpsToKml()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
