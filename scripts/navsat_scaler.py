import rospy
from sensor_msgs.msg import NavSatFix

class GPSConverter:
    def __init__(self):
        self.pub = rospy.Publisher('/modified_navsat_topic', NavSatFix, queue_size=10)
        rospy.Subscriber('/scaled_llh', NavSatFix, self.navsat_callback)
        
    def navsat_callback(self, original_msg):
        if not (37 <= original_msg.latitude / 1e7 <= 38 and 30 <= original_msg.longitude / 1e7 <= 31): 
            return
        
        modified_msg = NavSatFix()
        modified_msg.header = original_msg.header
        modified_msg.status = original_msg.status
        modified_msg.latitude = original_msg.latitude / 1e7
        modified_msg.longitude = original_msg.longitude / 1e7
        modified_msg.altitude = original_msg.altitude / 1e4
        modified_msg.position_covariance = original_msg.position_covariance
        modified_msg.position_covariance_type = original_msg.position_covariance_type
        
        self.pub.publish(modified_msg)

if __name__ == "__main__":
    rospy.init_node('navsat_converter')
    gps_converter = GPSConverter()
    rospy.spin()

