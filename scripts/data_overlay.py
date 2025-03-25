#!/usr/bin/env python3
import rospy
from robot_rms_msgs.msg import RMSStatus
from std_msgs.msg import String
from datetime import datetime

class DataOverlay:
    def __init__(self):
        # Publisher String
        self.pub = rospy.Publisher("/robot/aditional_information", String, queue_size=1)
        
        # Subscribe to RMSStatus
        rospy.Subscriber("/robot/rms/status", RMSStatus, self.status_callback)
    
    def status_callback(self, msg):
        # Get current time
        now = datetime.now().strftime("%H:%M:%S %m/%d/%y")
        # Create the string to publish
        combined_msg = f"{msg.current_poi}\n{now}"
        # Publish the string
        poi_msg = String()
        poi_msg.data = combined_msg
        self.pub.publish(poi_msg)

if __name__ == '__main__':
    rospy.init_node('data_overlay')
    republisher = DataOverlay()
    rospy.spin()