#! /usr/bin/python
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

class HeightAboveTakeoffPub:
    def __init__(self):
        self.gps_sub = rospy.Subscriber('gps_fix', NavSatFix, self.gpsCallback, queue_size=1)
        self.height_pub = rospy.Publisher('height_above_takeoff', Float64, queue_size=1)
        self.init_altitude = None
        
    def gpsCallback(self, msg):
        if self.init_altitude == None:
            self.init_altitude = msg.altitude
        height_msg = Float64()
        height_msg.data = msg.altitude - self.init_altitude
        self.height_pub.publish(height_msg)

if __name__ == '__main__':
    
    rospy.init_node('height_above_takeoff_pub')
    h = HeightAboveTakeoffPub()
    rospy.spin()