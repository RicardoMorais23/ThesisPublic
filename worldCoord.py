#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
#from visualization_msgs.msg import Marker, MarkerArray
import pymap3d as pm

class PointWorld(object):

    def __init__(self):

        # Coordinates Fault Point
        self.X_world = 0
        self.Y_world = 0
        self.Z_world = 0
        self.trans = 0
        self.rot = 0 

        # Coordinates Home position
        self.latHome = 34.0526511208
        self.longHome = 49.7918874648
        self.altHome = 0     
        
        #Publisher
        self.listener = tf.TransformListener()
        self.coordFaultLocal = rospy.Publisher("/fault/coordinate/local", PoseStamped, queue_size=1)
        self.coordFaultGlobal = rospy.Publisher("fault/coordinate/global", NavSatFix, queue_size=1)
        #self.marker_pub = rospy.Publisher("marker/faultPoint", MarkerArray, queue_size=1)

        # Subsciber
        self.subFlag = rospy.Subscriber("fault/coordinate/request", Bool, self.requestCallback)
        self.flag = False

    def requestCallback(self, msg):
        
        self.flag = msg.data
        

    def getCoordinates(self):
        
        (self.trans,self.rot) = self.listener.lookupTransform("map_ned", "point", rospy.Time(0))
        self.X_world = self.trans[0]
        self.Y_world = self.trans[1]
        self.Z_world = self.trans[2]
        # Publish fault point
        faultpoint = PoseStamped()
        faultpoint.header.stamp = rospy.Time.now()
        faultpoint.header.frame_id = "map_ned"
        faultpoint.pose.position.x = self.X_world
        faultpoint.pose.position.y = self.Y_world
        faultpoint.pose.position.z = self.Z_world
        faultpoint.pose.orientation.x = 0.0
        faultpoint.pose.orientation.y = 0.0
        faultpoint.pose.orientation.z = 0.0
        faultpoint.pose.orientation.w = 1.0
        # Convert ENU to GPS
        latPoint, longPoint, altPoint = self.enu2gps()
        # Publish gps fault coordinates
        gps_msg = NavSatFix()
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.header.frame_id = "map_ned"
        gps_msg.latitude = latPoint
        gps_msg.longitude = longPoint
        gps_msg.altitude = altPoint


        self.coordFaultLocal.publish(faultpoint)
        self.coordFaultGlobal.publish(gps_msg)        

        
        # Create a Marker to RVIZ
        #marker = Marker()
        #marker_array_msg = MarkerArray()
        #marker.header.stamp = rospy.get_rostime()
        #marker.header.frame_id = "map_ned"
        #marker.id = 1
        #marker.type = 2
        #marker.action = 0
        #marker.pose.position.x = self.X_world
        #marker.pose.position.y = self.Y_world
        #marker.pose.position.z = self.Z_world
		# marker.pose.orientation.x = tf.transform.rotation.x
		# marker.pose.orientation.y = tf.transform.rotation.y
		# marker.pose.orientation.z = tf.transform.rotation.z
		# marker.pose.orientation.w = tf.transform.rotation.w
        #marker.color.r = 1.0
        #marker.color.g = 0.0
        #marker.color.b = 0.0
        #marker.color.a = 1.0
        #marker.scale.x = 0.3
        #marker.scale.y = 0.3
        #marker.scale.z = 0.3# Print centroid coordinates
        #marker.lifetime.secs = 1
        #marker.frame_locked = False
        #marker.ns = "PointFault"
        #marker_array_msg.markers.append(marker)
        #self.marker_pub.publish(marker_array_msg)

    def enu2gps(self):
	    return pm.enu2geodetic(self.X_world, self.Y_world, self.Z_world, self.latHome, self.longHome, self.altHome)

def main():
    rospy.init_node("pointWorld_node")
    
    point_world = PointWorld()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        try:
            if point_world.flag == True:
                point_world.getCoordinates()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
           continue
        rate.sleep()

if __name__ == "__main__":
    main()
       