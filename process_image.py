#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
import tf, tf2_ros
import cv2
from cv_bridge import CvBridge
import numpy as np


class ImageProcess(object):

	def __init__(self):
		# Publisher
		self.image_pub = rospy.Publisher("/standard_vtol/camera/imageProcess", Image, queue_size=1)
		self.marker_pub = rospy.Publisher("/marker/centroid", MarkerArray, queue_size=1)
		self.pubFlag = rospy.Publisher("fault/coordinate/request", Bool, queue_size=1)
		# Subscriber
		self.subImageRGB = rospy.Subscriber("/standard_vtol/camera/rgb/image_raw/compressed", CompressedImage, self.image_callback)
		self.subImageDepth = rospy.Subscriber("/standard_vtol/camera/depth/image_raw", Image, self.image_depth_callback)
		# Variables
		self.fx = 454.6857718666893
		self.fy = 454.6857718666893
		self.Cx = 424.5
		self.Cy = 240.5
		self.Centroidx = 0
		self.Centroidy = 0
		self.depth_value = 0
		self.X_cam = 0
		self.Y_cam = 0
		self.Z_cam = 0 
		# Flag fault point
		self.flag_convertP = False

	def image_depth_callback(self, msg):

		bridge = CvBridge()
		depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
		
		# Get the depth value of centroid
		self.depth_value = depth_image[self.Centroidy, self.Centroidx]
		print("Depth value :",self.depth_value)

	def processPoint(self):
		# Get the point of intress 
		x_point = (self.Centroidx - self.Cx) / self.fx
		y_point = (self.Centroidy - self.Cy) / self.fy

		self.X_cam = x_point * self.depth_value
		self.Y_cam = y_point * self.depth_value
		self.Z_cam = self.depth_value

		# Create a Marker to RVIZ
		marker = Marker()
		marker_array_msg = MarkerArray()
		marker.header.stamp = rospy.get_rostime()
		marker.header.frame_id = "camera_link"
		marker.id = 1
		marker.type = 2
		marker.action = 0# Print centroid coordinates
		#print("Centroid coordinates (Cx, Cy):", Centroidx, Centroidy)
		marker.pose.position.x = self.X_cam
		marker.pose.position.y = self.Y_cam
		marker.pose.position.z = self.Z_cam
		# marker.pose.orientation.x = tf.transform.rotation.x
		# marker.pose.orientation.y = tf.transform.rotation.y
		# marker.pose.orientation.z = tf.transform.rotation.z
		# marker.pose.orientation.w = tf.transform.rotation.w
		marker.color.r = 1.0
		marker.color.g = 0.0
		marker.color.b = 0.0
		marker.color.a = 1.0
		marker.scale.x = 0.3
		marker.scale.y = 0.3
		marker.scale.z = 0.3# Print centroid coordinates
		#print("Centroid coordinates (Cx, Cy):", Centroidx, Centroidy)
		marker.lifetime.secs = 1
		marker.frame_locked = False
		marker.ns = "centroid"
		marker_array_msg.markers.append(marker)
		self.marker_pub.publish(marker_array_msg)

	def publish_tf_cam_point(self):
		#tf_msg = TFMessage()

		transform_s = TransformStamped()
		transform_s.header.stamp = rospy.Time.now()
		transform_s.header.frame_id = "camera_link"
		transform_s.child_frame_id = "point"

		#Translation
		transform_s.transform.translation.x = self.X_cam
		transform_s.transform.translation.y = self.Y_cam
		transform_s.transform.translation.z = self.Z_cam
		#Rotation
		q = tf.transformations.quaternion_from_euler(0, 0, 0)
		transform_s.transform.rotation.x = q[0]
		transform_s.transform.rotation.y = q[1]
		transform_s.transform.rotation.z = q[2]
		transform_s.transform.rotation.w = q[3]
		# Pub /tf_
		static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
		static_tf_broadcaster.sendTransform(transform_s)
	

		
	def image_callback(self, msg):

		bridge = CvBridge()
		flag_msg = Bool()
		np_arr = np.frombuffer(msg.data, np.uint8)
		image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		
		# Convert to HSV
		image_hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)
		#Threshold the HSV image, keep only the red pixels
		lower_red = cv2.inRange(image_hsv, (0, 100, 100), (20, 255, 255))
		upper_red = cv2.inRange(image_hsv, (160, 100, 100), (179, 255, 255))
		# Combine the above two images
		red_image = cv2.addWeighted(lower_red, 1.0, upper_red, 1.0, 0.0)
		# Gaussian Blur
		image_gaussian_blur = cv2.GaussianBlur(red_image, (5, 5), 1.5)
		
		# Calculate the moments
		moments = cv2.moments(image_gaussian_blur)
		
		try:
			self.Centroidx = int(moments['m10'] / moments['m00'])
			self.Centroidy = int(moments['m01'] / moments['m00'])
			
			self.processPoint()
			self.publish_tf_cam_point()
			self.flag_convertP = True
			flag_msg.data = self.flag_convertP
			self.pubFlag.publish(flag_msg)
			#Print centroid coordinates
			#print("Centroid coordinates (Cx, Cy):", self.Centroidx, self.Centroidy)
			# Draw a poin
			radius = 5  # adjust the radius size as needed
			color = (0, 255, 0)  # color of the circle 
			thickness = 2  # thickness of the circle boundary
			cv2.circle(image_gaussian_blur, (self.Centroidx, self.Centroidy), radius, color, thickness)
			
		
		except ZeroDivisionError as e:
			self.flag_convertP = False
			flag_msg.data = self.flag_convertP
			self.pubFlag.publish(flag_msg)
			print("Skip Image Processing")

		img_msg = bridge.cv2_to_imgmsg(image_gaussian_blur, encoding="mono8")

		img_msg.header.stamp = rospy.Time.now()

		self.image_pub.publish(img_msg)


def main():
    rospy.init_node("image_processing_node")
    image_process = ImageProcess() 

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
     
        #image_process.processPoint()
        rate.sleep()

if __name__ == "__main__":
    main()