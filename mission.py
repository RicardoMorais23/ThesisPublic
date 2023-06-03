#!/bin/env python

import rospy
import math
import pymap3d as pm
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from mavros_msgs.msg import PositionTarget
from mission_vtol.srv import waypoint_service

class pathMission(object):

    def __init__(self):
        # Vtol
        self.x_Vtol = 0
        self.y_Vtol = 0
        self.z_Vtol = 0
        self.roll_vtol = 0
        self.pitch_vtol = 0
        self.yaw_vtol = 0
        self.linearVel_x = 0
        self.linearVel_y = 0
        self.maxSpeed = 5 #(m/s)
        # Control Waypoint
        self.kp_acel = 0.4
        self.cont_reachedWayp = 0
        self.xVtol_c2wp = 0
        self.yVtol_c2wp = 0
        self.zVtol_c2wp = 0
        # Go to Point 
        self.distVtol_WayP = 0
        self.dist_tolerance = 5 #(meters)
        self.radiusWayp = 2 #(meters)
        self.yawPoint = 0
        # Coordinates Home position (reference)
        self.latHome = 34.0526511208
        self.longHome = 49.7918874648
        self.altHome = 0 
        # Coordinates Waypoint (GPS)  
        self.latWayP = 0
        self.longWayP = 0
        self.altWayP = 0
        self.numWayP = 0
        # Coordinates Waypoint (ENU)
        self.x_wayP = 0
        self.y_wayP = 0
        self.z_wayP = 0
        # Auxiliar variables
        self.waypoints = []
        # Flag to request new Waypoint
        self.waypointRequest = False
        # Flag to check finish mission
        self.finishedMission = False
        rospy.wait_for_service('sendWaypoint')
        self.service = rospy.ServiceProxy('sendWaypoint', waypoint_service)
        # Subscribe VTOL Position
        self.poseVtol = rospy.Subscriber('/mavros/global_position/local', Odometry, self.subPoseVtol)
        # Subscribe VTOL Orientation
        self.oritVtol = rospy.Subscriber('/mavros/imu/data', Imu, self.subYawVtol)
        # Setpoint
        self.pubSetpoint = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    def subPoseVtol(self, msg):
        # Get the Vtol Pose
        self.x_Vtol = msg.pose.pose.position.x
        self.y_Vtol = msg.pose.pose.position.y
        self.z_Vtol = msg.pose.pose.position.z

    def subYawVtol(self, msg):
        # Get the Vtol yaw
        x = msg.orientation.x  
        y = msg.orientation.y 
        z = msg.orientation.z 
        w = msg.orientation.w 
        orientation_list = [x, y, z, w]
        (self.roll_vtol, self.pitch_vtol, self.yaw_vtol) = euler_from_quaternion(orientation_list)
    
    def gps2enu(self):
        return pm.geodetic2enu(self.latWayP, self.longWayP, self.altWayP, self.latHome, self.longHome, self.altHome)
    
    def convertWaypoint(self):
        
        resp = self.service(self.waypointRequest)
        self.finishedMission = resp.finished
        # if not finished
        if self.finishedMission == False:
            self.latWayP = resp.latitude
            self.longWayP = resp.longitude
            self.altWayP = resp.altitude
            self.numWayP = resp.numWayP
            self.counter = resp.counter
		    # Convert GPS to ENUpara 
            self.x_wayP, self.y_wayP, self.z_wayP = self.gps2enu()
            #rospy.loginfo("New waypoint requested")
            if self.waypointRequest == True:
                self.waypoints.append((self.x_wayP,self.y_wayP,self.z_wayP))
                 
                if self.counter >= 1:
                    # Calculate the direction vector between the two points
                    direction_vector = np.array([self.waypoints[self.counter][0] - self.waypoints[self.counter-1][0], 
                                                 self.waypoints[self.counter][1] - self.waypoints[self.counter-1][1], 
                                                 self.waypoints[self.counter][2] - self.waypoints[self.counter-1][2]])
                    # Calculate the yaw angle using arctan2
                    self.yawPoint = math.atan2(direction_vector[1], direction_vector[0])
                
        elif self.finishedMission == True:	
            print("Sem WayPoints novos")
        
    def acelWayp(self):
        
        desired_velocity_vetor = []
        if self.counter >=1:
            # error signal for PD controller
            distWayP_Vtol = math.dist((self.waypoints[self.counter-1][0], self.waypoints[self.counter-1][1]), (self.x_Vtol,self.y_Vtol))
            velocity_adjustment = self.kp_acel * distWayP_Vtol
            
            #Limit the velocity to max speed
            velocity_adjustment = min(velocity_adjustment, self.maxSpeed)

            desired_velocity_vetor = ((self.waypoints[self.counter][0] - self.x_Vtol)/ self.distVtol_WayP * velocity_adjustment,
                                      (self.waypoints[self.counter][1] - self.y_Vtol)/ self.distVtol_WayP * velocity_adjustment,
                                      (self.waypoints[self.counter][2] - self.z_Vtol)/ self.distVtol_WayP * velocity_adjustment)
            
            self.linearVel_x = desired_velocity_vetor[0]
            self.linearVel_y = desired_velocity_vetor[1]

    def brakeWayp(self):
        
        desired_velocity_vetor = []
        if self.counter >=1:
    
            velocity_adjustment = self.kp_acel * self.distVtol_WayP
            
            #Limit the velocity to max speed
            velocity_adjustment = max(velocity_adjustment, 2)

            desired_velocity_vetor = ((self.waypoints[self.counter][0] - self.x_Vtol)/ self.distVtol_WayP * velocity_adjustment,
                                      (self.waypoints[self.counter][1] - self.y_Vtol)/ self.distVtol_WayP * velocity_adjustment,
                                      (self.waypoints[self.counter][2] - self.z_Vtol)/ self.distVtol_WayP * velocity_adjustment)
            
            self.linearVel_x = desired_velocity_vetor[0]
            self.linearVel_y = desired_velocity_vetor[1]    

    def go_to_point(self):
        
        # Create the message to publish 
        setPoint_msg = PositionTarget()
        setPoint_msg.header.stamp = rospy.Time.now()
        setPoint_msg.coordinate_frame = 1
    
        if self.finishedMission == False:
            # Velocity Control
            setPoint_msg.type_mask = 2019
            # Calculate the distance between Vtol and Waypoint
            self.distVtol_WayP = abs(math.dist((self.x_Vtol,self.y_Vtol), (self.x_wayP,self.y_wayP)))    

            if self.distVtol_WayP > self.dist_tolerance:
        
                self.waypointRequest = False
                self.acelWayp()
    
            elif self.distVtol_WayP <= self.dist_tolerance:
                
                self.brakeWayp() 

                if self.distVtol_WayP <= self.radiusWayp: 
                    # Reached waypoint
                    self.cont_reachedWayp += 1
                    if self.cont_reachedWayp == 1:
                        # Save the vtol position
                        self.xVtol_c2wp = self.x_Vtol
                        self.yVtol_c2wp = self.y_Vtol
                        self.zVtol_c2wp = self.z_Vtol# Create the message to publish 
                    # Check if vtol has passed the waypoint
                    dir_vector = (self.waypoints[self.counter][0] - self.xVtol_c2wp, 
                                  self.waypoints[self.counter][1] - self.yVtol_c2wp,
                                  self.waypoints[self.counter][2] - self.zVtol_c2wp)
                    vector_to_current = (self.x_Vtol - self.xVtol_c2wp, 
                                        self.y_Vtol - self.yVtol_c2wp, 
                                        self.z_Vtol - self.zVtol_c2wp)
                    # Calculate the dot product between direction vector and vector to current position
                    dot_product = sum(a * b for a,b in zip(dir_vector, vector_to_current)) - self.radiusWayp
                    # If vtol passed the waypoint
                    if dot_product > 0:
                        self.waypointRequest = True
                        self.cont_reachedWayp = 0
            
            print("VX:",self.linearVel_x, "Vy:",self.linearVel_y)
            # Control VTOL
            setPoint_msg.position.z = self.z_wayP
            setPoint_msg.velocity.x = self.linearVel_x
            setPoint_msg.velocity.y = self.linearVel_y
            setPoint_msg.yaw_rate = 0
            self.pubSetpoint.publish(setPoint_msg) 
        
        elif self.finishedMission == True:
            
            print(self.x_wayP)
            setPoint_msg.type_mask = 4088
            setPoint_msg.position.x = self.x_wayP
            setPoint_msg.position.y = self.y_wayP
            setPoint_msg.position.z = self.z_wayP
            self.pubSetpoint.publish(setPoint_msg) 



            


def main():
    rospy.init_node("mission_node")

    mission = pathMission()
    mission.waypointRequest = True
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        
        mission.convertWaypoint()
        mission.go_to_point()
        rate.sleep()


if __name__ == "__main__":
    main()	
