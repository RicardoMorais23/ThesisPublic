#!/bin/env python

import rospy
from mission_vtol.srv import waypoint_service, waypoint_serviceResponse

class WaypointService(object):

    def __init__(self):
        
        self.file_path = '/home/ricardo/Desktop/coordinates.txt'
        self.coordinates = []
        self.count = 0
        self.size = 0
        self.latitude = 0
        self.longitude = 0
        self.altitude = 0
        self.num_WayP = 0
        self.counter = 0
        self.flag_finished = False
        self.servWP = rospy.Service('sendWaypoint', waypoint_service, self.handle_send_waypoint)
        self.readFile()
        

    def readFile(self):
        cnt = 0
        with open(self.file_path, "r") as file:
            for line in file:
                if cnt == 1:
                    line_data = line.strip().split(",")
                    # LAT, LONG, ALT
                    self.coordinates.append((float(line_data[1]),float(line_data[2]),float(line_data[3])))
                cnt = 1
        self.size = len(self.coordinates)
        

    def handle_send_waypoint(self, req):
        
        if req.message == True and self.count != self.size:
            print(self.count)
            self.latitude = self.coordinates[self.count][0]
            self.longitude = self.coordinates[self.count][1]
            self.altitude = self.coordinates[self.count][2]
            self.num_WayP = self.size - self.count
            self.counter = self.count
            self.flag_finished = False
            self.count +=1

        elif req.message == True and self.count == self.size:
            self.latitude = self.coordinates[self.count-1][0]
            self.longitude = self.coordinates[self.count-1][1]
            self.altitude = self.coordinates[self.count-1][2]
            self.num_WayP = self.size - self.count
            self.counter = self.count
            self.flag_finished = True

        return waypoint_serviceResponse(self.latitude, self.longitude, self.altitude, self.num_WayP, self.counter, self.flag_finished)

def main():
    rospy.init_node("serviceWP_node")

    waypoint_service = WaypointService()


    rospy.spin()

if __name__ == "__main__":
    main()	


