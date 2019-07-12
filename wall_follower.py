#!/usr/bin/env python

import numpy as np
import sys, math, random, copy
import rospy, copy, time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class PotentialField:
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    LEN = 100

    def __init__(self):
        self.data = None
        self.cmd = AckermannDriveStamped()
        self.angle = 0
        self.collision =  False
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        #write your publishers and subscribers here; they should be the same as the wall follower's
        #cartesian points -- to be filled (tuples)
        self.cartPoints = [None for x in range(150)]
        #[speed, angle]
        self.finalVector = [0.5, 0]

        ##preventing collision



    def scan_callback(self, data):
        '''Checks LIDAR data'''
        self.data = data.ranges
        self.drive_callback()

    def drive_callback(self):
        '''Publishes drive commands'''
        self.convertPoints(self.data)
        self.calcFinalVector(self.data)
        self.cmd.drive.speed = self.finalVector[0]
        print(self.finalVector[0])
        self.cmd.drive.steering_angle = self.finalVector[1]
        self.drive_pub.publish(self.cmd)
        #make sure to publish cmd here

    def convertPoints(self, points):
        '''Convert all current LIDAR data to cartesian coordinates'''
        average = 0.0;
        for i in range(len(points)):
            average = average + points[i]
        average = average / len(points)
        for i in range(len(points)):
            speed = 1.0/points[i] * average

            angle = ((((i*1.0)/len(points)) * 4.1887) - 2.094) ##4.71238
            ##print("angle" )
            ##print(angle * 360/6.28)
            x = points[i] * math.cos(angle)
            y = points[i] * math.sin(angle)
            self.cartPoints[i] = [x,y]
        for i in range(50):
            self.cartPoints[100 + i] = [0,-1]

    def calcFinalVector(self, points):
        sum_x = 0.0
        sum_y = 0.0
        for i in range(len(self.cartPoints)):
            sum_x = sum_x - self.cartPoints[i][0]
        ##sum_x = sum_x/(len(self.cartPoints) * 1.0)

        for i in range(len(self.cartPoints)):
            sum_y = sum_y - self.cartPoints[i][1]
        ##sum_y = sum_y/(len(self.cartPoints) * 1.0)

        self.finalVector[0] = math.sqrt(math.pow(sum_x, 2) + math.pow(sum_y,2))
        self.finalVector[1] = math.atan2(sum_y,(sum_x + 0.00001))
        angle = self.finalVector[1] * (360/6.28)
        angle = 180 - angle

        if(angle > 90 and angle > -90):
            if(angle < 0):
                angle = angle * -1
                self.finalVector[1] = 1
            self.finalVector[1] = angle/90.0
        else:
            if(angle < 0):
                angle = angle * -1
                self.finalVector[1] = -1
            self.finalVector[1] = -1 * angle/90.0


        index = 1000
        min = 1000
        for i in range(20):
            tempi = i + 40 ##needs to change in a computer
            if(points[tempi] < min):
                min = points[tempi]
                index = tempi

        start = 0
        print(min)
        if(self.collision):
            end = time.time()
            if(end - start < 1):
                self.collision = False
            else:
                self.finalVector[0] = -2
                if(points[0] < points[99]):
                    self.finalVector[1] = 1
                else:
                    self.finalVector[1] = -1
        elif(min < 0.5):
            start = time.time()
            self.collision = True
            self.finalVector[0] = -10
            if(points[0] < points[99]):
                self.finalVector[1] = 1
            else:
                self.finalVector[1] = -1
        else:
            self.collision = False




        ##self.finalVector[0] = 2






if __name__ == "__main__":
    rospy.init_node('potential_field')
    potential_field = PotentialField()
    rospy.spin()
