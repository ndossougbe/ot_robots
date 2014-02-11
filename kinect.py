#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
* Travailler sur une ligne unique, récupérer la valeur minimale (obstacle le plus proche)
* kinect
    * voit de 40cm à 5m (précision dégradée entre 4 et 5m), angle d'ouverture 57°
    * diamètre 23 cm
    * chaque cellule => 4 octets
    * utilisation d'opencv pour interpréter le format d'image
* algorithme de contournement
    * l'angle entre l'obstacle et la direction est calculable en utilisant l'angle d'ouverture et la position de la valeur minimale sur la ligne
    * convertir la modification d'angle en twist (vitesse linéaire et vitesse angulaire)
        * Vitesse du robot: `v_x = (v_g+v_d)/2`
        * Rotation du robot (rad/s) : `omega = (-v_g+v_d)/2r`
        * `r` est le rayon du robot (moitié de la distance entre les roues) 
'''


import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from publishers import ThreadedPublisher
from threading import Thread
import numpy as np
import cv2
import cv2.cv as cv
import math


def convertToRad(degree):
    return degree * math.pi * 1.0 / 180


def convertToDegree(rad):
    return rad * 180.0 / math.pi

DISTANCE_MAX = 2  # meters
DISTANCE_MIN = 0.40  # meters
THETA_MAX = convertToRad(30)  # meters
THETA_MIN = convertToRad(10)  # meters

MAX_LINEAR_SPEED = 0.3
MAX_ANGULAR_SPEED = 1
    

class Braitenberg(ThreadedPublisher):
    def __init__(self, target_sim):
        if target_sim: 
            topic = '/turtle1/cmd_vel' 
        else:
            topic = '/cmd_vel_mux/input/teleop'
        super(Braitenberg, self).__init__(topic, Twist, 1/10.0)
        self.twist = Twist()
        self.kinect_thread = Thread(target=self.kinect_listener)

    def update(self):
        self.publish(self.twist)
        # print(self.twist)

    def start(self):
        self.kinect_thread.start()
        super(Braitenberg, self).start()

    def join(self):
        self.kinect_thread.join()
        super(Braitenberg, self).join()

    def angle_calculation(self, image_width, point_index, alpha=convertToRad(57)):
        theta = math.atan((2*math.fabs((image_width//2) - point_index) / image_width)* math.tan(alpha/2.0))
        if image_width//2 > point_index: return -theta
        else: return theta

    def kinect_listener(self):
        print 'ki start'
        rospy.Subscriber('camera/depth/image', Image, self.process_image)
        rospy.spin()

    def process_image(self, msg):
        print 'pi'
        bridge = CvBridge()
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # The depth image is a single-channel float32 image
            depth_image = bridge.imgmsg_to_cv(msg, "32FC1")
            
            floor_distance = 0.25
            mid_row_idx = depth_image.height//2

            d = 4209183091839
            idx = 0
            for x in xrange(depth_image.width):
                for y in xrange(mid_row_idx-2, mid_row_idx + 2 ):
                    tmp_depth = depth_image[y, x]
                    if tmp_depth != 'nan' and tmp_depth < d:
                        d = tmp_depth 
                        idx = x


            # print depth_image.width, depth_image.height, depth_image[depth_image.height//2, idx]
            theta = self.angle_calculation(depth_image.width, idx)
            # print theta
            self.twist = self.evaluate_twist(theta, d)
        except CvBridgeError, e:
            print e
        # depth_image.height = height of the matrix
        # depth_image.width = width of the matrix
        # depth_image[x,y] = the float value in m of a point place a a height x and width y


    def evaluate_twist(self, theta, distance):
        if (distance > DISTANCE_MAX):
            return Twist(Vector3(MAX_LINEAR_SPEED, 0, 0), Vector3(0,0,0))

        abs_theta = max(math.fabs(theta), THETA_MIN)
        angular_gain = (THETA_MAX - abs_theta) / (THETA_MAX - THETA_MIN) * theta/math.fabs(theta)
        linear_gain = (DISTANCE_MAX - distance) / (DISTANCE_MAX - DISTANCE_MIN)
        print theta, linear_gain, angular_gain
        return Twist(Vector3(MAX_LINEAR_SPEED * linear_gain, 0,0), Vector3( 0,0,MAX_ANGULAR_SPEED * angular_gain))

if __name__ == '__main__':
    try:
        rospy.init_node('remote_controller')
        
        b = Braitenberg(True)
        b.start()

        rospy.spin()  # waits until rospy.is_shutdown() is true (Ctrl+C)
 
        print '\nTerminating the publishers...'
        b.terminate()
        b.join()

        print 'All ok.'

    except rospy.ROSInterruptException:
        pass