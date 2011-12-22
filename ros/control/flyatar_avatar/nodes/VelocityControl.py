#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('avatar')
#import sys
import rospy
#import math
import tf
#import cv
import numpy
from flystage.srv import *
from flystage.msg import Commands,Velocity
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#from geometry_msgs.msg import PointStamped

class VelocityControl:

    def __init__(self):
        self.dt = rospy.get_param("Stage_Update_dt")

        self.tf_listener = tf.TransformListener()

        self.joy_sub = rospy.Subscriber("magnet/commands", Commands, self.commands_callback)
        self.vel_pub = rospy.Publisher("stage/command_velocity",Velocity)

        self.vel_stage = Velocity()
        self.vel_scale_factor = 100     # mm/s
        self.vel_vector_plate = numpy.array([[0],[0],[0],[1]])
        self.vel_vector_robot = numpy.array([[0],[0],[0],[1]])

        self.command_frame = "Stage"

    def vel_vector_convert(self,vel_vector,frame):
        (trans,q) = self.tf_listener.lookupTransform("Stage",frame,rospy.Time(0))
        rot_matrix = tf.transformations.quaternion_matrix(q)
        vel_vector_stage = numpy.dot(rot_matrix,vel_vector)
        x_vel_stage = vel_vector_stage[0]
        y_vel_stage = vel_vector_stage[1]
        return [x_vel_stage,y_vel_stage]

    def commands_callback(self,data):
        self.command_frame = data.header.frame_id
        self.radius_velocity = data.radius_velocity*self.vel_scale_factor
        self.tangent_velocity = data.tangent_velocity*self.vel_scale_factor
        self.vel_vector_plate[0,0] = data.x_velocity*self.vel_scale_factor
        self.vel_vector_plate[1,0] = data.y_velocity*self.vel_scale_factor

        try:
            (self.vel_stage.x_velocity,self.vel_stage.y_velocity) = self.vel_vector_convert(self.vel_vector_plate,"Plate")

            (trans,q) = self.tf_listener.lookupTransform(self.command_frame,'/Robot',rospy.Time(0))
            x = trans[0]
            y = trans[1]
            theta = numpy.arctan2(y,x)

            rot_matrix = tf.transformations.rotation_matrix(theta, (0,0,1))
            self.vel_vector_robot[0,0] = self.radius_velocity
            self.vel_vector_robot[1,0] = self.tangent_velocity
            vel_vector_plate = numpy.dot(rot_matrix,self.vel_vector_robot)
            self.vel_vector_plate[0,0] = vel_vector_plate[0]
            self.vel_vector_plate[1,0] = vel_vector_plate[1]

            (x_vel_stage,y_vel_stage) = self.vel_vector_convert(self.vel_vector_plate,"Plate")
            self.vel_stage.x_velocity += x_vel_stage
            self.vel_stage.y_velocity += y_vel_stage

            self.vel_pub.publish(self.vel_stage)
        except (tf.LookupException, tf.ConnectivityException):
            self.vel_stage.x_velocity = data.y_velocity*self.vel_scale_factor
            self.vel_stage.y_velocity = -data.x_velocity*self.vel_scale_factor
            self.vel_pub.publish(self.vel_stage)


if __name__ == '__main__':
    rospy.init_node('VelocityControl', anonymous=True)
    vc = VelocityControl()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
