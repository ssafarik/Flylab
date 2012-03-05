#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('track_image_contours')
import rospy
import numpy as N
import tf
from track_image_contours.msg import ArenaState
from plate_tf.msg import InBounds, FlyView
from geometry_msgs.msg import PointStamped
from pythonmodules import CircleFunctions


class PublishSystemState:
    def __init__(self):
        self.initialized = False
        self.in_bounds_radius = rospy.get_param('in_bounds_radius', 10.0)
        self.robot_fly_state_sub = rospy.Subscriber("ArenaState", ArenaState, self.arenastate_callback)
        self.tfrx = tf.TransformListener()

        self.pub_in_bounds = rospy.Publisher("InBounds", InBounds)
        self.pub_fly_view = rospy.Publisher("FlyView", FlyView)

        self.start_position_x = 0 #rospy.get_param("start_position_x")
        self.start_position_y = 0 #rospy.get_param("start_position_y")

        self.in_bounds = InBounds()
        self.fly_view = FlyView()

        self.robot_origin = PointStamped()
        self.robot_origin.header.frame_id = "Robot"
        self.robot_origin.point.x = 0
        self.robot_origin.point.y = 0
        self.robot_origin.point.z = 0

        #self.tfrx.waitForTransform("Fly", self.robot_origin.header.frame_id, rospy.Time(), rospy.Duration(5.0))

        self.initialized = True


    def arenastate_callback(self, arenastate):
        if self.initialized:
            robot_x = arenastate.robot.pose.position.x
            robot_y = arenastate.robot.pose.position.y
            try:
                fly_x = arenastate.flies[0].pose.position.x
                fly_y = arenastate.flies[0].pose.position.y
            except:
                return

            
            # Get and publish InBounds.
            robot_dist = N.sqrt((robot_x - self.start_position_x)**2 + (robot_y - self.start_position_y)**2)
            fly_dist = N.sqrt((fly_x - self.start_position_x)**2 + (fly_y - self.start_position_y)**2)
            self.in_bounds.bounds_radius = self.in_bounds_radius
            if robot_dist < self.in_bounds_radius:
                self.in_bounds.robot_in_bounds = True
            else:
                self.in_bounds.robot_in_bounds = False
            if fly_dist < self.in_bounds_radius:
                self.in_bounds.fly_in_bounds = True
            else:  
                self.in_bounds.fly_in_bounds = False 
            self.pub_in_bounds.publish(self.in_bounds)


            # Get & publish the FlyView.
            try:
                self.tfrx.waitForTransform("Fly", self.robot_origin.header.frame_id, rospy.Time(), rospy.Duration(15.0))
                self.robot_origin_fly_frame = self.tfrx.transformPoint("Fly", self.robot_origin)
                self.fly_view.robot_position_x = rx = self.robot_origin_fly_frame.point.x
                self.fly_view.robot_position_y = ry = self.robot_origin_fly_frame.point.y
                self.fly_view.robot_angle = CircleFunctions.mod_angle(N.arctan2(ry,rx))
                self.fly_view.robot_distance = N.sqrt(rx**2 + ry**2)
                self.pub_fly_view.publish(self.fly_view)
            except (tf.LookupException, tf.ConnectivityException):
                pass


if __name__ == '__main__':
    rospy.init_node('PublishSystemState')
    pss = PublishSystemState()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
