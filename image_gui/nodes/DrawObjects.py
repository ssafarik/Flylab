#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('image_gui')
import rospy
import copy
import image_gui.msg
import Colors
import CvPrimitives
import DrawPrimitives
import tf
from geometry_msgs.msg import PointStamped
import arena_tf.srv
import numpy as N
from tracking.msg import ArenaState


class DrawObjects:
    def __init__(self):
        self.initialized = False
        self.frameidDisplay = "ImageRect"
        self.pubDrawObjects = rospy.Publisher("DrawObjects/image_rect", image_gui.msg.DrawObjects)
        self.draw_objects = image_gui.msg.DrawObjects()
        self.subArenaState = rospy.Subscriber("ArenaState", ArenaState, self.arenastate_callback)
        self.arenastate = None
        rospy.sleep(5.0)
        self.hzDrawUpdate = rospy.get_param("hz_drawupdate","100")
        self.rate = rospy.Rate(self.hzDrawUpdate)

        rospy.wait_for_service('camera_from_arena')
        try:
            self.camera_from_arena = rospy.ServiceProxy('camera_from_arena', arena_tf.srv.ArenaCameraConversion)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        self.radiusArena = rospy.get_param('arena/radius_inner', 25.4)
        self.axis_length_arena = 4
        self.radiusMask = rospy.get_param('camera/mask/radius', 25)

        Xsrc = [0, self.radiusArena, self.axis_length_arena]
        Ysrc = [0, 0, 0]

        self.arena_origin_camera = PointStamped()
        self.arena_origin_camera.header.frame_id = "Camera"
        response = self.camera_from_arena(Xsrc,Ysrc)
        x0 = self.arena_origin_camera.point.x = response.Xdst[0]
        y0 = self.arena_origin_camera.point.y = response.Ydst[0]

        self.arena_point_camera = PointStamped()
        self.arena_point_camera.header.frame_id = "Camera"
        x1 = self.arena_point_camera.point.x = response.Xdst[1]
        y1 = self.arena_point_camera.point.y = response.Ydst[1]
        x2 = response.Xdst[2]
        y2 = response.Ydst[2]
        
        self.radiusMask = N.sqrt((x1-x0)**2 + (y1-y0)**2)
        self.axis_length_camera = N.sqrt((x2-x0)**2 + (y2-y0)**2)

        self.arena_origin = PointStamped()
        self.arena_origin.header.frame_id = "Arena"
        self.arena_origin.point.x = 0
        self.arena_origin.point.y = 0
        self.arena_origin.point.z = 0
        
        self.robot_image_origin = PointStamped()
        self.robot_image_origin.header.frame_id = "RobotContour"
        self.robot_image_origin.point.x = 0
        self.robot_image_origin.point.y = 0
        self.robot_image_origin.point.z = 0

        self.fly_image_origin = PointStamped()
        self.fly_image_origin.header.frame_id = "FlyContour"
        self.fly_image_origin.point.x = 0
        self.fly_image_origin.point.y = 0
        self.fly_image_origin.point.z = 0



        self.colors = Colors.Colors()
        self.origin = CvPrimitives.Point(0,0)

        self.tfrx = tf.TransformListener()
        #rospy.logwarn('DO waitForTransform(%s, %s, %s)' % (self.frameidDisplay, self.robot_image_origin.header.frame_id, rospy.Time()))
        #self.tfrx.waitForTransform(self.frameidDisplay, self.robot_image_origin.header.frame_id, rospy.Time(), rospy.Duration(15.0))
        #self.tfrx.waitForTransform(self.frameidDisplay, self.fly_image_origin.header.frame_id,   rospy.Time(), rospy.Duration(15.0))
        self.tfrx.waitForTransform(self.frameidDisplay, self.arena_origin.header.frame_id, rospy.Time(), rospy.Duration(15.0))
            

        self.arena_origin_display_frame = self.tfrx.transformPoint(self.frameidDisplay, self.arena_origin)

        self.arena_origin_primitives = CvPrimitives.Point(self.arena_origin_display_frame.point.x,
                                                          self.arena_origin_display_frame.point.y)

        self.markerArenaOrigin = DrawPrimitives.Axes(self.arena_origin_primitives.point)
        self.markerRobot       = DrawPrimitives.CenteredCircle(self.origin.point, 4, self.colors.blue, 2)
        self.markerFly         = DrawPrimitives.CenteredCircle(self.origin.point, 4, self.colors.red, 2)
        self.markerInBounds    = DrawPrimitives.CenteredCircle(self.arena_origin_primitives.point, self.radiusMask, self.colors.yellow, 2)
        self.markerMask        = DrawPrimitives.CenteredCircle(self.arena_origin_primitives.point, self.radiusMask, self.colors.yellow, 1)

        #self.draw_objects.draw_object_list = [self.markerArenaOrigin.draw_object,
        #                                      self.markerRobot.draw_object,
        #                                      self.markerFly.draw_object,
        #                                      self.markerInBounds.draw_object,
        #                                      self.markerMask.draw_object]
        self.draw_objects.draw_object_list = [self.markerInBounds.draw_object]

        self.draw_objects.show_all = True
        self.initialized = True


    def arenastate_callback(self, arenastate):
        self.arenastate = arenastate


    def update(self):
        if (self.initialized) and (self.arenastate is not None):
            try:
                #self.tfrx.waitForTransform(self.frameidDisplay, self.robot_image_origin.header.frame_id, rospy.Time(), rospy.Duration(1.0))
                self.robot_image_origin_display_frame = self.tfrx.transformPoint(self.frameidDisplay, self.robot_image_origin)
                self.markerRobot.change_center(CvPrimitives.Point(self.robot_image_origin_display_frame.point.x,
                                                                  self.robot_image_origin_display_frame.point.y).point)
            except (tf.LookupException, tf.ConnectivityException, tf.Exception), e:
                rospy.logwarn ('Exception in DrawObjects: %s' % e)

            try:                
                #self.tfrx.waitForTransform(self.frameidDisplay, self.fly_image_origin.header.frame_id, rospy.Time(), rospy.Duration(1.0))
                self.fly_image_origin_display_frame = self.tfrx.transformPoint(self.frameidDisplay, self.fly_image_origin)
                self.markerFly.change_center(CvPrimitives.Point(self.fly_image_origin_display_frame.point.x,
                                                                self.fly_image_origin_display_frame.point.y).point)
            except (tf.LookupException, tf.ConnectivityException, tf.Exception), e:
                rospy.logwarn ('Exception in DrawObjects: %s' % e)
                
            #self.markerRobot.change_center(CvPrimitives.Point(self.arenastate.robot.pose.position.x,
            #                                                   self.arenastate.robot.pose.position.y).point)
            #rospy.logwarn('DO posRobot=%s' % [self.arenastate.robot.pose.position.x, self.arenastate.robot.pose.position.y])

            self.pubDrawObjects.publish(self.draw_objects)


if __name__ == '__main__':
    rospy.init_node('DrawObjects')
    do = DrawObjects()

    while not rospy.is_shutdown():
        do.update()
        do.rate.sleep()
