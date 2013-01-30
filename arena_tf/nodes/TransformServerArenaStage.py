#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('arena_tf')
import rospy
import numpy as N
import tf
from geometry_msgs.msg import Point, Quaternion


class TransformServerArenaStage:
    def __init__(self):
        self.initialized = False
        rospy.init_node('TransformServerArenaStage')

        self.tfrx = tf.TransformListener()
        self.tfbx = tf.TransformBroadcaster()

        self.ptArenaStage = Point()
        self.ptArenaStage.x = rospy.get_param('arena_stage_x', 0.0)
        self.ptArenaStage.y = rospy.get_param('arena_stage_y', 0.0)
        self.ptArenaStage.z = rospy.get_param('arena_stage_z', 0.0)
        self.qArenaStage = Quaternion()
        self.qArenaStage.x = rospy.get_param('arena_stage_qx', 0.0)
        self.qArenaStage.y = rospy.get_param('arena_stage_qy', 0.0)
        self.qArenaStage.z = rospy.get_param('arena_stage_qz', 0.0)
        self.qArenaStage.w = rospy.get_param('arena_stage_qw', 1.0)

        self.transArenaStage = (self.ptArenaStage.x, 
                                self.ptArenaStage.y,
                                self.ptArenaStage.z)
        self.rotArenaStage   = (self.qArenaStage.x,
                                self.qArenaStage.y,
                                self.qArenaStage.z,
                                self.qArenaStage.w)

        self.initialized = True


    def SendTransforms(self):
        if self.initialized:
            self.tfbx.sendTransform(self.transArenaStage, 
                                    self.rotArenaStage,
                                    rospy.Time.now(),
                                    "Stage",     # child
                                    "Arena"      # parent
                                    )
        
        
    def Main(self):
        rate = rospy.Rate(100) # BUG: Reducing this causes erratic behavior w/ patterngen & fivebar. 
        try:
            while not rospy.is_shutdown():
                try:
                    self.SendTransforms()
                    rate.sleep()
                except tf.Exception:
                    rospy.logwarn ('Exception in TransformServerArenaStage()')
        except:
            print "Shutting down"


if __name__ == "__main__":
    tsps = TransformServerArenaStage()
    tsps.Main()
    
