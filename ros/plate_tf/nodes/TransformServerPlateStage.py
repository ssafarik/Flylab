#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('plate_tf')
import rospy
import numpy as N
import tf
import plate_tf.srv
from geometry_msgs.msg import Point, Quaternion


class TransformServerPlateStage:
    def __init__(self):
        self.initialized = False
        rospy.init_node('TransformServerPlateStage')

        self.tfrx = tf.TransformListener()
        self.tfbx = tf.TransformBroadcaster()

        self.ptPlateStage = Point()
        self.ptPlateStage.x = rospy.get_param('plate_stage_x', 0.0)
        self.ptPlateStage.y = rospy.get_param('plate_stage_y', 0.0)
        self.ptPlateStage.z = rospy.get_param('plate_stage_z', 0.0)
        self.qPlateStage = Quaternion()
        self.qPlateStage.x = rospy.get_param('plate_stage_qx', 0.0)
        self.qPlateStage.y = rospy.get_param('plate_stage_qy', 0.0)
        self.qPlateStage.z = rospy.get_param('plate_stage_qz', 0.0)
        self.qPlateStage.w = rospy.get_param('plate_stage_qw', 1.0)

        #while not self.initialized:
        #    try:
        #        (self.trans,self.rot) = self.tfrx.lookupTransform("Stage", "Plate", rospy.Time(0))
        #    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #        pass
        #    time.sleep(0.5)
        self.transPlateStage = (self.ptPlateStage.x,self.ptPlateStage.y,self.ptPlateStage.z)
        self.rotPlateStage = (self.qPlateStage.x,self.qPlateStage.y,self.qPlateStage.z,self.qPlateStage.w)

        self.T = tf.transformations.translation_matrix(self.transPlateStage)
        self.R = tf.transformations.quaternion_matrix(self.rotPlateStage)
        self.M = tf.transformations.concatenate_matrices(self.T, self.R)
        self.Minv = N.linalg.inv(self.M)

        srv_ps = rospy.Service('plate_to_stage', plate_tf.srv.PlateStageConversion, self.StageFromPlate_callback)
        srv_sp = rospy.Service('stage_to_plate', plate_tf.srv.PlateStageConversion, self.PlateFromStage_callback)

        self.initialized = True

    def StageFromPlate_callback(self,req):
    # Appears to be no longer used.
        rospy.logwarn ('StageFromPlate_callback()')
        nPoints = min(len(req.Xsrc), len(req.Ysrc))
        Xsrc = list(req.Xsrc)
        Ysrc = list(req.Ysrc)
        Zsrc = [0]*nPoints
        Hsrc = [1]*nPoints
        plate_points = N.array([Xsrc, Ysrc, Zsrc, Hsrc])
        stage_points = N.dot(self.M, plate_points)
        Xdst = stage_points[0,:]
        Ydst = stage_points[1,:]
        return {'Xdst': Xdst,
                'Ydst': Ydst}


    def PlateFromStage_callback(self,req):
    # Appears to be no longer used.
        rospy.logwarn ('PlateFromStage_callback()')
        nPoints = min(len(req.Xsrc), len(req.Ysrc))
        Xsrc = list(req.Xsrc)
        Ysrc = list(req.Ysrc)
        Zsrc = [0]*nPoints
        Hsrc = [1]*nPoints
        stage_points = N.array([Xsrc, Ysrc, Zsrc, Hsrc])
        plate_points = N.dot(self.Minv, stage_points)

        Xdst = plate_points[0,:]
        Ydst = plate_points[1,:]
        return {'Xdst': Xdst,
                'Ydst': Ydst}


    def SendTransforms(self):
        if self.initialized:
            self.tfbx.sendTransform(self.transPlateStage, 
                                    self.rotPlateStage,
                                    rospy.Time.now(),
                                    "Stage",     # child
                                    "Plate"      # parent
                                    )
        
        
    def Main(self):
        rate = rospy.Rate(100) # BUG: Reducing this causes erratic behavior w/ patterngen & fivebar. 
        try:
            while not rospy.is_shutdown():
                try:
                    self.SendTransforms()
                    rate.sleep()
                except tf.Exception:
                    rospy.logwarn ('Exception in TransformServerPlateStage()')
        except:
            print "Shutting down"


if __name__ == "__main__":
    tsps = TransformServerPlateStage()
    tsps.Main()
    
