#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('plate_tf')
import rospy
import numpy
import tf
import plate_tf.srv
import time

class Transforms:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.bInitialized = False
        while not self.bInitialized:
            try:
                (self.trans,self.rot) = self.tf_listener.lookupTransform("Stage", "Plate", rospy.Time(0))
                self.bInitialized = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            time.sleep(0.1)

        self.T = tf.transformations.translation_matrix(self.trans)
        self.R = tf.transformations.quaternion_matrix(self.rot)
        self.M = tf.transformations.concatenate_matrices(self.T, self.R)
        self.Minv = numpy.linalg.inv(self.M)

    def plate_to_stage(self,req):
        point_count = min(len(req.Xsrc), len(req.Ysrc))
        Xsrc = list(req.Xsrc)
        Ysrc = list(req.Ysrc)
        Zsrc = [0]*point_count
        Hsrc = [1]*point_count
        plate_points = numpy.array([Xsrc, Ysrc, Zsrc, Hsrc])
        stage_points = numpy.dot(self.M, plate_points)
        Xdst = stage_points[0,:]
        Ydst = stage_points[1,:]
        return {'Xdst': Xdst,
                'Ydst': Ydst}

    def stage_to_plate(self,req):
        point_count = min(len(req.Xsrc), len(req.Ysrc))
        Xsrc = list(req.Xsrc)
        Ysrc = list(req.Ysrc)
        Zsrc = [0]*point_count
        Hsrc = [1]*point_count
        stage_points = numpy.array([Xsrc, Ysrc, Zsrc, Hsrc])
        plate_points = numpy.dot(self.Minv, stage_points)

        Xdst = plate_points[0,:]
        Ydst = plate_points[1,:]
        return {'Xdst': Xdst,
                'Ydst': Ydst}

def plate_stage_transforms_server():
    rospy.init_node('PlateStageTransforms')
    transforms = Transforms()
    s_ps = rospy.Service('plate_to_stage', plate_tf.srv.PlateStageConversion, transforms.plate_to_stage)
    s_sp = rospy.Service('stage_to_plate', plate_tf.srv.PlateStageConversion, transforms.stage_to_plate)

if __name__ == "__main__":
    plate_stage_transforms_server()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
