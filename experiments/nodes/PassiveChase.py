#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('experiments')
import rospy
import numpy as N
import ExperimentLib
from experiments.srv import *
from geometry_msgs.msg import Point



#######################################################################################################
class ExperimentPassiveChase():
    def __init__(self):
        rospy.init_node('Experiment')
        
        # Fill out the data structure that defines the experiment.
        self.experimentparams = ExperimentParamsRequest()
        
        self.experimentparams.experiment.description = "Fly Chases Robot Moving in Circle"
        self.experimentparams.experiment.maxTrials = 1
        self.experimentparams.experiment.trial = 1
        
        self.experimentparams.save.filenamebase = "passivechase"
        self.experimentparams.save.arenastate = True
        self.experimentparams.save.video = True
        self.experimentparams.save.bag = False
        self.experimentparams.save.onlyWhileTriggered = True
        
        self.experimentparams.tracking.exclusionzone.enabled = False
        self.experimentparams.tracking.exclusionzone.point_list = [Point(x=0.0, y=0.0)]
        self.experimentparams.tracking.exclusionzone.radius_list = [0.0]
        
        self.experimentparams.home.enabled = False
        self.experimentparams.home.x = 0.0
        self.experimentparams.home.y = 0.0
        self.experimentparams.home.speed = 20
        self.experimentparams.home.timeout = -1
        self.experimentparams.home.tolerance = 2
        
        self.experimentparams.waitEntry = 0#150.0
        
        self.experimentparams.triggerEntry.enabled = False
        self.experimentparams.triggerEntry.frameidParent = 'Fly1'
        self.experimentparams.triggerEntry.frameidChild = 'Robot'
        self.experimentparams.triggerEntry.speedAbsParentMin =   0.0
        self.experimentparams.triggerEntry.speedAbsParentMax = 999.0
        self.experimentparams.triggerEntry.speedAbsChildMin  =   0.0
        self.experimentparams.triggerEntry.speedAbsChildMax  = 999.0
        self.experimentparams.triggerEntry.speedRelMin       =   0.0
        self.experimentparams.triggerEntry.speedRelMax       = 999.0
        self.experimentparams.triggerEntry.distanceMin =   0.0
        self.experimentparams.triggerEntry.distanceMax = 999.0
        self.experimentparams.triggerEntry.angleMin =  0.0 * N.pi / 180.0
        self.experimentparams.triggerEntry.angleMax =180.0 * N.pi / 180.0
        self.experimentparams.triggerEntry.angleTest = 'inclusive'
        self.experimentparams.triggerEntry.angleTestBilateral = True
        self.experimentparams.triggerEntry.timeHold = 0.0
        self.experimentparams.triggerEntry.timeout = -1
        

        # .move, .lasertrack, and .triggerExit all run concurrently.
        # The first one to finish preempts the others.
        self.experimentparams.move.enabled = True
        self.experimentparams.move.mode = 'pattern' # 'pattern' or 'relative'
        self.experimentparams.move.pattern.shape = 'circle' # 'constant' or 'circle' or 'square' or 'flylogo' or 'spiral' or 'ramp'
        self.experimentparams.move.pattern.hzPattern = 1/40
        self.experimentparams.move.pattern.hzPoint = 10
        self.experimentparams.move.pattern.count = -1
        self.experimentparams.move.pattern.size.x = rospy.get_param('motorarm/L1', 999)
        self.experimentparams.move.pattern.size.y = 0
        self.experimentparams.move.timeout = 600
        
        self.experimentparams.lasertrack.enabled = False
        
        self.experimentparams.triggerExit.enabled = False
        self.experimentparams.triggerExit.frameidParent = 'Fly1'
        self.experimentparams.triggerExit.frameidChild = 'Robot'
        self.experimentparams.triggerExit.speedAbsParentMin =   0.0
        self.experimentparams.triggerExit.speedAbsParentMax = 999.0
        self.experimentparams.triggerExit.speedAbsChildMin  =   0.0
        self.experimentparams.triggerExit.speedAbsChildMax  = 999.0
        self.experimentparams.triggerExit.speedRelMin       =   0.0
        self.experimentparams.triggerExit.speedRelMax       = 999.0
        self.experimentparams.triggerExit.distanceMin = 0.0
        self.experimentparams.triggerExit.distanceMax = 999.0
        self.experimentparams.triggerExit.angleMin =  0.0 * N.pi / 180.0
        self.experimentparams.triggerExit.angleMax =180.0 * N.pi / 180.0
        self.experimentparams.triggerExit.angleTest = 'inclusive'
        self.experimentparams.triggerExit.angleTestBilateral = True
        self.experimentparams.triggerExit.timeHold = 0.0
        self.experimentparams.triggerExit.timeout = -1

        self.experimentparams.waitExit = 0.0
        
        self.experiment = ExperimentLib.Experiment(self.experimentparams)


    def Run(self):
        self.experiment.Run()



if __name__ == '__main__':
    #try:
        experiment = ExperimentPassiveChase()
        experiment.Run()
        
    #except:
        rospy.loginfo("Shutting down")

        
