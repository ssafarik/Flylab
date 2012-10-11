#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('experiments')
import rospy
import numpy as N
import ExperimentLib
from geometry_msgs.msg import Point, Twist
from experiments.srv import *
from flycore.msg import MsgFrameState
from galvodirector.msg import MsgGalvoCommand
from patterngen.msg import MsgPattern


#######################################################################################################
class ExperimentRecord10MinTrials():
    def __init__(self):
        rospy.init_node('Experiment')
        
        # Fill out the data structure that defines the experiment.
        self.experimentparams = ExperimentParamsRequest()
        
        self.experimentparams.experiment.description = "Record 10 Minutes of Fly Movement"
        self.experimentparams.experiment.maxTrials = -1
        self.experimentparams.experiment.trial = 1
        
        self.experimentparams.save.filenamebase = "tenmin"
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
        
        self.experimentparams.waitEntry = 0.0
        
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
        self.experimentparams.move.enabled = False
        self.experimentparams.move.mode = 'relative'
        self.experimentparams.move.relative.tracking = True
        self.experimentparams.move.relative.frameidOriginPosition = "Plate"
        self.experimentparams.move.relative.frameidOriginAngle = "Plate"
        self.experimentparams.move.relative.distance = 50
        self.experimentparams.move.relative.angle =  90.0 * N.pi / 180.0
        self.experimentparams.move.relative.angleType = 'constant'
        self.experimentparams.move.relative.speed = 50
        self.experimentparams.move.relative.speedType = 'constant'
        self.experimentparams.move.relative.tolerance = 2
        self.experimentparams.move.timeout = -1
        
        self.experimentparams.lasertrack.enabled = False
        
        self.experimentparams.triggerExit.enabled = True
        self.experimentparams.triggerExit.frameidParent = 'Fly1'
        self.experimentparams.triggerExit.frameidChild = 'Robot'
        self.experimentparams.triggerExit.speedAbsParentMin = 999.0
        self.experimentparams.triggerExit.speedAbsParentMax = 999.0
        self.experimentparams.triggerExit.speedAbsChildMin  =   0.0
        self.experimentparams.triggerExit.speedAbsChildMax  = 999.0
        self.experimentparams.triggerExit.speedRelMin       =   0.0
        self.experimentparams.triggerExit.speedRelMax       = 999.0
        self.experimentparams.triggerExit.distanceMin = 999.0
        self.experimentparams.triggerExit.distanceMax =   0.0
        self.experimentparams.triggerExit.angleMin =  0.0 * N.pi / 180.0
        self.experimentparams.triggerExit.angleMax =180.0 * N.pi / 180.0
        self.experimentparams.triggerExit.angleTest = 'inclusive'
        self.experimentparams.triggerExit.angleTestBilateral = True
        self.experimentparams.triggerExit.timeHold = 0.0
        self.experimentparams.triggerExit.timeout = 600

        self.experimentparams.waitExit = 0.0
        
        self.experimentlib = ExperimentLib.ExperimentLib(self.experimentparams, 
                                                         newexperiment_callback = self.Newexperiment_callback, 
                                                         newtrial_callback = self.Newtrial_callback, 
                                                         endtrial_callback = self.Endtrial_callback)



    def Run(self):
        self.experimentlib.Run()
        

    # This function gets called at the start of a new experiment.  Use this to do any one-time initialization of hardware, etc.
    def Newexperiment_callback(self, userdata):
        return 'success'
        

    # This function gets called at the start of a new trial.  Use this to alter the experiment params from trial to trial.
    def Newtrial_callback(self, userdata):
        userdata.experimentparamsOut = userdata.experimentparamsIn
        return 'success'

    # This function gets called at the end of a new trial.  Use this to alter the experiment params from trial to trial.
    def Endtrial_callback(self, userdata):
        userdata.experimentparamsOut = userdata.experimentparamsIn
        return 'success'



if __name__ == '__main__':
    try:
        experiment = ExperimentRecord10MinTrials()
        experiment.Run()
        
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

        
