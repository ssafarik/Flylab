#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('experiments')
import rospy
import numpy as N
import ExperimentLib
from experiments.srv import *



#######################################################################################################
class ExperimentPassiveChase():
    def __init__(self):
        rospy.init_node('Experiment')
        
        # Fill out the data structure that defines the experiment.
        self.experimentparams = ExperimentParamsRequest()
        
        self.experimentparams.experiment.description = "Fly Chases Robot Moving in Circle"
        self.experimentparams.experiment.maxTrials = -1
        self.experimentparams.experiment.trial = 1
        
        self.experimentparams.save.filenamebase = "passivechase"
        self.experimentparams.save.arenastate = True
        self.experimentparams.save.video = False
        self.experimentparams.save.bag = False
        self.experimentparams.save.onlyWhileTriggered = True
        
        self.experimentparams.home.enabled = False
        self.experimentparams.home.x = 0.0
        self.experimentparams.home.y = 0.0
        self.experimentparams.home.timeout = -1
        self.experimentparams.home.tolerance = 2
        
        self.experimentparams.waitEntry = 0.0
        
        self.experimentparams.triggerEntry.enabled = True
        self.experimentparams.triggerEntry.distanceMin =   0.0
        self.experimentparams.triggerEntry.distanceMax = 999.0
        self.experimentparams.triggerEntry.speedMin =   0.0
        self.experimentparams.triggerEntry.speedMax = 999.0
        self.experimentparams.triggerEntry.angleMin =  0.0 * N.pi / 180.0
        self.experimentparams.triggerEntry.angleMax =180.0 * N.pi / 180.0
        self.experimentparams.triggerEntry.angleTest = 'inclusive'
        self.experimentparams.triggerEntry.angleTestBilateral = True
        self.experimentparams.triggerEntry.timeHold = 0.0
        self.experimentparams.triggerEntry.timeout = -1
        
        self.experimentparams.move.enabled = True
        self.experimentparams.move.mode = 'pattern' # 'pattern' or 'relative'
        self.experimentparams.move.pattern.shape = 'circle' # 'constant' or 'circle' or 'square' or 'flylogo' or 'spiral' or 'ramp'
        self.experimentparams.move.pattern.hz = 0.10
        self.experimentparams.move.pattern.count = -1
        self.experimentparams.move.pattern.radius = 32
        self.experimentparams.move.timeout = 600
        
        self.experimentparams.triggerExit.enabled = True
        self.experimentparams.triggerExit.distanceMin = 0.0
        self.experimentparams.triggerExit.distanceMax = 999.0
        self.experimentparams.triggerExit.speedMin =  0.0
        self.experimentparams.triggerExit.speedMax = 999.0
        self.experimentparams.triggerExit.angleMin =  0.0 * N.pi / 180.0
        self.experimentparams.triggerExit.angleMax =180.0 * N.pi / 180.0
        self.experimentparams.triggerExit.angleTest = 'inclusive'
        self.experimentparams.triggerExit.angleTestBilateral = True
        self.experimentparams.triggerExit.timeHold = 0.0
        self.experimentparams.triggerExit.timeout = -1

        self.experiment = ExperimentLib.Experiment(self.experimentparams)


    def Run(self):
        self.experiment.Run()



if __name__ == '__main__':
    try:
        experiment = ExperimentPassiveChase()
        experiment.Run()
        
    except:
        rospy.loginfo("Shutting down")

        
