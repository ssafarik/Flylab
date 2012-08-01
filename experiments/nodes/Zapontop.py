#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('experiments')
import rospy
import numpy as N
import ExperimentLib
from geometry_msgs.msg import Point
from experiments.srv import *
from galvodirector.msg import MsgGalvoCommand
from patterngen.msg import MsgPattern
from tracking.msg import ArenaState




#######################################################################################################
class ExperimentZapontop():
    def __init__(self):
        rospy.init_node('Experiment')
        
        # Fill out the data structure that defines the experiment.
        self.experimentparams = ExperimentParamsRequest()
        
        self.experimentparams.experiment.description = "When fly y>0 then laser"
        self.experimentparams.experiment.maxTrials = -1
        self.experimentparams.experiment.trial = 1
        
        self.experimentparams.save.filenamebase = "zapontop"
        self.experimentparams.save.arenastate = True
        self.experimentparams.save.video = False
        self.experimentparams.save.bag = False
        self.experimentparams.save.onlyWhileTriggered = False # Saves always.
        
        self.experimentparams.home.enabled = False
        self.experimentparams.home.x = 0.0
        self.experimentparams.home.y = 0.0
        self.experimentparams.home.speed = 20
        self.experimentparams.home.timeout = -1
        self.experimentparams.home.tolerance = 2
        
        self.experimentparams.waitEntry = 5.0 # laser off for at least 5 sec
        
        self.experimentparams.triggerEntry.enabled = True
        self.experimentparams.triggerEntry.frameidParent = 'Plate'
        self.experimentparams.triggerEntry.frameidChild = 'Fly1'
        self.experimentparams.triggerEntry.speedParentMin =   0.0
        self.experimentparams.triggerEntry.speedParentMax = 999.0
        self.experimentparams.triggerEntry.speedChildMin =   0.0
        self.experimentparams.triggerEntry.speedChildMax = 999.0
        self.experimentparams.triggerEntry.distanceMin =   0.0
        self.experimentparams.triggerEntry.distanceMax = 999.0
        self.experimentparams.triggerEntry.angleMin =  0.0 * N.pi / 180.0
        self.experimentparams.triggerEntry.angleMax =180.0 * N.pi / 180.0
        self.experimentparams.triggerEntry.angleTest = 'inclusive'
        self.experimentparams.triggerEntry.angleTestBilateral = False
        self.experimentparams.triggerEntry.timeHold = 0.0
        self.experimentparams.triggerEntry.timeout = -1
        
        
        # .move, .lasertrack, and .triggerExit all run concurrently.
        # The first one to finish preempts the others.
        self.experimentparams.move.enabled = False
        self.experimentparams.move.mode = 'relative'        
        self.experimentparams.move.relative.tracking = True
        self.experimentparams.move.relative.frameidOriginPosition = "Fly1"
        self.experimentparams.move.relative.frameidOriginAngle = "Fly1"
        self.experimentparams.move.relative.distance = 5
        self.experimentparams.move.relative.angle = 180.0 * N.pi / 180.0
        self.experimentparams.move.relative.angleType = 'constant'
        self.experimentparams.move.relative.speed = 200
        self.experimentparams.move.relative.speedType = 'constant'
        self.experimentparams.move.relative.tolerance = -1.0 # i.e. never get there.
        self.experimentparams.move.timeout = -1
        
        
        self.experimentparams.lasertrack.enabled = True
        self.experimentparams.lasertrack.pattern_list = []
        self.experimentparams.lasertrack.pattern_list.append(MsgPattern(mode       = 'byshape',
                                                                        shape      = 'grid',
                                                                        frame_id   = 'Fly1',
                                                                        hzPattern  = 40.0,
                                                                        hzPoint    = 1000.0,
                                                                        count      = 1,
                                                                        size       = Point(x=3,
                                                                                           y=3),
                                                                        preempt    = False,
                                                                        param      = 2), # Peano curve level.
                                                             )
        self.experimentparams.lasertrack.timeout = -1
        
        self.experimentparams.triggerExit.enabled = True
        self.experimentparams.triggerExit.frameidParent = 'Plate'
        self.experimentparams.triggerExit.frameidChild = 'Fly1'
        self.experimentparams.triggerExit.speedParentMin =   0.0
        self.experimentparams.triggerExit.speedParentMax = 999.0
        self.experimentparams.triggerExit.speedChildMin =   0.0
        self.experimentparams.triggerExit.speedChildMax = 999.0
        self.experimentparams.triggerExit.distanceMin = 0.0
        self.experimentparams.triggerExit.distanceMax = 999.0
        self.experimentparams.triggerExit.angleMin =180.0 * N.pi / 180.0
        self.experimentparams.triggerExit.angleMax =359.99 * N.pi / 180.0
        self.experimentparams.triggerExit.angleTest = 'inclusive'
        self.experimentparams.triggerExit.angleTestBilateral = False
        self.experimentparams.triggerExit.timeHold = 0.0
        self.experimentparams.triggerExit.timeout = 5 # laser on for 5 sec

        self.experiment = ExperimentLib.Experiment(self.experimentparams)



    def Run(self):
        self.experiment.Run()
        



if __name__ == '__main__':
    experiment = ExperimentZapontop()
    experiment.Run()
        

        
