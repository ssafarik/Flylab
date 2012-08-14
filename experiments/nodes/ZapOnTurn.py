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
from tracking.msg import ArenaState




#######################################################################################################
class ExperimentZapOnTurn():
    def __init__(self):
        rospy.init_node('Experiment')
        
        # Fill out the data structure that defines the experiment.
        self.experimentparams = ExperimentParamsRequest()
        
        self.experimentparams.experiment.description = "When fly turns then laser"
        self.experimentparams.experiment.maxTrials = -1
        self.experimentparams.experiment.trial = 1
        
        self.experimentparams.save.filenamebase = "zaponturn"
        self.experimentparams.save.arenastate = True
        self.experimentparams.save.video = False
        self.experimentparams.save.bag = False
        self.experimentparams.save.onlyWhileTriggered = False # Saves always.
        
        self.experimentparams.home.enabled = False
        
        self.experimentparams.waitEntry = 0.0
        
        self.experimentparams.triggerEntry.enabled = False
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
        
        
        self.experimentparams.lasertrack.enabled = True
        self.experimentparams.lasertrack.pattern_list = []
        self.experimentparams.lasertrack.stateFilterLo_list = []
        self.experimentparams.lasertrack.stateFilterHi_list = []
        for iFly in range(3):#rospy.get_param('nFlies', 0)):#2):#
            self.experimentparams.lasertrack.pattern_list.append(MsgPattern(mode       = 'byshape',
                                                                            shape      = 'grid',
                                                                            frame_id   = 'Fly%d' % (iFly+1),
                                                                            hzPattern  = 40.0,
                                                                            hzPoint    = 1000.0,
                                                                            count      = 1,
                                                                            size       = Point(x=3,
                                                                                               y=3),
                                                                            preempt    = False,
                                                                            param      = 3), # Peano curve level.
                                                                 )
            #self.experimentparams.lasertrack.stateFilterHi_list.append("{'speed':5.0}")
            #self.experimentparams.lasertrack.stateFilterLo_list.append("{'speed':0.0}")
            #self.experimentparams.lasertrack.stateFilterHi_list.append("{'velocity':{'linear':{'x':+6,'y':+6}}}")
            #self.experimentparams.lasertrack.stateFilterLo_list.append("{'velocity':{'linear':{'x':-6,'y':-6}}}")
            self.experimentparams.lasertrack.stateFilterHi_list.append("{'velocity':{'angular':{'z':999}}}")
            self.experimentparams.lasertrack.stateFilterLo_list.append("{'velocity':{'angular':{'z':0.5}}}")
            #self.experimentparams.lasertrack.stateFilterHi_list.append("{'pose':{'position':{'x':+999, 'y':999}}}")
            #self.experimentparams.lasertrack.stateFilterLo_list.append("{'pose':{'position':{'x':-999, 'y':0}}}")
        
        self.experimentparams.lasertrack.timeout = -1
        
        self.experimentparams.triggerExit.enabled = True
        self.experimentparams.triggerExit.frameidParent = 'Plate'
        self.experimentparams.triggerExit.frameidChild = 'Fly1'
        self.experimentparams.triggerExit.speedParentMin =   0.0
        self.experimentparams.triggerExit.speedParentMax = 999.0
        self.experimentparams.triggerExit.speedChildMin =   0.0
        self.experimentparams.triggerExit.speedChildMax = 999.0
        self.experimentparams.triggerExit.distanceMin = 999.0
        self.experimentparams.triggerExit.distanceMax = 111.0 # i.e. never
        self.experimentparams.triggerExit.angleMin =  0.0000 * N.pi / 180.0
        self.experimentparams.triggerExit.angleMax =359.9999 * N.pi / 180.0
        self.experimentparams.triggerExit.angleTest = 'inclusive'
        self.experimentparams.triggerExit.angleTestBilateral = False
        self.experimentparams.triggerExit.timeHold = 0.0
        self.experimentparams.triggerExit.timeout = 3600

        self.experimentparams.waitExit = 0.0
        
        self.experiment = ExperimentLib.Experiment(self.experimentparams)



    def Run(self):
        self.experiment.Run()
        



if __name__ == '__main__':
    experiment = ExperimentZapOnTurn()
    experiment.Run()
        

        
