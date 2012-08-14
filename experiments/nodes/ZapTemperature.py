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
class ExperimentZapafly():
    def __init__(self):
        rospy.init_node('Experiment')
        
        # Fill out the data structure that defines the experiment.
        self.experimentparams = ExperimentParamsRequest()
        
        self.experimentparams.experiment.description = "Aim laser at fly"
        self.experimentparams.experiment.maxTrials = -1
        self.experimentparams.experiment.trial = 1
        
        self.experimentparams.save.filenamebase = "zap"
        self.experimentparams.save.arenastate = True
        self.experimentparams.save.video = False
        self.experimentparams.save.bag = False
        self.experimentparams.save.onlyWhileTriggered = True
        
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
        self.experimentparams.triggerEntry.angleTestBilateral = True
        self.experimentparams.triggerEntry.timeHold = 0.0
        self.experimentparams.triggerEntry.timeout = -1
        
        
        # .move, .lasertrack, and .triggerExit all run concurrently.
        # The first one to finish preempts the others.
        self.experimentparams.move.enabled = False
        
        
        mode='platepoint' # 'platepoint' or 'plategrid' or 'trackgrid1' or 'trackgrid' or 'tracknumber' or 'trackflylogo' or 'fixedmaze' or 'fixedpoint'
        
        self.experimentparams.lasertrack.enabled = True
        self.experimentparams.lasertrack.pattern_list = []
        if mode=='platepoint':
            # Draw a point.
            self.experimentparams.lasertrack.pattern_list.append(MsgPattern(mode       = 'byshape',
                                                                            shape      = 'constant',
                                                                            frame_id   = 'Plate',
                                                                            hzPattern  = 40.0,
                                                                            hzPoint    = 1000.0,
                                                                            count      = 1,
                                                                            size       = Point(x=0,
                                                                                               y=0),
                                                                            preempt    = False,
                                                                            param      = 2), # Peano curve level.
                                                                 )
        if mode=='plategrid':
            self.experimentparams.lasertrack.pattern_list.append(MsgPattern(mode       = 'byshape',
                                                                            shape      = 'grid',
                                                                            frame_id   = 'Plate',
                                                                            hzPattern  = 40.0,
                                                                            hzPoint    = 1000.0,
                                                                            count      = 1,
                                                                            size       = Point(x=1,
                                                                                               y=1),
                                                                            preempt    = False,
                                                                            param      = 3), # Peano curve level.
                                                                 )
        if mode=='trackgrid1':
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
        if mode=='trackgrid':
            for iFly in range(rospy.get_param('nFlies', 0)):
                self.experimentparams.lasertrack.pattern_list.append(MsgPattern(mode       = 'byshape',
                                                                                shape      = 'grid',
                                                                                frame_id   = 'Fly%d' % (iFly+1),
                                                                                hzPattern  = 40.0,
                                                                                hzPoint    = 1000.0,
                                                                                count      = 1,
                                                                                size       = Point(x=3,
                                                                                                   y=3),
                                                                                preempt    = False,
                                                                                param      = 2), # Peano curve level.
                                                                     )
        if mode=='tracknumber':
            for iFly in range(rospy.get_param('nFlies', 0)):
                self.experimentparams.lasertrack.pattern_list.append(MsgPattern(mode       = 'byshape',
                                                                                shape      = '%s' % (iFly+1),
                                                                                frame_id   = 'Fly%d' % (iFly+1),
                                                                                hzPattern  = 40.0,
                                                                                hzPoint    = 1000.0,
                                                                                count      = 1,
                                                                                size       = Point(x=6,
                                                                                                   y=6),
                                                                                preempt    = False,
                                                                                param      = 2), # Peano curve level.
                                                                     )
        if mode=='trackflylogo':
            for iFly in range(rospy.get_param('nFlies', 0)):
                self.experimentparams.lasertrack.pattern_list.append(MsgPattern(mode       = 'byshape',
                                                                                shape      = 'flylogo',
                                                                                frame_id   = 'Fly%d' % (iFly+1),
                                                                                hzPattern  = 40.0,
                                                                                hzPoint    = 1000.0,
                                                                                count      = 1,
                                                                                size       = Point(x=6,
                                                                                                   y=6),
                                                                                preempt    = False,
                                                                                param      = 2), # Peano curve level.
                                                                     )
        if mode=='fixedmaze':
            # Draw a maze.
            self.experimentparams.lasertrack.pattern_list.append(MsgPattern(mode       = 'byshape',
                                                                            #shape      = '%s' % (iFly+1),
                                                                            #shape      = 'flylogo',
                                                                            shape      = 'grid',
                                                                            #shape      = 'constant',
                                                                            frame_id   = 'Plate',
                                                                            hzPattern  = 40.0,
                                                                            hzPoint    = 1000.0,
                                                                            count      = 1,
                                                                            size       = Point(x=140,
                                                                                               y=140),
                                                                            preempt    = False,
                                                                            param      = 2), # Peano curve level.
                                                                 )
        if mode=='fixedpoint':
            # Draw a point.
            self.experimentparams.lasertrack.pattern_list.append(MsgPattern(mode       = 'byshape',
                                                                            #shape      = '%s' % (iFly+1),
                                                                            #shape      = 'flylogo',
                                                                            #shape      = 'grid',
                                                                            shape      = 'constant',
                                                                            frame_id   = 'Plate',
                                                                            hzPattern  = 40.0,
                                                                            hzPoint    = 1000.0,
                                                                            count      = 1,
                                                                            size       = Point(x=0,
                                                                                               y=0),
                                                                            preempt    = False,
                                                                            param      = 2), # Peano curve level.
                                                                 )
        
        self.experimentparams.lasertrack.timeout = -1
        
        self.experimentparams.triggerExit.enabled = True
        self.experimentparams.triggerExit.distanceMin = 999.0
        self.experimentparams.triggerExit.distanceMax = 888.0 # i.e. never
        self.experimentparams.triggerExit.speedParentMin =   0.0
        self.experimentparams.triggerExit.speedParentMax = 999.0
        self.experimentparams.triggerExit.speedChildMin =   0.0
        self.experimentparams.triggerExit.speedChildMax = 999.0
        self.experimentparams.triggerExit.angleMin =  0.0 * N.pi / 180.0
        self.experimentparams.triggerExit.angleMax =180.0 * N.pi / 180.0
        self.experimentparams.triggerExit.angleTest = 'inclusive'
        self.experimentparams.triggerExit.angleTestBilateral = True
        self.experimentparams.triggerExit.timeHold = 0.0
        self.experimentparams.triggerExit.timeout = 660

        self.experimentparams.waitExit = 0.0
        
        self.experiment = ExperimentLib.Experiment(self.experimentparams)



    def Run(self):
        self.experiment.Run()
        



if __name__ == '__main__':
    experiment = ExperimentZapafly()
    experiment.Run()
        

        
