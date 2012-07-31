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
        self.experimentparams.home.x = 0.0
        self.experimentparams.home.y = 0.0
        self.experimentparams.home.speed = 20
        self.experimentparams.home.timeout = -1
        self.experimentparams.home.tolerance = 2
        
        self.experimentparams.waitEntry = 0.0
        
        self.experimentparams.triggerEntry.enabled = False
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
        
        
        mode='trackgrid1' # 'trackpoint1' or 'trackgrid1' or 'trackgrid' or 'tracknumber' or 'trackflylogo' or 'fixedmaze' or 'fixedpoint'
        
        self.experimentparams.lasertrack.enabled = True
        self.experimentparams.lasertrack.pattern_list = []
        if mode=='trackpoint1':
            # Draw a point.
            self.experimentparams.lasertrack.pattern_list.append(MsgPattern(mode       = 'byshape',
                                                                            shape      = 'constant',
                                                                            frame_id   = 'Fly1',
                                                                            hzPattern  = 40.0,
                                                                            hzPoint    = 1000.0,
                                                                            count      = 1,
                                                                            size       = Point(x=0,
                                                                                               y=0),
                                                                            preempt    = False,
                                                                            param      = 2), # Peano curve level.
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
        self.experimentparams.triggerExit.speedMin =  0.0
        self.experimentparams.triggerExit.speedMax = 999.0
        self.experimentparams.triggerExit.angleMin =  0.0 * N.pi / 180.0
        self.experimentparams.triggerExit.angleMax =180.0 * N.pi / 180.0
        self.experimentparams.triggerExit.angleTest = 'inclusive'
        self.experimentparams.triggerExit.angleTestBilateral = True
        self.experimentparams.triggerExit.timeHold = 0.0
        self.experimentparams.triggerExit.timeout = 660

        self.experiment = ExperimentLib.Experiment(self.experimentparams)



    def Run(self):
        self.experiment.Run()
        



if __name__ == '__main__':
    experiment = ExperimentZapafly()
    experiment.Run()
        

        
