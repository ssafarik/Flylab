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
        
        self.experimentparams.experiment.description = "Laser tracks all flies for 30min"
        self.experimentparams.experiment.maxTrials = -1
        self.experimentparams.experiment.trial = 1
        
        self.experimentparams.save.filenamebase = "zap"
        self.experimentparams.save.arenastate = True
        self.experimentparams.save.video = False
        self.experimentparams.save.bag = False
        self.experimentparams.save.onlyWhileTriggered = False
        
        self.experimentparams.tracking.exclusionzone.enabled = False
        self.experimentparams.tracking.exclusionzone.point_list = [Point(x=0.0, y=0.0)]
        self.experimentparams.tracking.exclusionzone.radius_list = [0.0]
        
        self.experimentparams.home.enabled = False
        
        self.experimentparams.waitEntry = 0.0
        
        self.experimentparams.triggerEntry.enabled = False
        self.experimentparams.triggerEntry.frameidParent = 'Plate'
        self.experimentparams.triggerEntry.frameidChild = 'Fly1'
        self.experimentparams.triggerEntry.speedAbsParentMin =   0.0
        self.experimentparams.triggerEntry.speedAbsParentMax = 999.0
        self.experimentparams.triggerEntry.speedAbsChildMin =   0.0
        self.experimentparams.triggerEntry.speedAbsChildMax = 999.0
        self.experimentparams.triggerEntry.speedRelMin =   0.0
        self.experimentparams.triggerEntry.speedRelMax = 999.0
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
        
        
        #mode='fixedpointlist'    # Laser to specific locations.
        #mode='fixedcircle'
        #mode='fixedmaze' 
        #mode='trackgrid'        # Small grid tracks flies.
        mode='tracknumber'      # Draw a numeral on flies.
        #mode='trackflylogo'
        flies_list = range(1,1+rospy.get_param('nFlies', 0))
        
        
        self.experimentparams.lasertrack.enabled = True
        self.experimentparams.lasertrack.pattern_list = []
        self.experimentparams.lasertrack.statefilterLo_list = []
        self.experimentparams.lasertrack.statefilterHi_list = []
        self.experimentparams.lasertrack.statefilterCriteria_list = []
        if mode=='fixedpointlist':
            # Draw a point.
            self.experimentparams.lasertrack.pattern_list.append(MsgPattern(mode       = 'bypoints',
                                                                            shape      = 'constant',
                                                                            frame_id   = 'Plate',
                                                                            hzPattern  = 40.0,
                                                                            hzPoint    = 1000.0,
                                                                            count      = 1,
                                                                            points     = [Point(x=34  ,y=34),
                                                                                          Point(x=34  ,y=34+6),
                                                                                          Point(x=34  ,y=34-6),
                                                                                          Point(x=34+6,y=34),
                                                                                          Point(x=34-6,y=34)],
                                                                            size       = Point(x=0,
                                                                                               y=0),
                                                                            preempt    = False,
                                                                            param      = 3), # Peano curve level.
                                                                 )
        if mode=='fixedcircle':
            # Draw a point.
            self.experimentparams.lasertrack.pattern_list.append(MsgPattern(mode       = 'byshape',
                                                                            shape      = 'circle',
                                                                            frame_id   = 'Plate',
                                                                            hzPattern  = 40.0,
                                                                            hzPoint    = 1000.0,
                                                                            count      = 1,
                                                                            size       = Point(x=80,
                                                                                               y= 0),
                                                                            preempt    = False,
                                                                            param      = 3), # Peano curve level.
                                                                 )
        if mode=='trackgrid':
            for iFly in flies_list:
                self.experimentparams.lasertrack.pattern_list.append(MsgPattern(mode       = 'byshape',
                                                                                shape      = 'grid',
                                                                                frame_id   = 'Fly%d' % iFly,
                                                                                hzPattern  = 40.0,
                                                                                hzPoint    = 1000.0,
                                                                                count      = 1,
                                                                                size       = Point(x=3,
                                                                                                   y=3),
                                                                                preempt    = False,
                                                                                param      = 3), # Peano curve level.
                                                                     )
        if mode=='tracknumber':
            for iFly in flies_list:
                self.experimentparams.lasertrack.pattern_list.append(MsgPattern(mode       = 'byshape',
                                                                                shape      = '%s' % iFly,
                                                                                frame_id   = 'Fly%dForecast' % iFly,
                                                                                hzPattern  = 10.0,
                                                                                hzPoint    = 1000.0,
                                                                                count      = 1,
                                                                                size       = Point(x=6,
                                                                                                   y=6),
                                                                                preempt    = False,
                                                                                param      = 0), # Peano curve level.
                                                                     )
        if mode=='trackflylogo':
            for iFly in flies_list:
                self.experimentparams.lasertrack.pattern_list.append(MsgPattern(mode       = 'byshape',
                                                                                shape      = 'flylogo',
                                                                                frame_id   = 'Fly%d' % iFly,
                                                                                hzPattern  = 40.0,
                                                                                hzPoint    = 1000.0,
                                                                                count      = 1,
                                                                                size       = Point(x=6,
                                                                                                   y=6),
                                                                                preempt    = False,
                                                                                param      = 0), # Peano curve level.
                                                                     )
        if mode=='fixedmaze':
            # Draw a maze.
            self.experimentparams.lasertrack.pattern_list.append(MsgPattern(mode       = 'byshape',
                                                                            shape      = 'grid',
                                                                            frame_id   = 'Plate',
                                                                            hzPattern  = 40.0,
                                                                            hzPoint    = 1000.0,
                                                                            count      = 1,
                                                                            size       = Point(x=140,
                                                                                               y=140),
                                                                            preempt    = False,
                                                                            param      = 2), # Peano curve level.
                                                                 )
        
        self.experimentparams.lasertrack.timeout = -1
        
        self.experimentparams.triggerExit.enabled = True
        self.experimentparams.triggerExit.distanceMin = 999.0
        self.experimentparams.triggerExit.distanceMax = 888.0 # i.e. never
        self.experimentparams.triggerExit.speedAbsParentMin =   0.0
        self.experimentparams.triggerExit.speedAbsParentMax = 999.0
        self.experimentparams.triggerExit.speedAbsChildMin =   0.0
        self.experimentparams.triggerExit.speedAbsChildMax = 999.0
        self.experimentparams.triggerExit.speedRelMin =   0.0
        self.experimentparams.triggerExit.speedRelMax = 999.0
        self.experimentparams.triggerExit.angleMin =  0.0 * N.pi / 180.0
        self.experimentparams.triggerExit.angleMax =180.0 * N.pi / 180.0
        self.experimentparams.triggerExit.angleTest = 'inclusive'
        self.experimentparams.triggerExit.angleTestBilateral = True
        self.experimentparams.triggerExit.timeHold = 0.0
        self.experimentparams.triggerExit.timeout = 1800

        self.experimentparams.waitExit = 0.0
        
        self.experiment = ExperimentLib.Experiment(self.experimentparams)



    def Run(self):
        self.experiment.Run()
        



if __name__ == '__main__':
    experiment = ExperimentZapafly()
    experiment.Run()
        

        
