#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('experiments')
import rospy
import numpy as np
import ExperimentLib
from geometry_msgs.msg import Point, Twist
from experiment_srvs.srv import Trigger, ExperimentParams, ExperimentParamsRequest, ExperimentParamsChoicesRequest
from flycore.msg import MsgFrameState
from galvodirector.msg import MsgGalvoCommand
from ledpanels.msg import MsgPanelsCommand
from patterngen.msg import MsgPattern
from tracking.msg import ArenaState




#######################################################################################################
class ExperimentZapafly():
    def __init__(self):
        rospy.init_node('Experiment')
        
        # Fill out the data structure that defines the experiment.
        self.experimentparams = ExperimentParamsChoicesRequest()
        
        self.experimentparams.experiment.description = 'Aim laser at fly'
        self.experimentparams.experiment.maxTrials = -1
        self.experimentparams.experiment.timeout = -1
        
        self.experimentparams.save.filenamebase = 'zap'
        self.experimentparams.save.csv = True
        self.experimentparams.save.bag = False
        self.experimentparams.save.mov = False
        self.experimentparams.save.fmf = False
        self.experimentparams.save.imagetopic_list = ['camera/image_rect']
        self.experimentparams.save.onlyWhileTriggered = True
        
        self.experimentparams.robotspec.nRobots = 0
        self.experimentparams.robotspec.width = 1.5875
        self.experimentparams.robotspec.height = 1.5875
        self.experimentparams.robotspec.isPresent = True                            # Set this to False if you remove the robot, but still want the actuation.
        self.experimentparams.robotspec.description = 'Black oxide magnet'

        self.experimentparams.flyspec.nFlies = 1
        self.experimentparams.flyspec.description = 'unspecified'
        
        self.experimentparams.tracking.exclusionzones.enabled = False
        self.experimentparams.tracking.exclusionzones.point_list = [Point(x=0.0, y=0.0)]
        self.experimentparams.tracking.exclusionzones.radius_list = [0.0]

        self.experimentparams.home.enabled = False
        
        self.experimentparams.pre.robot.enabled = False
        self.experimentparams.pre.lasergalvos.enabled = False
        self.experimentparams.pre.ledpanels.enabled = False

        self.experimentparams.pre.wait1 = 0.0
        self.experimentparams.pre.trigger.enabled = False
        self.experimentparams.pre.trigger.frameidParent = 'Arena'
        self.experimentparams.pre.trigger.frameidChild = 'Fly01'
        self.experimentparams.pre.trigger.speedAbsParentMin =   0.0
        self.experimentparams.pre.trigger.speedAbsParentMax = 999.0
        self.experimentparams.pre.trigger.speedAbsChildMin  =   0.0
        self.experimentparams.pre.trigger.speedAbsChildMax  = 999.0
        self.experimentparams.pre.trigger.speedRelMin       =   0.0
        self.experimentparams.pre.trigger.speedRelMax       = 999.0
        self.experimentparams.pre.trigger.distanceMin =   0.0
        self.experimentparams.pre.trigger.distanceMax = 999.0
        self.experimentparams.pre.trigger.angleMin =  0.0 * np.pi / 180.0
        self.experimentparams.pre.trigger.angleMax =180.0 * np.pi / 180.0
        self.experimentparams.pre.trigger.angleTest = 'inclusive'
        self.experimentparams.pre.trigger.angleTestBilateral = True
        self.experimentparams.pre.trigger.timeHold = 0.0
        self.experimentparams.pre.trigger.timeout = -1
        self.experimentparams.pre.wait2 = 0.0
        
        
        # .robot, .lasergalvos, .ledpanels, and .post.trigger all run concurrently.
        # The first one to finish preempts the others.
        self.experimentparams.trial.robot.enabled = False
        
        
        mode='platepoint' # 'platepoint' or 'plategrid' or 'trackgrid1' or 'trackgrid' or 'tracknumber' or 'trackflylogo' or 'fixedmaze' or 'fixedpoint'
        
        self.experimentparams.trial.lasergalvos.enabled = True
        self.experimentparams.trial.lasergalvos.pattern_list = []
        if mode=='platepoint':
            # Draw a point.
            self.experimentparams.trial.lasergalvos.pattern_list.append(MsgPattern(
                                                                            frameidPosition   = 'Arena',
                                                                            frameidAngle   = 'Arena',
                                                                            shape      = 'constant',
                                                                            hzPattern  = 40.0,
                                                                            hzPoint    = 1000.0,
                                                                            count      = 1,
                                                                            size       = Point(x=0,
                                                                                               y=0),
                                                                            restart    = False,
                                                                            param      = 2,
                                                                            direction  = 1), # Peano curve level.
                                                                 )
        if mode=='plategrid':
            self.experimentparams.trial.lasergalvos.pattern_list.append(MsgPattern(
                                                                            frameidPosition   = 'Arena',
                                                                            frameidAngle   = 'Arena',
                                                                            shape      = 'grid',
                                                                            hzPattern  = 40.0,
                                                                            hzPoint    = 1000.0,
                                                                            count      = 1,
                                                                            size       = Point(x=1,
                                                                                               y=1),
                                                                            restart    = False,
                                                                            param      = 3,
                                                                            direction  = 1), # Peano curve level.
                                                                 )
        if mode=='trackgrid1':
            self.experimentparams.trial.lasergalvos.pattern_list.append(MsgPattern(
                                                                            frameidPosition   = 'Fly01',
                                                                            frameidAngle   = 'Fly01',
                                                                            shape      = 'grid',
                                                                            hzPattern  = 40.0,
                                                                            hzPoint    = 1000.0,
                                                                            count      = 1,
                                                                            size       = Point(x=3,
                                                                                               y=3),
                                                                            restart    = False,
                                                                            param      = 2,
                                                                            direction  = 1), # Peano curve level.
                                                                 )
        if mode=='trackgrid':
            for iFly in range(self.experimentparams.flyspec.nFlies):
                self.experimentparams.trial.lasergalvos.pattern_list.append(MsgPattern(
                                                                                frameidPosition   = 'Fly%02dForecast' % (iFly+1),
                                                                                frameidAngle   = 'Fly%02dForecast' % (iFly+1),
                                                                                shape      = 'grid',
                                                                                hzPattern  = 40.0,
                                                                                hzPoint    = 1000.0,
                                                                                count      = 1,
                                                                                size       = Point(x=3,
                                                                                                   y=3),
                                                                                restart    = False,
                                                                                param      = 2,
                                                                                direction  = 1), # Peano curve level.
                                                                     )
        if mode=='tracknumber':
            for iFly in range(self.experimentparams.flyspec.nFlies):
                self.experimentparams.trial.lasergalvos.pattern_list.append(MsgPattern(
                                                                                frameidPosition   = 'Fly%02dForecast' % (iFly+1),
                                                                                frameidAngle   = 'Fly%02dForecast' % (iFly+1),
                                                                                shape      = '%s' % (iFly+1),
                                                                                hzPattern  = 40.0,
                                                                                hzPoint    = 1000.0,
                                                                                count      = 1,
                                                                                size       = Point(x=6,
                                                                                                   y=6),
                                                                                restart    = False,
                                                                                param      = 2,
                                                                                direction  = 1), # Peano curve level.
                                                                     )
        if mode=='trackflylogo':
            for iFly in range(self.experimentparams.flyspec.nFlies):
                self.experimentparams.trial.lasergalvos.pattern_list.append(MsgPattern(
                                                                                frameidPosition   = 'Fly%02dForecast' % (iFly+1),
                                                                                frameidAngle   = 'Fly%02dForecast' % (iFly+1),
                                                                                shape      = 'flylogo',
                                                                                hzPattern  = 40.0,
                                                                                hzPoint    = 1000.0,
                                                                                count      = 1,
                                                                                size       = Point(x=6,
                                                                                                   y=6),
                                                                                restart    = False,
                                                                                param      = 2,
                                                                                direction  = 1), # Peano curve level.
                                                                     )
        if mode=='fixedmaze':
            # Draw a maze.
            self.experimentparams.trial.lasergalvos.pattern_list.append(MsgPattern(
                                                                                frameidPosition   = 'Arena',
                                                                                frameidAngle   = 'Arena',
                                                                                shape      = 'grid',
                                                                                hzPattern  = 40.0,
                                                                                hzPoint    = 1000.0,
                                                                                count      = 1,
                                                                                size       = Point(x=140,
                                                                                                   y=140),
                                                                                restart    = False,
                                                                                param      = 2,
                                                                                direction  = 1), # Peano curve level.
                                                                     )
        if mode=='fixedpoint':
            # Draw a point.
            self.experimentparams.trial.lasergalvos.pattern_list.append(MsgPattern(
                                                                                frameidPosition   = 'Arena',
                                                                                frameidAngle   = 'Arena',
                                                                                shape      = 'constant',
                                                                                hzPattern  = 40.0,
                                                                                hzPoint    = 1000.0,
                                                                                count      = 1,
                                                                                size       = Point(x=0,
                                                                                                   y=0),
                                                                                restart    = False,
                                                                                param      = 2,
                                                                                direction  = 1), # Peano curve level.
                                                                     )
        
        
        self.experimentparams.trial.ledpanels.enabled = False
        self.experimentparams.trial.ledpanels.command = ['fixed']  # 'fixed', 'trackposition' (panel position follows fly position), or 'trackview' (panel position follows fly's viewpoint). 
        self.experimentparams.trial.ledpanels.idPattern = [1]
        self.experimentparams.trial.ledpanels.frame_id = ['Fly01Forecast']
        self.experimentparams.trial.ledpanels.statefilterHi = ['']
        self.experimentparams.trial.ledpanels.statefilterLo = ['']
        self.experimentparams.trial.ledpanels.statefilterCriteria = ['']

        self.experimentparams.post.trigger.enabled = True
        self.experimentparams.post.trigger.distanceMin = 999.0
        self.experimentparams.post.trigger.distanceMax = 888.0 # i.e. never
        self.experimentparams.post.trigger.speedAbsParentMin =   0.0
        self.experimentparams.post.trigger.speedAbsParentMax = 999.0
        self.experimentparams.post.trigger.speedAbsChildMin =   0.0
        self.experimentparams.post.trigger.speedAbsChildMax = 999.0
        self.experimentparams.post.trigger.speedRelMin =   0.0
        self.experimentparams.post.trigger.speedRelMax = 999.0
        self.experimentparams.post.trigger.angleMin =  0.0 * np.pi / 180.0
        self.experimentparams.post.trigger.angleMax =180.0 * np.pi / 180.0
        self.experimentparams.post.trigger.angleTest = 'inclusive'
        self.experimentparams.post.trigger.angleTestBilateral = True
        self.experimentparams.post.trigger.timeHold = 0.0
        self.experimentparams.post.trigger.timeout = 660

        self.experimentparams.post.wait = 0.0
        
        self.experimentlib = ExperimentLib.ExperimentLib(self.experimentparams, 
                                                         startexperiment_callback = self.StartExperiment_callback, 
                                                         starttrial_callback = self.StartTrial_callback, 
                                                         endtrial_callback = self.EndTrial_callback)



    def Run(self):
        self.experimentlib.Run()
        

    # This function gets called at the start of a new experiment.  Use this to do any one-time initialization of hardware, etc.
    def StartExperiment_callback(self, userdata):
        return 'success'

    # This function gets called at the start of a new trial.  Use this to alter the experiment params from trial to trial.
    def StartTrial_callback(self, userdata):
        userdata.experimentparamsChoicesOut = userdata.experimentparamsChoicesIn
        return 'success'

    # This function gets called at the end of a new trial.  Use this to alter the experiment params from trial to trial.
    def EndTrial_callback(self, userdata):
        userdata.experimentparamsOut = userdata.experimentparamsIn
        return 'success'




if __name__ == '__main__':
    experiment = ExperimentZapafly()
    experiment.Run()
        

        
