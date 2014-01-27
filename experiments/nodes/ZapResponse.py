#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('experiments')
import rospy
import numpy as N
import ExperimentLib
from geometry_msgs.msg import Point, Twist
from experiment_srvs.srv import Trigger, ExperimentParams, ExperimentParamsRequest
from flycore.msg import MsgFrameState
from galvodirector.msg import MsgGalvoCommand
from ledpanels.msg import MsgPanelsCommand
from patterngen.msg import MsgPattern
from tracking.msg import ArenaState




#######################################################################################################
class ExperimentZapresponse():
    def __init__(self):
        rospy.init_node('Experiment')
        
        # Fill out the data structure that defines the experiment.
        self.experimentparams = ExperimentParamsRequest()
        
        #flies = 'HCS'
        flies = 'TrpA1_sugar'
        #flies = 'TrpA1_paralysis'
        #flies = 'TrpA1_wingextender'
        t0 = 120     # off 
        t1 = 3      # on
        t2 = 117    # off
        laserpower = '260mW'
        
        
        self.experimentparams.experiment.description = 'Laser is off for %d secs, on for %d secs, and off for %d secs' % (t0, t1, t2)
        self.experimentparams.experiment.maxTrials = 100
        self.experimentparams.experiment.trial = 1
        
        self.experimentparams.save.filenamebase = "zapresponse_%s_%02ds_%02ds_%02ds_%s_" % (flies, t0, t1, t2, laserpower)
        self.experimentparams.save.csv = True
        self.experimentparams.save.bag = False
        self.experimentparams.save.mov = False
        self.experimentparams.save.imagetopic_list = ['camera/image_rect']
        self.experimentparams.save.onlyWhileTriggered = False
        
        self.experimentparams.robotspec.nRobots = 0
        self.experimentparams.robotspec.width = 1.5875
        self.experimentparams.robotspec.height = 1.5875
        self.experimentparams.robotspec.isPresent = True                            # Set this to False if you remove the robot, but still want the actuation.
        self.experimentparams.robotspec.description = "Black oxide magnet"

        self.experimentparams.flyspec.nFlies = 1
        self.experimentparams.flyspec.description = flies
        
        self.experimentparams.tracking.exclusionzones.enabled = False
        self.experimentparams.tracking.exclusionzones.point_list = [Point(x=0.0, y=0.0)]
        self.experimentparams.tracking.exclusionzones.radius_list = [0.0]
        
        self.experimentparams.pre.robot.enabled = False
        self.experimentparams.pre.lasergalvos.enabled = False
        self.experimentparams.pre.ledpanels.enabled = False
        
        self.experimentparams.pre.wait1 = t0
        self.experimentparams.pre.trigger.enabled = False
        self.experimentparams.pre.trigger.frameidParent = 'Arena'
        self.experimentparams.pre.trigger.frameidChild = 'Fly01'
        self.experimentparams.pre.trigger.speedAbsParentMin =   0.0
        self.experimentparams.pre.trigger.speedAbsParentMax = 999.0
        self.experimentparams.pre.trigger.speedAbsChildMin =   0.0
        self.experimentparams.pre.trigger.speedAbsChildMax = 999.0
        self.experimentparams.pre.trigger.speedRelMin =   0.0
        self.experimentparams.pre.trigger.speedRelMax = 999.0
        self.experimentparams.pre.trigger.distanceMin =   0.0
        self.experimentparams.pre.trigger.distanceMax = 999.0
        self.experimentparams.pre.trigger.angleMin =  0.0 * N.pi / 180.0
        self.experimentparams.pre.trigger.angleMax =180.0 * N.pi / 180.0
        self.experimentparams.pre.trigger.angleTest = 'inclusive'
        self.experimentparams.pre.trigger.angleTestBilateral = True
        self.experimentparams.pre.trigger.timeHold = 0.0
        self.experimentparams.pre.trigger.timeout = -1
        self.experimentparams.pre.wait2 = 0.0
        
        
        # .robot, .lasergalvos, .ledpanels, and .post.trigger all run concurrently.
        # The first one to finish preempts the others.
        self.experimentparams.trial.robot.enabled = False
        
        
        flies_list = range(1,1+self.experimentparams.flyspec.nFlies)
        
        self.experimentparams.trial.lasergalvos.enabled = True
        self.experimentparams.trial.lasergalvos.pattern_list = []
        self.experimentparams.trial.lasergalvos.statefilterHi_list = []
        self.experimentparams.trial.lasergalvos.statefilterLo_list = []
        self.experimentparams.trial.lasergalvos.statefilterCriteria_list = []
        for iFly in flies_list:
            self.experimentparams.trial.lasergalvos.pattern_list.append(MsgPattern(
                                                                            frameidPosition   = 'Fly%02dForecast' % iFly,
                                                                            frameidAngle   = 'Fly%02dForecast' % iFly,
                                                                            shape      = 'grid',
                                                                            hzPattern  = 40.0,
                                                                            hzPoint    = 1000.0,
                                                                            count      = 1,
                                                                            size       = Point(x=2,
                                                                                               y=2),
                                                                            preempt    = False,
                                                                            param      = 3, # Peano curve level.
                                                                            direction  = 1),
                                                                 )
        
        
        self.experimentparams.trial.ledpanels.enabled = True
        self.experimentparams.trial.ledpanels.command = 'fixed'  # 'fixed', 'trackposition' (panel position follows fly position), or 'trackview' (panel position follows fly's viewpoint). 
        self.experimentparams.trial.ledpanels.idPattern = 3
        self.experimentparams.trial.ledpanels.frame_id = 'Fly01Forecast'
        self.experimentparams.trial.ledpanels.statefilterHi = ''
        self.experimentparams.trial.ledpanels.statefilterLo = ''
        self.experimentparams.trial.ledpanels.statefilterCriteria = ''

        self.experimentparams.post.trigger.enabled = True
        self.experimentparams.post.trigger.frameidParent = 'Arena'
        self.experimentparams.post.trigger.frameidChild = 'Fly01'
        self.experimentparams.post.trigger.speedAbsParentMin =   0.0
        self.experimentparams.post.trigger.speedAbsParentMax = 999.0
        self.experimentparams.post.trigger.speedAbsChildMin =   0.0
        self.experimentparams.post.trigger.speedAbsChildMax = 999.0
        self.experimentparams.post.trigger.speedRelMin =   0.0
        self.experimentparams.post.trigger.speedRelMax = 999.0
        self.experimentparams.post.trigger.distanceMin = 999.0
        self.experimentparams.post.trigger.distanceMax = 888.0 # i.e. never
        self.experimentparams.post.trigger.angleMin =  0.0 * N.pi / 180.0
        self.experimentparams.post.trigger.angleMax =180.0 * N.pi / 180.0
        self.experimentparams.post.trigger.angleTest = 'inclusive'
        self.experimentparams.post.trigger.angleTestBilateral = True
        self.experimentparams.post.trigger.timeHold = 0.0
        self.experimentparams.post.trigger.timeout = t1

        self.experimentparams.post.wait = t2
        
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
        userdata.experimentparamsOut = userdata.experimentparamsIn
        return 'success'

    # This function gets called at the end of a new trial.  Use this to alter the experiment params from trial to trial.
    def EndTrial_callback(self, userdata):
        userdata.experimentparamsOut = userdata.experimentparamsIn
        return 'success'
        



if __name__ == '__main__':
    experiment = ExperimentZapresponse()
    experiment.Run()
        

        
