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
from LEDPanels.msg import MsgPanelsCommand
from patterngen.msg import MsgPattern



#######################################################################################################
class ExperimentOpenTrigger():
    def __init__(self):
        rospy.init_node('Experiment')
        
        # Fill out the data structure that defines the experiment.
        self.experimentparams = ExperimentParamsRequest()
        
        self.experimentparams.experiment.description = "Open triggering"
        self.experimentparams.experiment.maxTrials = -1
        self.experimentparams.experiment.trial = 1
        
        self.experimentparams.save.filenamebase = "open"
        self.experimentparams.save.arenastate = True
        self.experimentparams.save.video = False
        self.experimentparams.save.bag = False
        self.experimentparams.save.onlyWhileTriggered = True
        
        self.experimentparams.tracking.exclusionzones.enabled = False
        self.experimentparams.tracking.exclusionzones.point_list = [Point(x=0.0, y=0.0)]
        self.experimentparams.tracking.exclusionzones.radius_list = [0.0]
        
        self.experimentparams.pre.robot.enabled = False
        self.experimentparams.pre.lasertrack.enabled = False
        self.experimentparams.pre.ledpanels.enabled = False
        self.experimentparams.pre.wait1 = 0.0
        
        self.experimentparams.pre.trigger.enabled = True
        self.experimentparams.pre.trigger.frameidParent = 'Fly1'
        self.experimentparams.pre.trigger.frameidChild = 'Robot'
        self.experimentparams.pre.trigger.speedAbsParentMin =   0.0
        self.experimentparams.pre.trigger.speedAbsParentMax = 999.0
        self.experimentparams.pre.trigger.speedAbsChildMin  =   0.0
        self.experimentparams.pre.trigger.speedAbsChildMax  = 999.0
        self.experimentparams.pre.trigger.speedRelMin       =   0.0
        self.experimentparams.pre.trigger.speedRelMax       = 999.0
        self.experimentparams.pre.trigger.distanceMin =   0.0
        self.experimentparams.pre.trigger.distanceMax = 999.0
        self.experimentparams.pre.trigger.angleMin =  0.0 * N.pi / 180.0
        self.experimentparams.pre.trigger.angleMax =180.0 * N.pi / 180.0
        self.experimentparams.pre.trigger.angleTest = 'inclusive'
        self.experimentparams.pre.trigger.angleTestBilateral = True
        self.experimentparams.pre.trigger.timeHold = 0.0
        self.experimentparams.pre.trigger.timeout = -1
        
        self.experimentparams.pre.wait2 = 0.0
        

        # .robot.move, .lasertrack, and .post.trigger all run concurrently.
        # The first one to finish preempts the others.
        self.experimentparams.trial.robot.enabled = True
        self.experimentparams.trial.robot.move.mode = 'relative'
        self.experimentparams.trial.robot.move.relative.tracking = True
        self.experimentparams.trial.robot.move.relative.frameidOriginPosition = "Arena"
        self.experimentparams.trial.robot.move.relative.frameidOriginAngle = "Arena"
        self.experimentparams.trial.robot.move.relative.distance = 40
        self.experimentparams.trial.robot.move.relative.angle =  90.0 * N.pi / 180.0
        self.experimentparams.trial.robot.move.relative.angleType = 'constant'
        self.experimentparams.trial.robot.move.relative.speed = 20
        self.experimentparams.trial.robot.move.relative.speedType = 'constant'
        self.experimentparams.trial.robot.move.relative.tolerance = 2
        self.experimentparams.trial.robot.move.timeout = -1
        self.experimentparams.trial.robot.home.enabled = True
        self.experimentparams.trial.robot.home.x = 0.0
        self.experimentparams.trial.robot.home.y = 0.0
        self.experimentparams.trial.robot.home.speed = 20
        self.experimentparams.trial.robot.home.timeout = -1
        self.experimentparams.trial.robot.home.tolerance = 2
        
        
        self.experimentparams.trial.lasertrack.enabled = False
        
        self.experimentparams.trial.ledpanels.enabled = False
        self.experimentparams.trial.ledpanels.command = 'fixed'  # 'fixed', 'trackposition' (panel position follows fly position), or 'trackview' (panel position follows fly's viewpoint). 
        self.experimentparams.trial.ledpanels.idPattern = 1
        self.experimentparams.trial.ledpanels.frame_id = 'Fly1Forecast'
        self.experimentparams.trial.ledpanels.statefilterHi = ''
        self.experimentparams.trial.ledpanels.statefilterLo = ''
        self.experimentparams.trial.ledpanels.statefilterCriteria = ''
        self.experimentparams.trial.ledpanels.timeout = -1

        self.experimentparams.post.trigger.enabled = True
        self.experimentparams.post.trigger.frameidParent = 'Fly1'
        self.experimentparams.post.trigger.frameidChild = 'Robot'
        self.experimentparams.post.trigger.speedAbsParentMin =   0.0
        self.experimentparams.post.trigger.speedAbsParentMax = 999.0
        self.experimentparams.post.trigger.speedAbsChildMin  =   0.0
        self.experimentparams.post.trigger.speedAbsChildMax  = 999.0
        self.experimentparams.post.trigger.speedRelMin       =   0.0
        self.experimentparams.post.trigger.speedRelMax       = 999.0
        self.experimentparams.post.trigger.distanceMin = 0.0
        self.experimentparams.post.trigger.distanceMax = 999.0
        self.experimentparams.post.trigger.angleMin =  0.0 * N.pi / 180.0
        self.experimentparams.post.trigger.angleMax =180.0 * N.pi / 180.0
        self.experimentparams.post.trigger.angleTest = 'inclusive'
        self.experimentparams.post.trigger.angleTestBilateral = True
        self.experimentparams.post.trigger.timeHold = 0.0
        self.experimentparams.post.trigger.timeout = -1

        self.experimentparams.post.wait = 0.0
        
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
    def Endtrial_callbackTrialend_callback(self, userdata):
        userdata.experimentparamsOut = userdata.experimentparamsIn
        return 'success'


if __name__ == '__main__':
    try:
        experiment = ExperimentOpenTrigger()
        experiment.Run()
        
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

        
