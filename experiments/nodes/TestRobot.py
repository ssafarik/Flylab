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
class ExperimentChase():
    def __init__(self):
        rospy.init_node('Experiment')
        
        # Fill out the data structure that defines the experiment.
        self.experimentparams = ExperimentParamsRequest()
        
        self.experimentparams.experiment.description = "Testing Robot Movement"
        self.experimentparams.experiment.maxTrials = -1
        self.experimentparams.experiment.trial = 1
        
        self.experimentparams.save.filenamebase = "test"
        self.experimentparams.save.arenastate = False
        self.experimentparams.save.images = False
        self.experimentparams.save.imagetopic_list = ['camera/image_rect']
        self.experimentparams.save.onlyWhileTriggered = True
        
        self.experimentparams.tracking.exclusionzones.enabled = False
        self.experimentparams.tracking.exclusionzones.point_list = [Point(x=50.0, y=68.0)]
        self.experimentparams.tracking.exclusionzones.radius_list = [3.0]
        
        self.experimentparams.pre.robot.enabled = False
        self.experimentparams.pre.lasergalvos.enabled = False
        self.experimentparams.pre.ledpanels.enabled = False

        self.experimentparams.pre.wait1 = 0.0
        self.experimentparams.pre.trigger.enabled = False
        self.experimentparams.pre.wait2 = 0.0
        

        # .robot.move, .lasergalvos, and .triggerExit all run concurrently.
        # The first one to finish preempts the others.
        self.experimentparams.trial.robot.enabled = True
        self.experimentparams.trial.robot.move.mode = 'pattern'        
        self.experimentparams.trial.robot.move.relative.tracking = False
        self.experimentparams.trial.robot.move.relative.frameidOriginPosition = "Robot"
        self.experimentparams.trial.robot.move.relative.frameidOriginAngle = "Fly1Forecast"
        self.experimentparams.trial.robot.move.relative.distance = 20
        self.experimentparams.trial.robot.move.relative.angle = 0.0 * N.pi / 180.0
        self.experimentparams.trial.robot.move.relative.angleType = 'constant'
        self.experimentparams.trial.robot.move.relative.speed = 20
        self.experimentparams.trial.robot.move.relative.speedType = 'constant'
        self.experimentparams.trial.robot.move.relative.tolerance = -1.0 # i.e. never get there.
        self.experimentparams.trial.robot.move.pattern.frameidPosition = 'Arena'               # 
        self.experimentparams.trial.robot.move.pattern.frameidAngle = 'Arena'               # 

# Four points step response.
#        self.experimentparams.trial.robot.move.pattern.shape = 'square'               # 'constant' or 'circle' or 'square' or 'flylogo' or 'spiral' or 'grid'
#        self.experimentparams.trial.robot.move.pattern.hzPattern = 1/16               # Patterns per second.
#        self.experimentparams.trial.robot.move.pattern.hzPoint = 1/4                   # Points per second in the pattern.

# Smooth circle.
        self.experimentparams.trial.robot.move.pattern.shape = 'circle'               # 'constant' or 'circle' or 'square' or 'flylogo' or 'spiral' or 'grid'
        self.experimentparams.trial.robot.move.pattern.hzPattern = 1/20               # Patterns per second.
        self.experimentparams.trial.robot.move.pattern.hzPoint = 50                   # Points per second in the pattern.

        self.experimentparams.trial.robot.move.pattern.count = -1
        self.experimentparams.trial.robot.move.pattern.size.x = 40
        self.experimentparams.trial.robot.move.pattern.size.y = 0
        self.experimentparams.trial.robot.move.pattern.param = 0
        self.experimentparams.trial.robot.move.pattern.direction = 1
        self.experimentparams.trial.robot.home.enabled = True
        self.experimentparams.trial.robot.home.x = 0.0
        self.experimentparams.trial.robot.home.y = 0.0
        self.experimentparams.trial.robot.home.speed = 20
        self.experimentparams.trial.robot.home.tolerance = 2

        self.experimentparams.trial.lasergalvos.enabled = False
        self.experimentparams.trial.ledpanels.enabled = False


        self.experimentparams.post.trigger.enabled = True
        self.experimentparams.post.trigger.frameidParent = 'Fly1'
        self.experimentparams.post.trigger.frameidChild = 'Robot'
        self.experimentparams.post.trigger.speedAbsParentMin = 999.0
        self.experimentparams.post.trigger.speedAbsParentMax = 111.0 # i.e. Never.
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
    #try:
        experiment = ExperimentChase()
        experiment.Run()
        
    #except:
        pass

        
