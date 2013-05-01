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
from LEDPanels.msg import MsgPanelsCommand
from patterngen.msg import MsgPattern



#######################################################################################################
class Experiment():
    def __init__(self):
        rospy.init_node('Experiment')
        
        # Fill out the data structure that defines the experiment.
        self.experimentparams = ExperimentParamsRequest()
        
        self.experimentparams.experiment.description = "Get chased"
        self.experimentparams.experiment.maxTrials = -1
        self.experimentparams.experiment.trial = 1
        
        self.experimentparams.save.filenamebase = "getchased"
        self.experimentparams.save.csv = True
        self.experimentparams.save.bag = True
        self.experimentparams.save.png = False
        self.experimentparams.save.imagetopic_list = ['camera/image_rect']
        self.experimentparams.save.onlyWhileTriggered = False
        
        self.experimentparams.tracking.exclusionzones.enabled = True
        self.experimentparams.tracking.exclusionzones.point_list = [Point(x=-79, y=28)]
        self.experimentparams.tracking.exclusionzones.radius_list = [3.0]
        
        self.experimentparams.pre.robot.enabled = True
        self.experimentparams.pre.robot.move.mode = 'pattern'                       # 'relative' or 'pattern'.  Move relative to the given frame, or move in a preset pattern.
        self.experimentparams.pre.robot.move.relative.tracking = True               # True=update the target point continually.  False=the target point is set at the trigger time. 
        self.experimentparams.pre.robot.move.relative.frameidOriginPosition = "Fly1"
        self.experimentparams.pre.robot.move.relative.frameidOriginAngle = "Fly1"
        self.experimentparams.pre.robot.move.relative.distance = 35                 # Distance to the target point from the origin frame's position.
        self.experimentparams.pre.robot.move.relative.angle = 0                     # Angle to the target point from the origin frame's x-axis.
        self.experimentparams.pre.robot.move.relative.angleType = 'random'          # 'constant' or 'random'.  Use given angle always, or choose random angle once per move.
        self.experimentparams.pre.robot.move.relative.speed = 20                    # Speed at which to move the robot toward the target point. 
        self.experimentparams.pre.robot.move.relative.speedType = 'constant'        # 'constant' or 'random'.  Use the given value, or a random frpre of it. 
        self.experimentparams.pre.robot.move.relative.tolerance = 2                 # When robot-to-target distance is within this tolerance, then the move is over.
        self.experimentparams.pre.robot.move.pattern.frameidPosition = 'Arena'               # 
        self.experimentparams.pre.robot.move.pattern.frameidAngle = 'Arena'               # 
        self.experimentparams.pre.robot.move.pattern.shape = 'circle'               # 'constant' or 'circle' or 'square' or 'flylogo' or 'spiral' or 'grid'
        self.experimentparams.pre.robot.move.pattern.hzPattern = 1/73 # for 5.2mm/sec               # Patterns per second.
        self.experimentparams.pre.robot.move.pattern.hzPoint = 20                   # The update rate for the actuator.
        self.experimentparams.pre.robot.move.pattern.count = -1
        self.experimentparams.pre.robot.move.pattern.size.x = 60
        self.experimentparams.pre.robot.move.pattern.size.y = 0
        self.experimentparams.pre.robot.move.pattern.param = 0
        self.experimentparams.pre.robot.move.pattern.direction = 1

        self.experimentparams.pre.lasergalvos.enabled = False
        
        self.experimentparams.pre.ledpanels.enabled = True
        self.experimentparams.pre.ledpanels.command = 'fixed'  # 'fixed', 'trackposition' (panel position follows fly position), or 'trackview' (panel position follows fly's viewpoint). 
        self.experimentparams.pre.ledpanels.idPattern = 1
        self.experimentparams.pre.ledpanels.origin.x = 0 
        self.experimentparams.pre.ledpanels.origin.y = 0 
        self.experimentparams.pre.ledpanels.frame_id = 'Fly1Forecast'
        self.experimentparams.pre.ledpanels.statefilterHi = ''
        self.experimentparams.pre.ledpanels.statefilterLo = ''
        self.experimentparams.pre.ledpanels.statefilterCriteria = ''
        
        self.experimentparams.pre.wait1 = 0.0
        self.experimentparams.pre.trigger.enabled = True
        self.experimentparams.pre.trigger.frameidParent = 'Fly1'
        self.experimentparams.pre.trigger.frameidChild = 'Robot'
        self.experimentparams.pre.trigger.speedAbsParentMin =  5.0
        self.experimentparams.pre.trigger.speedAbsParentMax =  40.0
        self.experimentparams.pre.trigger.speedAbsChildMin  =   0.0
        self.experimentparams.pre.trigger.speedAbsChildMax  = 999.0
        self.experimentparams.pre.trigger.speedRelMin       =   0.0
        self.experimentparams.pre.trigger.speedRelMax       = 999.0
        self.experimentparams.pre.trigger.distanceMin =  6.0
        self.experimentparams.pre.trigger.distanceMax = 10.0
        self.experimentparams.pre.trigger.angleMin =  0.0 * N.pi / 180.0
        self.experimentparams.pre.trigger.angleMax = 90.0 * N.pi / 180.0
        self.experimentparams.pre.trigger.angleTest = 'inclusive'
        self.experimentparams.pre.trigger.angleTestBilateral = True
        self.experimentparams.pre.trigger.timeHold = 0.0
        self.experimentparams.pre.trigger.timeout = -1
        self.experimentparams.pre.wait2 = 0.0
        

        # .robot.move, .lasergalvos, and .triggerExit all run concurrently.
        # The first one to finish preempts the others.
        self.experimentparams.trial.robot.enabled = True
        self.experimentparams.trial.robot.move.mode = 'relative'        
        self.experimentparams.trial.robot.move.relative.tracking = True
        self.experimentparams.trial.robot.move.relative.frameidOriginPosition = "Fly1Forecast"
        self.experimentparams.trial.robot.move.relative.frameidOriginAngle = "Fly1Forecast"
        self.experimentparams.trial.robot.move.relative.distance = 8
        self.experimentparams.trial.robot.move.relative.angle = 180.0 * N.pi / 180.0
        self.experimentparams.trial.robot.move.relative.angleType = 'current' # 'constant' or 'random' or 'current'
        self.experimentparams.trial.robot.move.relative.speed = 5.2
        self.experimentparams.trial.robot.move.relative.speedType = 'constant'
        self.experimentparams.trial.robot.move.relative.tolerance = -1.0 # i.e. never get there.
        self.experimentparams.trial.robot.home.enabled = True
        self.experimentparams.trial.robot.home.x = 60.0
        self.experimentparams.trial.robot.home.y = 0.0
        self.experimentparams.trial.robot.home.speed = 20
        self.experimentparams.trial.robot.home.tolerance = 2
        
        
        self.experimentparams.trial.lasergalvos.enabled = False
        
        self.experimentparams.trial.ledpanels.enabled = True
        self.experimentparams.trial.ledpanels.command = 'fixed'  # 'fixed', 'trackposition' (panel position follows fly position), or 'trackview' (panel position follows fly's viewpoint). 
        self.experimentparams.trial.ledpanels.idPattern = 1
        self.experimentparams.trial.ledpanels.origin.x = 0 
        self.experimentparams.trial.ledpanels.origin.y = 0 
        self.experimentparams.trial.ledpanels.frame_id = 'Fly1Forecast'
        self.experimentparams.trial.ledpanels.statefilterHi = ''
        self.experimentparams.trial.ledpanels.statefilterLo = ''
        self.experimentparams.trial.ledpanels.statefilterCriteria = ''

        self.experimentparams.post.trigger.enabled = True
        self.experimentparams.post.trigger.frameidParent = 'Fly1'
        self.experimentparams.post.trigger.frameidChild = 'Robot'
        self.experimentparams.post.trigger.speedAbsParentMin =   0.0
        self.experimentparams.post.trigger.speedAbsParentMax =   3.0
        self.experimentparams.post.trigger.speedAbsChildMin  =   0.0
        self.experimentparams.post.trigger.speedAbsChildMax  = 999.0
        self.experimentparams.post.trigger.speedRelMin       =   0.0
        self.experimentparams.post.trigger.speedRelMax       = 999.0
        self.experimentparams.post.trigger.distanceMin = 0.0
        self.experimentparams.post.trigger.distanceMax = 999.0
        self.experimentparams.post.trigger.angleMin =   0.0 * N.pi / 180.0
        self.experimentparams.post.trigger.angleMax = 180.0 * N.pi / 180.0
        self.experimentparams.post.trigger.angleTest = 'inclusive'
        self.experimentparams.post.trigger.angleTestBilateral = True
        self.experimentparams.post.trigger.timeHold = 4.0
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
        experiment = Experiment()
        experiment.Run()
        
    #except:
        pass

        
