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
class Experiment():
    def __init__(self):
        rospy.init_node('Experiment')
        
        # Fill out the data structure that defines the experiment.
        self.experimentparams = ExperimentParamsRequest()
        
        self.experimentparams.experiment.description = 'Navigation between two food spots with a movable pole'
        self.experimentparams.experiment.maxTrials = -1
        self.experimentparams.experiment.timeout = -1
        
        self.experimentparams.save.filenamebase = 'poledance'
        self.experimentparams.save.csv = True
        self.experimentparams.save.bag = True
        self.experimentparams.save.mov = False
        self.experimentparams.save.imagetopic_list = ['camera/image_rect']
        self.experimentparams.save.onlyWhileTriggered = False
        
        self.experimentparams.robotspec.nRobots = 1
        self.experimentparams.robotspec.width = 1.5875
        self.experimentparams.robotspec.height = 1.5875
        self.experimentparams.robotspec.isPresent = True                            # Set this to False if you remove the robot, but still want the actuation.
        self.experimentparams.robotspec.description = 'Black oxide magnet'

        self.experimentparams.flyspec.nFlies = 1
        self.experimentparams.flyspec.description = 'unspecified'
        
        self.experimentparams.tracking.exclusionzones.enabled = False
        self.experimentparams.tracking.exclusionzones.point_list = [Point(x=0.00304053, y=0.00015492)]
        self.experimentparams.tracking.exclusionzones.radius_list = [0]
        
        self.experimentparams.pre.robot.enabled = True
        self.experimentparams.pre.robot.move.mode = 'pattern'                       # 'relative' or 'pattern'.  Move relative to the given frame, or move in a preset pattern.
        self.experimentparams.pre.robot.move.pattern.frameidPosition = 'Arena'
        self.experimentparams.pre.robot.move.pattern.frameidAngle = 'Arena'
        self.experimentparams.pre.robot.move.pattern.shape = 'constant'             # 'constant' or 'circle' or 'square' or 'flylogo' or 'spiral' or 'grid'
        self.experimentparams.pre.robot.move.pattern.hzPattern = 1/10               # Patterns per second.0
        self.experimentparams.pre.robot.move.pattern.hzPoint = 20                   # The update rate for the actuator.
        self.experimentparams.pre.robot.move.pattern.count = -1
        self.experimentparams.pre.robot.move.pattern.size.x = 0
        self.experimentparams.pre.robot.move.pattern.size.y = 0
        self.experimentparams.pre.robot.move.pattern.direction = 0                  # Step forward (+1) or reverse (-1) through the pattern points.  0 means choose at random, +1 or -1.

        self.experimentparams.pre.lasergalvos.enabled = False
        self.experimentparams.pre.ledpanels.enabled = False
        self.experimentparams.pre.wait1 = 0.0
        
        self.experimentparams.pre.trigger.enabled = True
        self.experimentparams.pre.trigger.frameidParent = 'Arena'
        self.experimentparams.pre.trigger.frameidChild = 'Fly01'
        self.experimentparams.pre.trigger.speedAbsParentMin =   0.0
        self.experimentparams.pre.trigger.speedAbsParentMax = 999.0
        self.experimentparams.pre.trigger.speedAbsChildMin =   0.0
        self.experimentparams.pre.trigger.speedAbsChildMax =   4.0
        self.experimentparams.pre.trigger.speedRelMin =   0.0
        self.experimentparams.pre.trigger.speedRelMax = 999.0
        self.experimentparams.pre.trigger.distanceMin =   0.0
        self.experimentparams.pre.trigger.distanceMax = 8.0
        self.experimentparams.pre.trigger.angleMin = 45.0 * N.pi / 180.0
        self.experimentparams.pre.trigger.angleMax =135.0 * N.pi / 180.0
        self.experimentparams.pre.trigger.angleTest = 'inclusive'
        self.experimentparams.pre.trigger.angleTestBilateral = False
        self.experimentparams.pre.trigger.timeHold = 5.0
        self.experimentparams.pre.trigger.timeout = -1
        
        self.experimentparams.pre.wait2 = 0.0
        
        
        # .robot, .lasergalvos, .ledpanels, and .post.trigger all run concurrently.
        # The first one to finish preempts the others.
        self.experimentparams.trial.robot.enabled = True
        self.experimentparams.trial.robot.move.mode = 'pattern'                       # 'relative' or 'pattern'.  Move relative to the given frame, or move in a preset pattern.
        self.experimentparams.trial.robot.move.pattern.frameidPosition = 'Arena'
        self.experimentparams.trial.robot.move.pattern.frameidAngle = 'Arena'
        self.experimentparams.trial.robot.move.pattern.shape = 'constant'               # 'constant' or 'circle' or 'square' or 'flylogo' or 'spiral' or 'grid'
        self.experimentparams.trial.robot.move.pattern.hzPattern = 1/10               # Patterns per second.0
        self.experimentparams.trial.robot.move.pattern.hzPoint = 20                   # The update rate for the actuator.
        self.experimentparams.trial.robot.move.pattern.count = -1
        self.experimentparams.trial.robot.move.pattern.size.x = 0
        self.experimentparams.trial.robot.move.pattern.size.y = -30
        self.experimentparams.trial.robot.move.pattern.direction = 0                  # Step forward (+1) or reverse (-1) through the pattern points.  0 means choose at random, +1 or -1.

        self.experimentparams.trial.lasergalvos.enabled = False
        
        self.experimentparams.trial.ledpanels.enabled = False
        self.experimentparams.trial.ledpanels.command = 'fixed'  # 'fixed', 'trackposition' (panel position follows fly position), or 'trackview' (panel position follows fly's viewpoint). 
        self.experimentparams.trial.ledpanels.idPattern = 1
        self.experimentparams.trial.ledpanels.frame_id = 'Fly1Forecast'
        self.experimentparams.trial.ledpanels.statefilterHi = ''
        self.experimentparams.trial.ledpanels.statefilterLo = ''
        self.experimentparams.trial.ledpanels.statefilterCriteria = ''

        self.experimentparams.post.trigger.enabled = True
        self.experimentparams.post.trigger.frameidParent = 'Arena'
        self.experimentparams.post.trigger.frameidChild = 'Fly01'
        self.experimentparams.post.trigger.speedAbsParentMin =  999.0
        self.experimentparams.post.trigger.speedAbsParentMax = 111.0 # never
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
        self.experimentparams.post.trigger.timeout = 600.0 
        
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
    experiment = Experiment()
    experiment.Run()
        

        
