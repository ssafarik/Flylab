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



#######################################################################################################
class Experiment():
    def __init__(self):
        rospy.init_node('Experiment')
        
        # Fill out the data structure that defines the experiment.
        self.experimentparams = ExperimentParamsChoicesRequest()
        
        self.experimentparams.experiment.description = 'Maze Learning With Food'
        self.experimentparams.experiment.maxTrials = -1
        self.experimentparams.experiment.timeout = -1
        
        self.experimentparams.save.filenamebase = 'maze'
        self.experimentparams.save.csv = True
        self.experimentparams.save.bag = False
        self.experimentparams.save.mov = False
        self.experimentparams.save.fmf = False
        self.experimentparams.save.imagetopic_list = ['camera/image_rect']
        self.experimentparams.save.onlyWhileTriggered = False
        
        self.experimentparams.robotspec.nRobots = 0
        self.experimentparams.robotspec.width = 1.5875
        self.experimentparams.robotspec.height = 1.5875
        self.experimentparams.robotspec.isPresent = True                            # Set this to False if you remove the robot, but still want the actuation.
        self.experimentparams.robotspec.description = 'Black oxide magnet'

        self.experimentparams.flyspec.nFlies = 1
        self.experimentparams.flyspec.description = 'unspecified'
        
        self.experimentparams.tracking.exclusionzones.enabled = True
        self.experimentparams.tracking.exclusionzones.point_list = [Point(x=-12.0, y=7.0), 
                                                                    Point(x=-12.0, y=-7.0), 
                                                                    Point(x=-25.0, y=0.0)]
        self.experimentparams.tracking.exclusionzones.radius_list = [4.0, 
                                                                     4.0, 
                                                                     15.0]
        
        self.experimentparams.home.enabled = False
        self.experimentparams.home.x = 0.0
        self.experimentparams.home.y = 0.0
        self.experimentparams.home.speed = 20
        self.experimentparams.home.tolerance = 2

        self.experimentparams.pre.robot.enabled = False
        self.experimentparams.pre.lasergalvos.enabled = False
        self.experimentparams.pre.ledpanels.enabled = False
        self.experimentparams.pre.wait1 = 1.0
        self.experimentparams.pre.trigger.enabled = False
        self.experimentparams.pre.trigger.frameidParent = 'Fly01'
        self.experimentparams.pre.trigger.frameidChild = 'Robot'
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
        self.experimentparams.trial.robot.enabled                           = False
        self.experimentparams.trial.robot.move.mode                         = 'relative'
        self.experimentparams.trial.robot.move.relative.tracking            = [True]
        self.experimentparams.trial.robot.move.relative.frameidOrigin       = ['Arena']
        self.experimentparams.trial.robot.move.relative.distance            = [50]  # mm
        self.experimentparams.trial.robot.move.relative.angleOffset         = [np.pi/2.0]   # Angular offset to target point in given frame (radians).
        self.experimentparams.trial.robot.move.relative.angleVelocity       = [0]  # 2pi*freq # Angular velocity of target point in given frame (radians per sec).
        self.experimentparams.trial.robot.move.relative.angleOscMag         = [0]  # Oscillatory addition to angular velocity.
        self.experimentparams.trial.robot.move.relative.angleOscFreq        = [0]  # Hz of the added oscillation.
        self.experimentparams.trial.robot.move.relative.speedMax            = [50]  # Maximum allowed speed of travel (mm/sec).
        self.experimentparams.trial.robot.move.relative.tolerance           = [2]  # mm
        self.experimentparams.trial.robot.move.relative.typeAngleOffset     = ['constant']  # 'constant' or 'random' (on range [0,2pi]) or 'current' (use current angle from frameidOrigin to robot)
        self.experimentparams.trial.robot.move.relative.typeAngleVelocity   = ['constant']  # 'constant' or 'random' (on range [0,angleVelocity])
        self.experimentparams.trial.robot.move.relative.typeAngleOscMag     = ['constant']  # 'constant' or 'random' (on range [0,angleOscMag])
        self.experimentparams.trial.robot.move.relative.typeAngleOscFreq    = ['constant']  # 'constant' or 'random' (on range [0,angleOscFreq])
        self.experimentparams.trial.robot.move.relative.typeSpeedMax        = ['constant']  # 'constant' or 'random' (on range [0,speedMax])


        self.experimentparams.trial.lasergalvos.enabled = False
        
        self.experimentparams.trial.ledpanels.enabled = False
        self.experimentparams.trial.ledpanels.command = ['fixed']  # 'fixed', 'trackposition' (panel position follows fly position), or 'trackview' (panel position follows fly's viewpoint). 
        self.experimentparams.trial.ledpanels.idPattern = [1]
        self.experimentparams.trial.ledpanels.frame_id = ['Fly01Forecast']
        self.experimentparams.trial.ledpanels.statefilterHi = ['']
        self.experimentparams.trial.ledpanels.statefilterLo = ['']
        self.experimentparams.trial.ledpanels.statefilterCriteria = ['']

        self.experimentparams.post.trigger.enabled = True
        self.experimentparams.post.trigger.frameidParent = 'Fly01'
        self.experimentparams.post.trigger.frameidChild = 'Robot'
        self.experimentparams.post.trigger.speedAbsParentMin = 999.0
        self.experimentparams.post.trigger.speedAbsParentMax = 999.0
        self.experimentparams.post.trigger.speedAbsChildMin  =   0.0
        self.experimentparams.post.trigger.speedAbsChildMax  = 999.0
        self.experimentparams.post.trigger.speedRelMin       =   0.0
        self.experimentparams.post.trigger.speedRelMax       = 999.0
        self.experimentparams.post.trigger.distanceMin = 999.0
        self.experimentparams.post.trigger.distanceMax =   0.0
        self.experimentparams.post.trigger.angleMin =  0.0 * np.pi / 180.0
        self.experimentparams.post.trigger.angleMax =180.0 * np.pi / 180.0
        self.experimentparams.post.trigger.angleTest = 'inclusive'
        self.experimentparams.post.trigger.angleTestBilateral = True
        self.experimentparams.post.trigger.timeHold = 0.0
        self.experimentparams.post.trigger.timeout = 600

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
    try:
        experiment = Experiment()
        experiment.Run()
        
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')

        
