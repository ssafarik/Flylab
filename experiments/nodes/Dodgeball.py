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
class Experiment():
    def __init__(self):
        rospy.init_node('Experiment')
        
        # Fill out the data structure that defines the experiment.
        self.experimentparams = ExperimentParamsRequest()
        
        self.experimentparams.experiment.description = "Dodgeball"
        self.experimentparams.experiment.maxTrials = -1
        self.experimentparams.experiment.trial = 1
        
        self.experimentparams.save.filenamebase = "dodgeball"
        self.experimentparams.save.arenastate = True
        self.experimentparams.save.video = True
        self.experimentparams.save.bag = False
        self.experimentparams.save.onlyWhileTriggered = False
        
        self.experimentparams.tracking.exclusionzones.enabled = True
        self.experimentparams.tracking.exclusionzones.point_list = [Point(x=50.0, y=68.0)]
        self.experimentparams.tracking.exclusionzones.radius_list = [3.0]
        
        self.experimentparams.waitEntry1 = 0.0
        
        self.experimentparams.triggerEntry.enabled = True
        self.experimentparams.triggerEntry.frameidParent = 'Fly1'
        self.experimentparams.triggerEntry.frameidChild = 'Robot'
        self.experimentparams.triggerEntry.speedAbsParentMin =   5.0        # Absolute speed of the parent in fixed frame.
        self.experimentparams.triggerEntry.speedAbsParentMax =  60.0
        self.experimentparams.triggerEntry.speedAbsChildMin  =   0.0        # Absolute speed of the child in fixed frame.
        self.experimentparams.triggerEntry.speedAbsChildMax  = 999.0
        self.experimentparams.triggerEntry.speedRelMin       =   0.0        # Relative speed of child to parent.
        self.experimentparams.triggerEntry.speedRelMax       = 999.0
        self.experimentparams.triggerEntry.distanceMin = 10.0               # Distance between child and parent frames.
        self.experimentparams.triggerEntry.distanceMax = 35.0
        self.experimentparams.triggerEntry.angleMin = 00.0 * N.pi / 180.0   # Angle of the child frame from the perspective of the parent frame.
        self.experimentparams.triggerEntry.angleMax =180.0 * N.pi / 180.0
        self.experimentparams.triggerEntry.angleTest = 'inclusive'          # 'inclusive' or 'exclusive' of the given angle range.
        self.experimentparams.triggerEntry.angleTestBilateral = True        # True=bilateral, False=unilateral.
        self.experimentparams.triggerEntry.timeHold = 0.1                   # How long the conditions must be continually met before the trigger happens.
        self.experimentparams.triggerEntry.timeout = -1
        
        self.experimentparams.waitEntry2 = 0.0
        

        # .robot, .lasertrack, .ledpanels, and .triggerExit all run concurrently.
        # The first one to finish preempts the others.
        self.experimentparams.robot.enabled = True
        self.experimentparams.robot.move.mode = 'relative'                        # 'relative' or 'pattern'.  Move relative to the given frame, or move in a preset pattern.
        self.experimentparams.robot.move.relative.tracking = True                # True=update the target point continually.  False=the target point is set at the trigger time. 
        self.experimentparams.robot.move.relative.frameidOriginPosition = "Fly1"
        self.experimentparams.robot.move.relative.frameidOriginAngle = "Fly1"
        self.experimentparams.robot.move.relative.distance = 3                    # Distance to the target point from the origin frame's position.
        self.experimentparams.robot.move.relative.angle = 0                       # Angle to the target point from the origin frame's x-axis.
        self.experimentparams.robot.move.relative.angleType = 'random'          # 'constant' or 'random'.  Use given angle always, or choose random angle once per move.
        self.experimentparams.robot.move.relative.speed = 20                      # Speed at which to move the robot toward the target point. 
        self.experimentparams.robot.move.relative.speedType = 'constant'          # 'constant' or 'random'.  Use the given value, or a random fraction of it. 
        self.experimentparams.robot.move.relative.tolerance = 2                   # When robot-to-target distance is within this tolerance, then the move is over.
        self.experimentparams.robot.move.timeout = -1                             # When this duration has passed, then the move is over.
        self.experimentparams.robot.home.enabled = True
        self.experimentparams.robot.home.x = 0.0
        self.experimentparams.robot.home.y = 0.0
        self.experimentparams.robot.home.speed = 20
        self.experimentparams.robot.home.timeout = -1
        self.experimentparams.robot.home.tolerance = 2
        
        
        self.experimentparams.lasertrack.enabled = False
        
        self.experimentparams.ledpanels.enabled = True
        self.experimentparams.ledpanels.command = 'fixed'  # 'fixed', 'trackposition' (panel position follows fly position), or 'trackview' (panel position follows fly's viewpoint). 
        self.experimentparams.ledpanels.idPattern = 1 
        self.experimentparams.ledpanels.origin.x = 0 
        self.experimentparams.ledpanels.origin.y = 0 
        self.experimentparams.ledpanels.frame_id = 'Fly1Forecast'
        self.experimentparams.ledpanels.statefilterHi = ''
        self.experimentparams.ledpanels.statefilterLo = ''
        self.experimentparams.ledpanels.statefilterCriteria = ''
        self.experimentparams.ledpanels.timeout = -1

        self.experimentparams.triggerExit.enabled = True
        self.experimentparams.triggerExit.frameidParent = 'Fly1'
        self.experimentparams.triggerExit.frameidChild = 'Robot'
        self.experimentparams.triggerExit.speedAbsParentMin = 999.0
        self.experimentparams.triggerExit.speedAbsParentMax = 111.0 # i.e. never
        self.experimentparams.triggerExit.speedAbsChildMin  =   0.0
        self.experimentparams.triggerExit.speedAbsChildMax  = 999.0
        self.experimentparams.triggerExit.speedRelMin       =   0.0
        self.experimentparams.triggerExit.speedRelMax       = 999.0
        self.experimentparams.triggerExit.distanceMin = 0.0              
        self.experimentparams.triggerExit.distanceMax = 999.0
        self.experimentparams.triggerExit.angleMin =  0.0 * N.pi / 180.0   
        self.experimentparams.triggerExit.angleMax =180.0 * N.pi / 180.0
        self.experimentparams.triggerExit.angleTest = 'inclusive'
        self.experimentparams.triggerExit.angleTestBilateral = True
        self.experimentparams.triggerExit.timeHold = 1.0
        self.experimentparams.triggerExit.timeout = 2

        self.experimentparams.waitExit = 0.0
        
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
    def Endtrial_callback(self, userdata):
        userdata.experimentparamsOut = userdata.experimentparamsIn
        return 'success'



if __name__ == '__main__':
    try:
        experiment = Experiment()
        experiment.Run()
        
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

        
