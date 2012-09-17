#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('experiments')
import rospy
import numpy as N
from geometry_msgs.msg import Point
import ExperimentLib
from experiments.srv import *



#######################################################################################################
class ExperimentDodgeball():
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
        self.experimentparams.save.onlyWhileTriggered = True
        
        self.experimentparams.tracking.exclusionzone.enabled = True
        self.experimentparams.tracking.exclusionzone.point_list = [Point(x=50.0, y=68.0)]
        self.experimentparams.tracking.exclusionzone.radius_list = [3.0]
        
        self.experimentparams.home.enabled = True
        self.experimentparams.home.x = 0.0
        self.experimentparams.home.y = 0.0
        self.experimentparams.home.speed = 20
        self.experimentparams.home.timeout = -1
        self.experimentparams.home.tolerance = 2
        
        self.experimentparams.waitEntry = 0.0
        
        self.experimentparams.triggerEntry.enabled = True
        self.experimentparams.triggerEntry.frameidParent = 'Fly1'
        self.experimentparams.triggerEntry.frameidChild = 'Robot'
        self.experimentparams.triggerEntry.speedAbsParentMin =   3.0        # Absolute speed of the parent in fixed frame.
        self.experimentparams.triggerEntry.speedAbsParentMax =  40.0
        self.experimentparams.triggerEntry.speedAbsChildMin  =   0.0        # Absolute speed of the child in fixed frame.
        self.experimentparams.triggerEntry.speedAbsChildMax  = 999.0
        self.experimentparams.triggerEntry.speedRelMin       =   0.0        # Relative speed of child to parent.
        self.experimentparams.triggerEntry.speedRelMax       = 999.0
        self.experimentparams.triggerEntry.distanceMin = 20.0               # Distance between child and parent frames.
        self.experimentparams.triggerEntry.distanceMax = 50.0
        self.experimentparams.triggerEntry.angleMin = 00.0 * N.pi / 180.0   # Angle of the child frame from the perspective of the parent frame.
        self.experimentparams.triggerEntry.angleMax =180.0 * N.pi / 180.0
        self.experimentparams.triggerEntry.angleTest = 'inclusive'          # 'inclusive' or 'exclusive' of the given angle range.
        self.experimentparams.triggerEntry.angleTestBilateral = True        # True=bilateral, False=unilateral.
        self.experimentparams.triggerEntry.timeHold = 0.5                   # How long the conditions must be continually met before the trigger happens.
        self.experimentparams.triggerEntry.timeout = -1
        

        # .move, .lasertrack, and .triggerExit all run concurrently.
        # The first one to finish preempts the others.
        self.experimentparams.move.enabled = True
        self.experimentparams.move.mode = 'relative'                        # 'relative' or 'pattern'.  Move relative to the given frame, or move in a preset pattern.
        self.experimentparams.move.relative.tracking = True                # True=update the target point continually.  False=the target point is set at the trigger time. 
        self.experimentparams.move.relative.frameidOriginPosition = "Fly1"
        self.experimentparams.move.relative.frameidOriginAngle = "Fly1"
        self.experimentparams.move.relative.distance = 3                    # Distance to the target point from the origin frame's position.
        self.experimentparams.move.relative.angle = 0                       # Angle to the target point from the origin frame's x-axis.
        self.experimentparams.move.relative.angleType = 'random'          # 'constant' or 'random'.  Use given angle always, or choose random angle once per move.
        self.experimentparams.move.relative.speed = 20                      # Speed at which to move the robot toward the target point. 
        self.experimentparams.move.relative.speedType = 'constant'          # 'constant' or 'random'.  Use the given value, or a random fraction of it. 
        self.experimentparams.move.relative.tolerance = 2                   # When robot-to-target distance is within this tolerance, then the move is over.
        self.experimentparams.move.timeout = 10                             # When this duration has passed, then the move is over.
        
        self.experimentparams.lasertrack.enabled = False
        
        self.experimentparams.triggerExit.enabled = False
        self.experimentparams.triggerExit.frameidParent = 'Fly1'
        self.experimentparams.triggerExit.frameidChild = 'Robot'
        self.experimentparams.triggerExit.speedAbsParentMin =   0.0
        self.experimentparams.triggerExit.speedAbsParentMax = 999.0
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
        self.experimentparams.triggerExit.timeout = -1

        self.experimentparams.waitExit = 0.0
        
        self.experiment = ExperimentLib.Experiment(self.experimentparams)


    def Run(self):
        self.experiment.Run()



if __name__ == '__main__':
    try:
        experiment = ExperimentDodgeball()
        experiment.Run()
        
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

        
