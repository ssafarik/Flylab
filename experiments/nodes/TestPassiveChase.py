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



#######################################################################################################
class ExperimentPassiveChase():
    def __init__(self):
        rospy.init_node('Experiment')
        
        # Fill out the data structure that defines the experiment.
        self.experimentparams = ExperimentParamsRequest()
        
        self.experimentparams.experiment.description = "Fly Chases Robot Moving in Circle"
        self.experimentparams.experiment.maxTrials = -1
        self.experimentparams.experiment.timeout = -1
        
        self.experimentparams.save.filenamebase = "testpassivechase"
        self.experimentparams.save.csv = True
        self.experimentparams.save.bag = True
        self.experimentparams.save.mov = False
        self.experimentparams.save.imagetopic_list = ['camera/image_rect']
        self.experimentparams.save.onlyWhileTriggered = True
        
        self.experimentparams.robotspec.nRobots = 1
        self.experimentparams.robotspec.width = 1.5875
        self.experimentparams.robotspec.height = 1.5875
        self.experimentparams.robotspec.isPresent = True                            # Set this to False if you remove the robot, but still want the actuation.
        self.experimentparams.robotspec.description = "Black oxide magnet"

        self.experimentparams.flyspec.nFlies = 1
        self.experimentparams.flyspec.description = "unspecified"
        
        self.experimentparams.tracking.exclusionzones.enabled = False
        self.experimentparams.tracking.exclusionzones.point_list = [Point(x=0.0, y=0.0)]
        self.experimentparams.tracking.exclusionzones.radius_list = [0.0]
        
        self.experimentparams.pre.robot.enabled = False
        self.experimentparams.pre.lasergalvos.enabled = False
        self.experimentparams.pre.ledpanels.enabled = False
        self.experimentparams.pre.wait1 = 0#150.0
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
        self.experimentparams.pre.trigger.angleMin =  0.0 * N.pi / 180.0
        self.experimentparams.pre.trigger.angleMax =180.0 * N.pi / 180.0
        self.experimentparams.pre.trigger.angleTest = 'inclusive'
        self.experimentparams.pre.trigger.angleTestBilateral = True
        self.experimentparams.pre.trigger.timeHold = 0.0
        self.experimentparams.pre.trigger.timeout = -1
        self.experimentparams.pre.wait2 = 0.0
        

        # .robot, .lasergalvos, .ledpanels, and .post.trigger all run concurrently.
        # The first one to finish preempts the others.
        self.experimentparams.trial.robot.enabled = True
        self.experimentparams.trial.robot.move.mode = 'pattern' # 'pattern' or 'relative'
        self.experimentparams.trial.robot.move.pattern.frameidPosition = 'Arena'            # 
        self.experimentparams.trial.robot.move.pattern.frameidAngle = 'Arena'               # 
        self.experimentparams.trial.robot.move.pattern.shape = 'circle' # 'constant' or 'circle' or 'square' or 'flylogo' or 'spiral' or 'ramp'
        self.experimentparams.trial.robot.move.pattern.hzPattern = 1/40
        self.experimentparams.trial.robot.move.pattern.hzPoint = 100
        self.experimentparams.trial.robot.move.pattern.count = -1
        self.experimentparams.trial.robot.move.pattern.size.x = rospy.get_param('motorarm/L1', 999)
        self.experimentparams.trial.robot.move.pattern.size.y = 0
        self.experimentparams.trial.robot.move.pattern.param = 0
        self.experimentparams.trial.robot.move.pattern.direction = 1
        self.experimentparams.trial.robot.home.enabled = True
        self.experimentparams.trial.robot.home.x = rospy.get_param('motorarm/L1', 999)
        self.experimentparams.trial.robot.home.y = 0.0
        self.experimentparams.trial.robot.home.speed = 20
        self.experimentparams.trial.robot.home.tolerance = 2
        
        
        self.experimentparams.trial.lasergalvos.enabled = False

        
        self.experimentparams.trial.ledpanels.enabled = True
        self.experimentparams.trial.ledpanels.command = 'fixed'  # 'fixed', 'trackposition' (panel position follows fly position), or 'trackview' (panel position follows fly's viewpoint). 
        self.experimentparams.trial.ledpanels.idPattern = 1
        self.experimentparams.trial.ledpanels.origin.x = 0 
        self.experimentparams.trial.ledpanels.origin.y = 0 
        self.experimentparams.trial.ledpanels.frame_id = 'Fly01Forecast'
        self.experimentparams.trial.ledpanels.statefilterHi = ''
        self.experimentparams.trial.ledpanels.statefilterLo = ''
        self.experimentparams.trial.ledpanels.statefilterCriteria = ''

        self.experimentparams.post.trigger.enabled = True
        self.experimentparams.post.trigger.frameidParent = 'Fly01'
        self.experimentparams.post.trigger.frameidChild = 'Robot'
        self.experimentparams.post.trigger.speedAbsParentMin = 999.0
        self.experimentparams.post.trigger.speedAbsParentMax = 111.0 # i.e. NEVER
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
        self.experimentparams.post.trigger.timeout = 60

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
        experiment = ExperimentPassiveChase()
        experiment.Run()
        
    #except:
        rospy.loginfo("Shutting down")

        
