#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('experiments')
import rospy
import numpy as np
import ExperimentLib
from geometry_msgs.msg import Point, Twist
from experiment_srvs.srv import Trigger, ExperimentParamsChoices, ExperimentParamsChoicesRequest
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
        
        self.experimentparams.experiment.description = 'Measure the temperature via the laser.'
        self.experimentparams.experiment.maxTrials = -1
        self.experimentparams.experiment.timeout = 600
        
        self.experimentparams.save.filenamebase = 'lasertemp'
        self.experimentparams.save.csv = False
        self.experimentparams.save.bag = False
        self.experimentparams.save.mov = False
        self.experimentparams.save.imagetopic_list = ['camera/image_rect']
        self.experimentparams.save.onlyWhileTriggered = True
        
        self.experimentparams.robotspec.nRobots = 0
        self.experimentparams.robotspec.width = 1.5875
        self.experimentparams.robotspec.height = 1.5875
        self.experimentparams.robotspec.isPresent = True                            # Set this to False if you remove the robot, but still want the actuation.
        self.experimentparams.robotspec.description = 'Black oxide magnet'

        self.experimentparams.flyspec.nFlies = 1
        self.experimentparams.flyspec.description = 'unspecified'
        
        self.experimentparams.tracking.exclusionzones.enabled = True
        self.experimentparams.tracking.exclusionzones.point_list = [Point(x=28.0, y=0.0)]
        self.experimentparams.tracking.exclusionzones.radius_list = [20.0]
        
        self.experimentparams.home.enabled = False
        self.experimentparams.home.x = rospy.get_param('motorarm/L1', 999)
        self.experimentparams.home.y = 0.0
        self.experimentparams.home.speed = 20
        self.experimentparams.home.tolerance = 4

        self.experimentparams.pre.robot.enabled = False
        self.experimentparams.pre.lasergalvos.enabled = False
        self.experimentparams.pre.ledpanels.enabled = False
        self.experimentparams.pre.wait1 = 0
        self.experimentparams.pre.trigger.enabled = False
        self.experimentparams.pre.wait2 = 0.0
        

        # .robot, .lasergalvos, .ledpanels, and .post.trigger all run concurrently.
        # The first one to finish preempts the others.
        self.experimentparams.trial.robot.enabled = False
        
        
        self.experimentparams.trial.lasergalvos.enabled = True
        self.experimentparams.trial.lasergalvos.statefilterLo_list = []
        self.experimentparams.trial.lasergalvos.statefilterHi_list = []
        self.experimentparams.trial.lasergalvos.statefilterCriteria_list = []
        iFly = 1
        self.experimentparams.trial.lasergalvos.pattern_list = []
        self.experimentparams.trial.lasergalvos.pattern_list.append(MsgPattern(
                                                                    frameidPosition   = 'Fly%02d' % iFly,
                                                                    frameidAngle   = 'Fly%02d' % iFly,
                                                                    shape      = 'grid',
                                                                    hzPattern  = 40.0,
                                                                    hzPoint    = 1000.0,
                                                                    count      = 1,
                                                                    points     =[],
                                                                    size       = Point(x=3,
                                                                                       y=3),
                                                                    restart    = False,
                                                                    param      = 3, # Peano curve level.
                                                                    direction  = 1),
                                                                   )

        
        self.experimentparams.trial.ledpanels.enabled = False

        self.experimentparams.post.trigger.enabled = True
        self.experimentparams.post.trigger.frameidParent = 'Fly01'
        self.experimentparams.post.trigger.frameidChild = 'Robot'
        self.experimentparams.post.trigger.speedAbsParentMin =   0.0
        self.experimentparams.post.trigger.speedAbsParentMax = 999.0
        self.experimentparams.post.trigger.speedAbsChildMin  =   0.0
        self.experimentparams.post.trigger.speedAbsChildMax  = 999.0
        self.experimentparams.post.trigger.speedRelMin       =   0.0
        self.experimentparams.post.trigger.speedRelMax       = 999.0
        self.experimentparams.post.trigger.distanceMin = 999.0
        self.experimentparams.post.trigger.distanceMax = 111.0 # Never
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
    #try:
        experiment = Experiment()
        experiment.Run()
        
    #except:
        rospy.loginfo('Shutting down')

        
