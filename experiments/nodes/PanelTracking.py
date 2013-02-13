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
        
        self.pubLEDPanels = rospy.Publisher('LEDPanels/command', MsgPanelsCommand, latch=True)
        self.xPanelPattern = 0
        self.yPanelPattern = 0
        
        
        # Fill out the data structure that defines the experiment.
        self.experimentparams = ExperimentParamsRequest()
        
        self.experimentparams.experiment.description = "Rotate Panels to Track a Fly"
        self.experimentparams.experiment.maxTrials = -1
        self.experimentparams.experiment.trial = 1
        
        self.experimentparams.save.filenamebase = "paneltracking"
        self.experimentparams.save.arenastate = True
        self.experimentparams.save.video = False
        self.experimentparams.save.bag = False
        self.experimentparams.save.onlyWhileTriggered = False # Saves always.

        self.experimentparams.tracking.exclusionzones.enabled = False
        self.experimentparams.tracking.exclusionzones.point_list = [Point(x=52.3, y=-51.0)]
        self.experimentparams.tracking.exclusionzones.radius_list = [7.0]
        
        self.experimentparams.pre.robot.enabled = False
        self.experimentparams.pre.lasertrack.enabled = False
        self.experimentparams.pre.ledpanels.enabled = False
        self.experimentparams.pre.wait1 = 0.5
        self.experimentparams.pre.trigger.enabled = False
        self.experimentparams.pre.trigger.frameidParent = 'Arena'
        self.experimentparams.pre.trigger.frameidChild = 'Fly1'
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
        self.experimentparams.pre.trigger.angleTestBilateral = False
        self.experimentparams.pre.trigger.timeHold = 0.0
        self.experimentparams.pre.trigger.timeout = -1
        self.experimentparams.pre.wait2 = 0.0
        
        
        # .robot, .lasertrack, .ledpanels, and .post.trigger all run concurrently.
        # The first one to finish preempts the others.
        self.experimentparams.trial.robot.enabled = False
        
        
        self.experimentparams.trial.lasertrack.enabled = False
        self.experimentparams.trial.lasertrack.pattern_list = []
        self.experimentparams.trial.lasertrack.statefilterHi_list = []
        self.experimentparams.trial.lasertrack.statefilterLo_list = []
        self.experimentparams.trial.lasertrack.statefilterCriteria_list = []
        for iFly in range(rospy.get_param('nFlies', 0)):#range(3):#
            self.experimentparams.trial.lasertrack.pattern_list.append(MsgPattern(shape      = 'grid',
                                                                            frameid   = 'Fly%dForecast' % (iFly+1),
                                                                            hzPattern  = 40.0,
                                                                            hzPoint    = 1000.0,
                                                                            count      = 1,
                                                                            size       = Point(x=2,
                                                                                               y=2),
                                                                            preempt    = False,
                                                                            param      = 3, # Peano curve level.
                                                                            direction  = 1),
                                                                 )
            #self.experimentparams.trial.lasertrack.statefilterHi_list.append("{'speed':5.0}")
            #self.experimentparams.trial.lasertrack.statefilterLo_list.append("{'speed':0.0}")
            #self.experimentparams.trial.lasertrack.statefilterHi_list.append("{'velocity':{'linear':{'x':+6,'y':+6}}}")
            #self.experimentparams.trial.lasertrack.statefilterLo_list.append("{'velocity':{'linear':{'x':-6,'y':-6}}}")
            #self.experimentparams.trial.lasertrack.statefilterHi_list.append("{'velocity':{'angular':{'z':999}}}")
            #self.experimentparams.trial.lasertrack.statefilterLo_list.append("{'velocity':{'angular':{'z':0.5}}}")
            self.experimentparams.trial.lasertrack.statefilterHi_list.append("{'pose':{'position':{'x':-25, 'y':-25}}}")  # This is the cool zone.
            self.experimentparams.trial.lasertrack.statefilterLo_list.append("{'pose':{'position':{'x':-50, 'y':-50}}}")
            self.experimentparams.trial.lasertrack.statefilterCriteria_list.append("exclusive")
        
        self.experimentparams.trial.ledpanels.enabled = True
        self.experimentparams.trial.ledpanels.command = 'fixed'  # 'fixed', 'trackposition' (panel position follows fly position), or 'trackview' (panel position follows fly's viewpoint). 
        self.experimentparams.trial.ledpanels.idPattern = 2
        self.experimentparams.trial.ledpanels.origin.x = 10
        self.experimentparams.trial.ledpanels.origin.y = 0
        self.experimentparams.trial.ledpanels.frame_id = 'Fly1Forecast'
        self.experimentparams.trial.ledpanels.statefilterHi = ''
        self.experimentparams.trial.ledpanels.statefilterLo = ''
        self.experimentparams.trial.ledpanels.statefilterCriteria = ''

        
        self.experimentparams.post.trigger.enabled = True
        self.experimentparams.post.trigger.frameidParent = 'Arena'
        self.experimentparams.post.trigger.frameidChild = 'Fly1'
        self.experimentparams.post.trigger.speedAbsParentMin =   0.0
        self.experimentparams.post.trigger.speedAbsParentMax = 999.0
        self.experimentparams.post.trigger.speedAbsChildMin  =   0.0
        self.experimentparams.post.trigger.speedAbsChildMax  = 999.0
        self.experimentparams.post.trigger.speedRelMin       =   0.0
        self.experimentparams.post.trigger.speedRelMax       = 999.0
        self.experimentparams.post.trigger.distanceMin = 999.0
        self.experimentparams.post.trigger.distanceMax = 111.0                  # i.e. NEVER
        self.experimentparams.post.trigger.angleMin =  0.0000 * N.pi / 180.0
        self.experimentparams.post.trigger.angleMax =359.9999 * N.pi / 180.0
        self.experimentparams.post.trigger.angleTest = 'inclusive'
        self.experimentparams.post.trigger.angleTestBilateral = False
        self.experimentparams.post.trigger.timeHold = 0.0
        self.experimentparams.post.trigger.timeout = 300      # 5 minute trials.

        self.experimentparams.post.wait = 0.0
        
        self.experimentlib = ExperimentLib.ExperimentLib(self.experimentparams, 
                                                         newexperiment_callback = self.Newexperiment_callback, 
                                                         newtrial_callback = self.Newtrial_callback, 
                                                         endtrial_callback = self.Endtrial_callback)



    def Run(self):
        self.experimentlib.Run()
        

    # This function gets called at the start of a new experiment.  Use this to do any one-time initialization of hardware, etc.
    def Newexperiment_callback(self, userdata):
#        # Set the LED pattern to the origin.
#        msgPanelsCommand = MsgPanelsCommand(command='stop')
#        self.pubLEDPanels.publish (msgPanelsCommand)
#
#        msgPanelsCommand = MsgPanelsCommand(command='set_pattern_id', arg1=1) # Assumes preprogrammed to the three-section pattern from Reiser paper. 
#        self.pubLEDPanels.publish (msgPanelsCommand)
#
#        msgPanelsCommand = MsgPanelsCommand(command='set_position', arg1=0, arg2=0)
#        self.pubLEDPanels.publish (msgPanelsCommand)
        return 'success'

    
    # This function gets called at the start of a new trial.  Use this to alter the experiment params from trial to trial.
    def Newtrial_callback(self, userdata):
        userdata.experimentparamsOut = userdata.experimentparamsIn
        return 'success'


    # This function gets called at the end of a new trial.  Use this to alter the experiment params from trial to trial.
    def Endtrial_callback(self, userdata):
        
        # Create a rotation matrix R for an angle of +90 or -90, chosen at random.
        plusorminusone = (N.random.random()>=0.5) * 2 - 1       # -1 means +90 (CCW), +1 means -90 (CW).
        R = N.array([[0,plusorminusone],[-plusorminusone,0]])
        
        
        # Rotate the pattern on the LED panels.
        self.xPanelPattern += plusorminusone * 48 # 48 = 1/4 of the pixel width of the 24 panels.  192 pixels all the way around.  24 panels * 8 pixels per panel = 192.
        self.xPanelPattern %= 192
        self.yPanelPattern = 0
        msgPanelsCommand = MsgPanelsCommand(command='set_position', arg1=self.xPanelPattern, arg2=self.yPanelPattern)
        self.pubLEDPanels.publish (msgPanelsCommand)
         
        
        # Rotate the points in the statefilter strings by R.  (Convert string to dict, rotate, then convert dict back to string).
        statefilterHi_list = []
        statefilterLo_list = []
        for iFilter in range(len(userdata.experimentparamsIn.lasertrack.statefilterLo_list)):
            # Convert strings to dicts.
            statefilterHi_dict = eval(userdata.experimentparamsIn.lasertrack.statefilterHi_list[iFilter])
            statefilterLo_dict = eval(userdata.experimentparamsIn.lasertrack.statefilterLo_list[iFilter])

            # Rotate
            if 'pose' in statefilterLo_dict:
                if 'position' in statefilterLo_dict['pose']:
                    x = statefilterHi_dict['pose']['position']['x']
                    y = statefilterHi_dict['pose']['position']['y']
                    pt = N.array([x,y])
                    [xRotA,yRotA] = N.dot(R,pt)

                    x = statefilterLo_dict['pose']['position']['x']
                    y = statefilterLo_dict['pose']['position']['y']
                    pt = N.array([x,y])
                    [xRotB,yRotB] = N.dot(R,pt)

                    # Rotated hi/lo are now not necessarily in the same order.
                    statefilterHi_dict['pose']['position']['x'] = max(xRotA,xRotB)
                    statefilterHi_dict['pose']['position']['y'] = max(yRotA,yRotB)
                    statefilterLo_dict['pose']['position']['x'] = min(xRotA,xRotB)
                    statefilterLo_dict['pose']['position']['y'] = min(yRotA,yRotB)

            statefilterLo_list.append(str(statefilterLo_dict))
            statefilterHi_list.append(str(statefilterHi_dict))


        # Save the results into the output.            
        experimentparamsOut = userdata.experimentparamsIn
        experimentparamsOut.lasertrack.statefilterLo_list = statefilterLo_list
        experimentparamsOut.lasertrack.statefilterHi_list = statefilterHi_list
        userdata.experimentparamsOut = experimentparamsOut
        
        return 'success'


if __name__ == '__main__':
    experiment = Experiment()
    experiment.Run()
        

        
