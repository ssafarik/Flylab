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
from patterngen.msg import MsgPattern
from LEDPanels.msg import MsgPanelsCommand




#######################################################################################################
class Experiment():
    def __init__(self):
        rospy.init_node('Experiment')
        
        self.pubLEDPanels = rospy.Publisher('LEDPanels/command', MsgPanelsCommand, latch=True)
        self.xPanelPattern = 0
        self.yPanelPattern = 0
        
        
        # Fill out the data structure that defines the experiment.
        self.experimentparams = ExperimentParamsRequest()
        
        self.experimentparams.experiment.description = "Visual Place Learning paper by Reiser"
        self.experimentparams.experiment.maxTrials = -1
        self.experimentparams.experiment.trial = 1
        
        self.experimentparams.save.filenamebase = "vpl"
        self.experimentparams.save.arenastate = True
        self.experimentparams.save.video = False
        self.experimentparams.save.bag = False
        self.experimentparams.save.onlyWhileTriggered = True # Saves always.

        self.experimentparams.tracking.exclusionzone.enabled = False
        self.experimentparams.tracking.exclusionzone.point_list = [Point(x=52.3, y=-51.0)]
        self.experimentparams.tracking.exclusionzone.radius_list = [7.0]
        
        self.experimentparams.home.enabled = False
        
        self.experimentparams.waitEntry = 0.0
        
        self.experimentparams.triggerEntry.enabled = False
        self.experimentparams.triggerEntry.frameidParent = 'Plate'
        self.experimentparams.triggerEntry.frameidChild = 'Fly1'
        self.experimentparams.triggerEntry.speedAbsParentMin =   0.0
        self.experimentparams.triggerEntry.speedAbsParentMax = 999.0
        self.experimentparams.triggerEntry.speedAbsChildMin  =   0.0
        self.experimentparams.triggerEntry.speedAbsChildMax  = 999.0
        self.experimentparams.triggerEntry.speedRelMin       =   0.0
        self.experimentparams.triggerEntry.speedRelMax       = 999.0
        self.experimentparams.triggerEntry.distanceMin =   0.0
        self.experimentparams.triggerEntry.distanceMax = 999.0
        self.experimentparams.triggerEntry.angleMin =  0.0 * N.pi / 180.0
        self.experimentparams.triggerEntry.angleMax =180.0 * N.pi / 180.0
        self.experimentparams.triggerEntry.angleTest = 'inclusive'
        self.experimentparams.triggerEntry.angleTestBilateral = False
        self.experimentparams.triggerEntry.timeHold = 0.0
        self.experimentparams.triggerEntry.timeout = -1
        
        
        # .move, .lasertrack, and .triggerExit all run concurrently.
        # The first one to finish preempts the others.
        self.experimentparams.move.enabled = False
        
        
        self.experimentparams.lasertrack.enabled = True
        self.experimentparams.lasertrack.pattern_list = []
        self.experimentparams.lasertrack.statefilterHi_list = []
        self.experimentparams.lasertrack.statefilterLo_list = []
        self.experimentparams.lasertrack.statefilterCriteria_list = []
        for iFly in range(rospy.get_param('nFlies', 0)):#range(3):#
            self.experimentparams.lasertrack.pattern_list.append(MsgPattern(mode       = 'byshape',
                                                                            shape      = 'grid',
                                                                            frame_id   = 'Fly%dForecast' % (iFly+1),
                                                                            hzPattern  = 40.0,
                                                                            hzPoint    = 1000.0,
                                                                            count      = 1,
                                                                            size       = Point(x=2,
                                                                                               y=2),
                                                                            preempt    = False,
                                                                            param      = 3), # Peano curve level.
                                                                 )
            #self.experimentparams.lasertrack.statefilterHi_list.append("{'speed':5.0}")
            #self.experimentparams.lasertrack.statefilterLo_list.append("{'speed':0.0}")
            #self.experimentparams.lasertrack.statefilterHi_list.append("{'velocity':{'linear':{'x':+6,'y':+6}}}")
            #self.experimentparams.lasertrack.statefilterLo_list.append("{'velocity':{'linear':{'x':-6,'y':-6}}}")
            #self.experimentparams.lasertrack.statefilterHi_list.append("{'velocity':{'angular':{'z':999}}}")
            #self.experimentparams.lasertrack.statefilterLo_list.append("{'velocity':{'angular':{'z':0.5}}}")
            self.experimentparams.lasertrack.statefilterHi_list.append("{'pose':{'position':{'x':-25, 'y':-25}}}")  # This is the cool zone.
            self.experimentparams.lasertrack.statefilterLo_list.append("{'pose':{'position':{'x':-50, 'y':-50}}}")
            self.experimentparams.lasertrack.statefilterCriteria_list.append("exclusive")
        self.experimentparams.lasertrack.timeout = -1
        
        self.experimentparams.triggerExit.enabled = True
        self.experimentparams.triggerExit.frameidParent = 'Plate'
        self.experimentparams.triggerExit.frameidChild = 'Fly1'
        self.experimentparams.triggerExit.speedAbsParentMin =   0.0
        self.experimentparams.triggerExit.speedAbsParentMax = 999.0
        self.experimentparams.triggerExit.speedAbsChildMin  =   0.0
        self.experimentparams.triggerExit.speedAbsChildMax  = 999.0
        self.experimentparams.triggerExit.speedRelMin       =   0.0
        self.experimentparams.triggerExit.speedRelMax       = 999.0
        self.experimentparams.triggerExit.distanceMin = 999.0
        self.experimentparams.triggerExit.distanceMax = 111.0 # i.e. never
        self.experimentparams.triggerExit.angleMin =  0.0000 * N.pi / 180.0
        self.experimentparams.triggerExit.angleMax =359.9999 * N.pi / 180.0
        self.experimentparams.triggerExit.angleTest = 'inclusive'
        self.experimentparams.triggerExit.angleTestBilateral = False
        self.experimentparams.triggerExit.timeHold = 0.0
        self.experimentparams.triggerExit.timeout = 300      # 5 minute trials.

        self.experimentparams.waitExit = 0.0
        
        self.experimentlib = ExperimentLib.ExperimentLib(self.experimentparams, 
                                                         newexperiment_callback = self.Newexperiment_callback, 
                                                         newtrial_callback = self.Newtrial_callback, 
                                                         endtrial_callback = self.Endtrial_callback)



    def Run(self):
        self.experimentlib.Run()
        

    # This function gets called at the start of a new experiment.  Use this to do any one-time initialization of hardware, etc.
    def Newexperiment_callback(self, userdata):
        # Set the LED pattern to the origin.
        msgPanelsCommand = MsgPanelsCommand(command='stop')
        self.pubLEDPanels.publish (msgPanelsCommand)

        msgPanelsCommand = MsgPanelsCommand(command='set_pattern_id', arg1=1) # Assumes preprogrammed to the three-section pattern from Reiser paper. 
        self.pubLEDPanels.publish (msgPanelsCommand)

        msgPanelsCommand = MsgPanelsCommand(command='set_position', arg1=0, arg2=0)
        self.pubLEDPanels.publish (msgPanelsCommand)
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
        

        
