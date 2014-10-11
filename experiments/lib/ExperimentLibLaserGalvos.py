#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('experiments')
import rospy
import actionlib
import copy
import numpy as np
import smach
import tf

from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped, Quaternion, Twist, Vector3
from std_msgs.msg import Header, ColorRGBA, String
from flycore.msg import MsgFrameState, TrackingCommand
from galvodirector.msg import MsgGalvoCommand
from tracking.msg import ArenaState
from visualization_msgs.msg import Marker

import ExperimentLibStatefilter


#######################################################################################################
#######################################################################################################
class Reset (smach.State):
    def __init__(self, tfrx=None):
        smach.State.__init__(self, 
                             outcomes=['success','disabled','preempt','aborted'],
                             input_keys=['experimentparamsChoicesIn'])

        self.arenastate = None
        self.rosrate = rospy.Rate(rospy.get_param('experiment/looprate', 50))

        queue_size_arenastate = rospy.get_param('tracking/queue_size_arenastate', 1)
        self.pubGalvoCommand    = rospy.Publisher('galvodirector/command', MsgGalvoCommand, latch=True)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)

        rospy.on_shutdown(self.OnShutdown_callback)
        
        
        # Command messages.
        self.commandExperiment = 'continue'
        self.commandExperiment_list = ['continue','pause_now','pause_after_trial', 'exit_after_trial', 'exit_now']
        self.subCommand = rospy.Subscriber('experiment/command', String, self.CommandExperiment_callback)

        


    def CommandExperiment_callback(self, msgString):
        self.commandExperiment = msgString.data
            
        
    def OnShutdown_callback(self):
        pass
        
        
    def ArenaState_callback(self, arenastate):
        self.arenastate = arenastate



    def execute(self, userdata):
        rospy.loginfo("EL State ResetGalvos()")

        if (userdata.experimentparamsChoicesIn.trial.lasergalvos.enabled):
            commandGalvoBeamsink = MsgGalvoCommand()
            commandGalvoBeamsink.enable_laser = False # False will send it to the beamsink.
            self.pubGalvoCommand.publish(commandGalvoBeamsink)
            rv = 'success'
        
        else:
            rv = 'disabled'
            
        #rospy.loginfo ('EL Exiting ResetHardware()')
        return rv
    
        
# End class Reset()        

        

#######################################################################################################
#######################################################################################################
# Action()
# 
# Control the laser according to the experimentparams.  Turns off when done.
# This state allows enabling the laser only when the given object's state (i.e. Fly state) is 
# in a restricted domain of states.
#
class Action (smach.State):
    def __init__(self, mode='trial', tfrx=None):
        self.tfrx = tfrx
        
        smach.State.__init__(self, 
                             outcomes=['disabled','preempt','aborted'],
                             input_keys=['experimentparamsIn'],
                             output_keys=['experimentparamsOut'])
        
        self.mode = mode
        self.rosrate = rospy.Rate(rospy.get_param('experiment/looprate', 50))
        self.arenastate = None
        self.dtVelocity = rospy.Duration(rospy.get_param('tracking/dtVelocity', 0.2)) # Interval over which to calculate velocity.

        self.pubGalvoCommand    = rospy.Publisher('galvodirector/command', MsgGalvoCommand, latch=True)
        queue_size_arenastate   = rospy.get_param('tracking/queue_size_arenastate', 1)
        self.subArenaState      = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)

        self.Statefilter = ExperimentLibStatefilter.Statefilter()

        rospy.on_shutdown(self.OnShutdown_callback)
        
        # Command messages.
        self.commandExperiment = 'continue'
        self.commandExperiment_list = ['continue','pause_now','pause_after_trial', 'exit_after_trial', 'exit_now']
        self.subCommand = rospy.Subscriber('experiment/command', String, self.CommandExperiment_callback)


    def CommandExperiment_callback(self, msgString):
        self.commandExperiment = msgString.data
            
        
    def OnShutdown_callback(self):
        pass


    def ArenaState_callback(self, arenastate):
        self.arenastate = arenastate
        
    
        
    # step_time_elapsed()
    # Increment the elapsed time, adjusting for the pause/continue status of the experiment.
    #
    def step_time_elapsed(self):
        timeNow = rospy.Time.now()
        if (self.commandExperiment != 'pause_now'):
            dt = timeNow - self.timePrev
        else:
            dt = rospy.Duration(0)

        self.timePrev = timeNow
        self.experimentparams.experiment.timeTrialElapsed += dt.to_sec()


        
    def execute(self, userdata):
        self.experimentparams = copy.deepcopy(userdata.experimentparamsIn)
        self.timePrev = rospy.Time.now()

        if self.mode == 'pre':
            self.paramsIn = self.experimentparams.pre
        if self.mode == 'trial':
            self.paramsIn = self.experimentparams.trial


        for pattern in self.paramsIn.lasergalvos.pattern_list:
            rospy.loginfo("EL State Lasergalvos(%s)" % pattern)

        if self.paramsIn.lasergalvos.enabled:
            rv = 'aborted'
            self.timeStart = rospy.Time.now()
    
            # Create the tracking command for the galvo director.
            commandGalvo = MsgGalvoCommand()
            commandGalvo.enable_laser = True
            commandGalvo.units = 'millimeters' # 'millimeters' or 'volts'
            commandGalvo.pattern_list = self.paramsIn.lasergalvos.pattern_list

            # Determine if we're showing patterns only for certain states.            
            nPatterns = len(self.paramsIn.lasergalvos.pattern_list)
            if len(self.paramsIn.lasergalvos.statefilterLo_list) == nPatterns and \
               len(self.paramsIn.lasergalvos.statefilterHi_list) == nPatterns:
                isStatefiltered = True
            else:
                isStatefiltered = False
                
            if (len(self.paramsIn.lasergalvos.statefilterLo_list) != len(self.paramsIn.lasergalvos.statefilterHi_list)):
                rospy.logwarn('Check your experiment params structure:  Number of statefilters does not match number of patterns.  Not filtering states.')
                

            # Initialize statefilter vars.                
            bInStatefilterRangePrev = [False for i in range(nPatterns)]
            bInStatefilterRange     = [False for i in range(nPatterns)]

                             
            # If unfiltered, publish the command.
            if not isStatefiltered:
                self.pubGalvoCommand.publish(commandGalvo)
    
            # Move galvos until preempt or timeout.        
            while not rospy.is_shutdown():
                self.step_time_elapsed()
                
                # If state is used to determine when pattern is shown.
                if isStatefiltered:
                    # Check if any filterstates have changed.
                    bFilterStateChanged = False
                    for iPattern in range(nPatterns):
                        # Get the pattern.
                        pattern = self.paramsIn.lasergalvos.pattern_list[iPattern]

                        # Convert strings to dicts.
                        statefilterLo_dict = eval(self.paramsIn.lasergalvos.statefilterLo_list[iPattern])
                        statefilterHi_dict = eval(self.paramsIn.lasergalvos.statefilterHi_list[iPattern])
                        statefilterCriteria = self.paramsIn.lasergalvos.statefilterCriteria_list[iPattern]
                
                        # If possible, take the pose &/or velocity &/or speed from arenastate, else use transform via ROS.
                        pose = None
                        velocity = None     
                        speed = None   
                        if self.arenastate is not None:
                            if ('Robot' in pattern.frameidPosition):
                                #pose = self.arenastate.robot.pose  # For consistency w/ galvodirector, we'll get pose via transform.
                                velocity = self.arenastate.robot.velocity
                                speed = self.arenastate.robot.speed
                                
                            elif ('Fly' in pattern.frameidPosition):
                                for iFly in range(len(self.arenastate.flies)):
                                    if (self.arenastate.flies[iFly].name in pattern.frameidPosition):
                                        #pose = self.arenastate.flies[iFly].pose  # For consistency w/ galvodirector, we'll get pose via transform.
                                        velocity = self.arenastate.flies[iFly].velocity
                                        speed = self.arenastate.flies[iFly].speed
                                        break

                        # Get the timestamp for transforms.
                        stamp=None
                        if (pose is None) or (velocity is None):
                            try:
                                stamp = self.tfrx.getLatestCommonTime('Arena', pattern.frameidPosition)
                            except tf.Exception:
                                pass

                            
                        # If we still need the pose (i.e. the frame wasn't in arenastate), then get it from ROS.
                        if (pose is None) and (stamp is not None) and self.tfrx.canTransform('Arena', pattern.frameidPosition, stamp):
                            try:
                                poseStamped = self.tfrx.transformPose('Arena', PoseStamped(header=Header(stamp=stamp,
                                                                                                          frame_id=pattern.frameidPosition),
                                                                                            pose=Pose(position=Point(0,0,0),
                                                                                                      orientation=Quaternion(0,0,0,1)
                                                                                                      )
                                                                                            )
                                                                      )
                                pose = poseStamped.pose
                            except tf.Exception:
                                pose = None

                                
                        # If we still need the velocity, then get it from ROS.
                        if (velocity is None) and (stamp is not None) and self.tfrx.canTransform('Arena', pattern.frameidPosition, stamp):
                            try:
                                velocity_tuple = self.tfrx.lookupTwist('Arena', pattern.frameidPosition, stamp, self.dtVelocity)
                            except tf.Exception:
                                velocity = None
                            else:
                                velocity = Twist(linear=Point(x=velocity_tuple[0][0],
                                                              y=velocity_tuple[0][1],
                                                              z=velocity_tuple[0][2]), 
                                                 angular=Point(x=velocity_tuple[1][0],
                                                               y=velocity_tuple[1][1],
                                                               z=velocity_tuple[1][2]))

                        # If we still need the speed, then get it from velocity.
                        if (speed is None) and (velocity is not None):
                            speed = np.linalg.norm([velocity.linear.x, velocity.linear.y, velocity.linear.z])

                                                        
                        # See if any of the states are in range of the filter.
                        if (pose is not None) and (velocity is not None) and (speed is not None):
                            state = MsgFrameState(pose = pose, 
                                                  velocity = velocity,
                                                  speed = speed)
                            bInStatefilterRangePrev[iPattern] = bInStatefilterRange[iPattern]
                            bInStatefilterRange[iPattern] = self.Statefilter.InRange(state, statefilterLo_dict, statefilterHi_dict, statefilterCriteria)
                            self.Statefilter.PublishMarkers (state, statefilterLo_dict, statefilterHi_dict, statefilterCriteria)
                            
                            # If any of the filter states have changed, then we need to update them all.
                            if bInStatefilterRangePrev[iPattern] != bInStatefilterRange[iPattern]:
                                bFilterStateChanged = True
                    # end for iPattern in range(nPatterns)
                    
                    # If filter state has changed, then publish the new command                    
                    if bFilterStateChanged:
                        commandGalvo.pattern_list = []
                        for iPattern in range(nPatterns):
                            if bInStatefilterRange[iPattern]:
                                pattern = self.paramsIn.lasergalvos.pattern_list[iPattern]
                                commandGalvo.pattern_list.append(pattern)
                    
                        if len(commandGalvo.pattern_list)>0:
                            commandGalvo.enable_laser = True
                        else:
                            commandGalvo.enable_laser = False
                            
                        self.pubGalvoCommand.publish(commandGalvo)

                # else commandGalvo.pattern_list contains all patterns, and has already been published.
                
                
                if self.preempt_requested():
                    rospy.loginfo('preempt requested: Lasergalvos()')
                    self.service_preempt()
                    rv = 'preempt'
                    break
    
                self.rosrate.sleep()


                # Handle commands.
                if (self.commandExperiment=='continue'):
                    pass
                
                if (self.commandExperiment=='pause_now'):
                    while (self.commandExperiment=='pause_now'):
                        rospy.sleep(0.5)
                        self.timePrev = rospy.Time.now()

                if (self.commandExperiment=='pause_after_trial'):
                    pass
                
                if (self.commandExperiment=='exit_after_trial'):
                    pass
                
                if (self.commandExperiment=='exit_now'):
                    rv = 'aborted'
                    break
                
            # end while()
                
        else:
            rv = 'disabled'


                
        # Turn off the laser.
        commandGalvo = MsgGalvoCommand()
        commandGalvo.enable_laser = False
        self.pubGalvoCommand.publish(commandGalvo)
        
                
        userdata.experimentparamsOut = self.experimentparams
        return rv
# End class Action()
    

            
