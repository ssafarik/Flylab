#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('experiments')
import rospy
import actionlib
import copy
import numpy as N
import smach
import tf

from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped, Quaternion, Twist, Vector3
from std_msgs.msg import Header, ColorRGBA, String
from flycore.msg import MsgFrameState, TrackingCommand
from ledpanels.msg import MsgPanelsCommand
from tracking.msg import ArenaState
from visualization_msgs.msg import Marker

import ExperimentLibStatefilter


#######################################################################################################
#######################################################################################################
class Reset (smach.State):
    def __init__(self, tfrx=None):
        self.tfrx = tfrx
        smach.State.__init__(self, 
                             outcomes=['success','disabled','preempt','aborted'],
                             input_keys=['experimentparamsIn'])

        self.arenastate = None
        self.rosrate = rospy.Rate(rospy.get_param('experiment/looprate', 50))

        queue_size_arenastate = rospy.get_param('tracking/queue_size_arenastate', 1)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)
        self.pubLEDPanelsCommand = rospy.Publisher('ledpanels/command', MsgPanelsCommand, latch=True)

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



    # Init the LEDPanels to either off, or to the pretrial state, depending on what's requested.
    def execute(self, userdata):
        rospy.loginfo("EL State ResetLEDPanels()")

        rv = 'disabled'
        if (userdata.experimentparamsIn.pre.ledpanels.enabled):
            rv = 'success'
            msgPanelsCommand = MsgPanelsCommand(command='stop')
            self.pubLEDPanelsCommand.publish (msgPanelsCommand)

            msgPanelsCommand = MsgPanelsCommand(command='set_pattern_id', 
                                                arg1=userdata.experimentparamsIn.pre.ledpanels.idPattern)
            self.pubLEDPanelsCommand.publish (msgPanelsCommand)

            msgPanelsCommand = MsgPanelsCommand(command='set_position', 
                                                arg1=userdata.experimentparamsIn.pre.ledpanels.origin.x, 
                                                arg2=userdata.experimentparamsIn.pre.ledpanels.origin.y)  # Set (x,y) position for the experiment.
            self.pubLEDPanelsCommand.publish (msgPanelsCommand)
        else:
            msgPanelsCommand = MsgPanelsCommand(command='all_off')
            self.pubLEDPanelsCommand.publish (msgPanelsCommand)
            

        return rv
# End class Reset()        

        

#######################################################################################################
#######################################################################################################
# Action()
# 
# Control the LEDPanels according to the experimentparams.  
# This smach state allows enabling the panels only when the given object's state (i.e. Fly state) is 
# in a restricted domain of states.
#
class Action (smach.State):
    def __init__(self, mode='trial', tfrx=None):
        self.tfrx = tfrx
        
        smach.State.__init__(self, 
                             outcomes=['disabled','preempt','aborted'],
                             input_keys=['experimentparamsIn'])
        
        self.mode = mode
        
        # If this rate is too fast, then we get panel glitches.  50Hz=ok, 100Hz=not.  Alternatively could use the serial port rtscts, but it doesn't seem to be supported.
        self.rosrate = rospy.Rate(rospy.get_param('experiment/looprate', 50))    
        
        self.arenastate = None
        self.dtVelocity = rospy.Duration(rospy.get_param('tracking/dtVelocity', 0.2)) # Interval over which to calculate velocity.
        self.xpanels    = rospy.get_param('ledpanels/xpanels', 1)
        self.ypanels    = rospy.get_param('ledpanels/ypanels', 1)
        queue_size_arenastate    = rospy.get_param('tracking/queue_size_arenastate', 1)

        self.pubLEDPanelsCommand = rospy.Publisher('ledpanels/command', MsgPanelsCommand, latch=True)
        self.subArenaState       = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)

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
        
    
    
    def execute(self, userdata):
        if self.mode == 'pre':
            self.paramsIn = userdata.experimentparamsIn.pre
        if self.mode == 'trial':
            self.paramsIn = userdata.experimentparamsIn.trial

        # Create the panels command.
        command = MsgPanelsCommand(command='all_off', arg1=0, arg2=0, arg3=0, arg4=0)

        if self.paramsIn.ledpanels.enabled:
            self.timeStart = rospy.Time.now()
            rv = 'aborted'
            xmax = self.xpanels * 8 # pixels per panel.
    
            # Determine if we're operating only for certain states.
            if len(self.paramsIn.ledpanels.statefilterHi) > 0:
                isStatefiltered = True
            else:
                isStatefiltered = False

            # Initialize statefilter vars.                
            bInStatefilterRangePrev = True
            bInStatefilterRange     = True

                             
            # Set the panels pattern.
            command.command = 'set_pattern_id'
            command.arg1 = self.paramsIn.ledpanels.idPattern
            self.pubLEDPanelsCommand.publish(command)
    
    
            # Loop sending commands to the panels, until preempt or timeout.        
            while not rospy.is_shutdown():
                # If possible, take the pose &/or velocity &/or speed from arenastate, else use transform via ROS.
                pose = None
                velocity = None     
                speed = None   
                if self.arenastate is not None:
                    if ('Robot' in self.paramsIn.ledpanels.frame_id):
                        #pose = self.arenastate.robot.pose  # For consistency w/ galvodirector, we'll get pose via transform.
                        velocity = self.arenastate.robot.velocity
                        speed = self.arenastate.robot.speed
                        
                    elif ('Fly' in self.paramsIn.ledpanels.frame_id):
                        for iFly in range(len(self.arenastate.flies)):
                            if (self.arenastate.flies[iFly].name in self.paramsIn.ledpanels.frame_id):
                                #pose = self.arenastate.flies[iFly].pose  # For consistency w/ galvodirector, we'll get pose via transform.
                                velocity = self.arenastate.flies[iFly].velocity
                                speed = self.arenastate.flies[iFly].speed
                                break

                # Get the timestamp for transforms.
                stamp=None
                if (pose is None) or (velocity is None):
                    try:
                        stamp = self.tfrx.getLatestCommonTime('Arena', self.paramsIn.ledpanels.frame_id)
                    except tf.Exception:
                        pass

                    
                # If we still need the pose (i.e. the frame wasn't in arenastate), then get it from ROS.
                if (pose is None) and (stamp is not None) and self.tfrx.canTransform('Arena', self.paramsIn.ledpanels.frame_id, stamp):
                    try:
                        poseStamped = self.tfrx.transformPose('Arena', PoseStamped(header=Header(stamp=stamp,
                                                                                              frame_id=self.paramsIn.ledpanels.frame_id),
                                                                                pose=Pose(position=Point(0,0,0),
                                                                                          orientation=Quaternion(0,0,0,1)
                                                                                          )
                                                                                )
                                                           )
                        pose = poseStamped.pose
                    except tf.Exception:
                        pose = None

                        
                # If we still need the velocity, then get it from ROS.
                if (velocity is None) and (stamp is not None) and self.tfrx.canTransform('Arena', self.paramsIn.ledpanels.frame_id, stamp):
                    try:
                        velocity_tuple = self.tfrx.lookupTwist('Arena', self.paramsIn.ledpanels.frame_id, stamp, self.dtVelocity)
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
                    speed = N.linalg.norm([velocity.linear.x, velocity.linear.y, velocity.linear.z])

                                                    
                # See if the state is in range of the filter.
                if isStatefiltered:
                    # Check if any filterstates have changed.
                    bFilterStateChanged = False

                    # Convert filter string to dict.
                    statefilterLo_dict = eval(self.paramsIn.ledpanels.statefilterLo)
                    statefilterHi_dict = eval(self.paramsIn.ledpanels.statefilterHi)
                    statefilterCriteria = self.paramsIn.ledpanels.statefilterCriteria
            
                    if (pose is not None) and (velocity is not None) and (speed is not None):
                        state = MsgFrameState(pose = pose, 
                                              velocity = velocity,
                                              speed = speed)

                        bInStatefilterRangePrev = bInStatefilterRange
                        bInStatefilterRange = self.Statefilter.InRange(state, statefilterLo_dict, statefilterHi_dict, statefilterCriteria)
                        self.Statefilter.PublishMarkers (state, statefilterLo_dict, statefilterHi_dict, statefilterCriteria)
                        
                    # Check if the filter state has changed.
                    if bInStatefilterRangePrev != bInStatefilterRange:
                        bFilterStateChanged = True
                        
                # end if isStatefiltered

                    
                # If in range of the statefilter, then publish the new command                    
                if bInStatefilterRange and (pose is not None) and (velocity is not None) and (speed is not None):
                    bValidCommand = False
                    if self.paramsIn.ledpanels.command == 'fixed':
                        command.command = 'set_position'
                        command.arg1 = self.paramsIn.ledpanels.origin.x
                        command.arg2 = self.paramsIn.ledpanels.origin.y
                        bValidCommand = True

                    elif self.paramsIn.ledpanels.command == 'trackposition':
                        angle = (2.0*N.pi) - N.arctan2(pose.position.y, pose.position.x) % (2.0*N.pi)
                        if N.isfinite(angle):
                            x = xmax * angle / (2.0*N.pi)
                            y = 0
                            command.command = 'set_position'
                            command.arg1 = int(x)
                            command.arg2 = int(y)
                            bValidCommand = True

                    elif self.paramsIn.ledpanels.command == 'trackview':
                        r = rospy.get_param('ledpanels/radius', 120) # Radius of the panels.
                        xp = pose.position.x
                        yp = pose.position.y
                        q = pose.orientation
                        rpy = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
                        theta = -rpy[2] % (2.0 * N.pi)
                        tantheta = N.tan(theta)
                        
                        # Points on the circle intersecting with the fly's axis.
                        x1=(1+tantheta**2)**(-1)*((-1)*yp*tantheta+xp*tantheta**2 + (-1)*(r**2+(-1)*yp**2+2*xp*yp*tantheta+r**2*tantheta**2+(-1)*xp**2*tantheta**2)**(1/2))
                        y1=yp+(-1)*xp*tantheta+tantheta*(1+tantheta**2)**(-1)*((-1)*yp*tantheta+xp*tantheta**2+(-1)*(r**2+(-1)*yp**2+2*xp*yp*tantheta+r**2*tantheta**2+(-1)*xp**2*tantheta**2)**(1/2));
                        x2=(1+tantheta**2)**(-1)*((-1)*yp*tantheta+xp*tantheta**2 + (r**2+(-1)*yp**2+2*xp*yp*tantheta+r**2*tantheta**2+(-1)*xp**2*tantheta**2)**(1/2))
                        y2=yp+(-1)*xp*tantheta+tantheta*(1+tantheta**2)**(-1)*((-1)*yp*tantheta+xp*tantheta**2+(r**2+(-1)*yp**2+2*xp*yp*tantheta+r**2*tantheta**2+(-1)*xp**2*tantheta**2)**(1/2))                        
                        
                        
                        # Choose between the two intersection points by moving forward 1mm, and seeing which point we got closer to.
                        r1 = N.linalg.norm([xp-x1, yp-y1]) # Fly to pt1 distance
                        r2 = N.linalg.norm([xp-x2, yp-y2]) # Fly to pt2 distance
                        xa = xp + 1 * N.cos(theta)         # Go one millimeter forward.
                        ya = yp + 1 * N.sin(theta)
                        r1a= N.linalg.norm([xa-x1, ya-y1]) # Fly+1mm to pt1 distance
                        
                        # If we got closer to pt1 at xa, then use pt1, else use pt2 
                        if r1a < r1:
                            angle = N.arctan2(y1,x1) % (2.0*N.pi)
                        else:
                            angle = N.arctan2(y2,x2) % (2.0*N.pi)
                        
                        if N.isfinite(angle):
                            x = xmax * angle / (2.0*N.pi)
                            y = 0
                            
                            command.command = 'set_position'
                            command.arg1 = int(x)
                            command.arg2 = int(y)
                            bValidCommand = True
                        #else:
                            #rospy.logwarn('isnan: (%f,%f), (%f,%f)' % (x1,y1,x2,y2))
                            
                    # end if command == 'trackposition' or 'trackview'
                        
                    if (bValidCommand):                    
                        self.pubLEDPanelsCommand.publish(command)
                        
            
                if self.preempt_requested():
                    rospy.loginfo('preempt requested: LEDPanels()')
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

                if (self.commandExperiment=='pause_after_trial'):
                    pass
                
                if (self.commandExperiment=='exit_after_trial'):
                    pass
                
                if (self.commandExperiment=='exit_now'):
                    rv = 'aborted'
                    break
                
            # End while not rospy.is_shutdown()

        else:
            rv = 'disabled'
            command.command = 'all_off'
            self.pubLEDPanelsCommand.publish(command)
            
        # end if self.paramsIn.ledpanels.enabled
                
                
        return rv
# End class Action()

            
            
