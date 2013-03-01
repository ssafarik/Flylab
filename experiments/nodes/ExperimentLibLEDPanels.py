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
from flycore.srv import SrvFrameState, SrvFrameStateRequest
from experiments.srv import Trigger, ExperimentParams
from LEDPanels.msg import MsgPanelsCommand
from tracking.msg import ArenaState
from visualization_msgs.msg import Marker


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
        self.pubLEDPanelsCommand = rospy.Publisher('LEDPanels/command', MsgPanelsCommand, latch=True)

        rospy.on_shutdown(self.OnShutdown_callback)
        
        
        # Command messages.
        self.commandExperiment = 'continue'
        self.commandExperiment_list = ['continue','pause', 'stage/calibrate', 'exit', 'exitnow']
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

        self.pubMarker           = rospy.Publisher('visualization_marker', Marker)
        self.pubLEDPanelsCommand = rospy.Publisher('LEDPanels/command', MsgPanelsCommand, latch=True)
        self.subArenaState       = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)

        rospy.on_shutdown(self.OnShutdown_callback)
        
        
        # Command messages.
        self.commandExperiment = 'continue'
        self.commandExperiment_list = ['continue','pause', 'stage/calibrate', 'exit', 'exitnow']
        self.subCommand = rospy.Subscriber('experiment/command', String, self.CommandExperiment_callback)



    def CommandExperiment_callback(self, msgString):
        self.commandExperiment = msgString.data
            
        
    def OnShutdown_callback(self):
        pass
        


    def ArenaState_callback(self, arenastate):
        self.arenastate = arenastate
        
    
    # PublishStatefilterMarkers()
    #  
    def PublishStatefilterMarkers(self, state, statefilterLo_dict, statefilterHi_dict, statefilterCriteria):
        if 'pose' in statefilterLo_dict:
            poseLo_dict = statefilterLo_dict['pose'] 
            poseHi_dict = statefilterHi_dict['pose']
             
            if 'position' in poseLo_dict:
                positionLo_dict = poseLo_dict['position']
                positionHi_dict = poseHi_dict['position']
                (xLo,xHi) = (-9999,+9999)
                (yLo,yHi) = (-9999,+9999)
                (zLo,zHi) = (0,0.1)
                if ('x' in positionLo_dict):
                    xLo = positionLo_dict['x'] 
                    xHi = positionHi_dict['x']
                if ('y' in positionLo_dict):
                    yLo = positionLo_dict['y'] 
                    yHi = positionHi_dict['y']
                if ('z' in positionLo_dict):
                    zLo = positionLo_dict['z'] 
                    zHi = positionHi_dict['z']

                if ('x' in positionLo_dict) or ('y' in positionLo_dict) or ('z' in positionLo_dict):
                    markerTarget = Marker(header=Header(stamp = state.header.stamp,
                                                        frame_id='Arena'),
                                          ns='statefilter',
                                          id=1,
                                          type=Marker.CUBE,
                                          action=0,
#                                          type=Marker.LINE_STRIP
#                                          points=[Point(x=xLo,y=yLo,z=zLo),Point(x=xHi,y=yLo,z=zLo),Point(x=xHi,y=yHi,z=zLo),Point(x=xLo,y=yHi,z=zLo),Point(x=xLo,y=yLo,z=zLo)],
#                                          scale=Vector3(x=0.2, y=0.0, z=0.0),
                                          pose=Pose(position=Point(x=(xLo+xHi)/2, 
                                                                   y=(yLo+yHi)/2, 
                                                                   z=(zLo+zHi)/2)),
                                          scale=Vector3(x=xHi-xLo,
                                                        y=yHi-yLo,
                                                        z=zHi-zLo),
                                          color=ColorRGBA(a=0.1,
                                                          r=1.0,
                                                          g=1.0,
                                                          b=1.0),
                                          lifetime=rospy.Duration(1.0))
                    self.pubMarker.publish(markerTarget)
        
        
    
    # InStatefilterRange()
    # Check if the given state falls in the given region.
    # For statefilterCriteria=="inclusive", if the given state is within all the terms, then the filter returns True.
    # For statefilterCriteria=="exclusive", if the given state is within all the terms, then the filter returns False.
    #
    # We have to manually go through each of the entries in the dict, rather than
    # using a MsgFrameState, since we need the dict to only contain the entries
    # we care about, and the MsgFrameState always contains them all.
    #  
    def InStatefilterRange(self, state, statefilterLo_dict, statefilterHi_dict, statefilterCriteria):
        rv = True
        if 'pose' in statefilterLo_dict:
            poseLo_dict = statefilterLo_dict['pose'] 
            poseHi_dict = statefilterHi_dict['pose']
             
            if 'position' in poseLo_dict:
                 positionLo_dict = poseLo_dict['position']
                 positionHi_dict = poseHi_dict['position']
                 if 'x' in positionLo_dict:
                     xLo = positionLo_dict['x'] 
                     xHi = positionHi_dict['x']
                     if (state.pose.position.x < xLo) or (xHi < state.pose.position.x):
                         rv = False   
                 if 'y' in positionLo_dict:
                     yLo = positionLo_dict['y'] 
                     yHi = positionHi_dict['y']
                     if (state.pose.position.y < yLo) or (yHi < state.pose.position.y):
                         rv = False   
                 if 'z' in positionLo_dict:
                     zLo = positionLo_dict['z'] 
                     zHi = positionHi_dict['z']
                     if (state.pose.position.z < zLo) or (zHi < state.pose.position.z):
                         rv = False   
                         
            if 'orientation' in poseLo_dict:
                 orientationLo_dict = poseLo_dict['orientation']
                 orientationHi_dict = poseHi_dict['orientation']
                 if 'x' in orientationLo_dict:
                     xLo = orientationLo_dict['x'] 
                     xHi = orientationHi_dict['x']
                     if (state.pose.orientation.x < xLo) or (xHi < state.pose.orientation.x):
                         rv = False   
                 if 'y' in orientationLo_dict:
                     yLo = orientationLo_dict['y'] 
                     yHi = orientationHi_dict['y']
                     if (state.pose.orientation.y < yLo) or (yHi < state.pose.orientation.y):
                         rv = False   
                 if 'z' in orientationLo_dict:
                     zLo = orientationLo_dict['z'] 
                     zHi = orientationHi_dict['z']
                     if (state.pose.orientation.z < zLo) or (zHi < state.pose.orientation.z):
                         rv = False   
                 if 'w' in orientationLo_dict:
                     wLo = orientationLo_dict['w'] 
                     wHi = orientationHi_dict['w']
                     if (state.pose.orientation.w < wLo) or (wHi < state.pose.orientation.w):
                         rv = False   

        if 'velocity' in statefilterLo_dict:
            velocityLo_dict = statefilterLo_dict['velocity'] 
            velocityHi_dict = statefilterHi_dict['velocity']
             
            if 'linear' in velocityLo_dict:
                 linearLo_dict = velocityLo_dict['linear']
                 linearHi_dict = velocityHi_dict['linear']
                 if 'x' in linearLo_dict:
                     xLo = linearLo_dict['x'] 
                     xHi = linearHi_dict['x']
                     if (state.velocity.linear.x < xLo) or (xHi < state.velocity.linear.x):
                         rv = False   
                 if 'y' in linearLo_dict:
                     yLo = linearLo_dict['y'] 
                     yHi = linearHi_dict['y']
                     if (state.velocity.linear.y < yLo) or (yHi < state.velocity.linear.y):
                         rv = False   
                 if 'z' in linearLo_dict:
                     zLo = linearLo_dict['z'] 
                     zHi = linearHi_dict['z']
                     if (state.velocity.linear.z < zLo) or (zHi < state.velocity.linear.z):
                         rv = False   
                         
            if 'angular' in velocityLo_dict:
                 angularLo_dict = velocityLo_dict['angular']
                 angularHi_dict = velocityHi_dict['angular']
                 if 'x' in angularLo_dict:
                     xLo = angularLo_dict['x'] 
                     xHi = angularHi_dict['x']
                     if (state.velocity.angular.x < xLo) or (xHi < state.velocity.angular.x):
                         rv = False   
                 if 'y' in angularLo_dict:
                     yLo = angularLo_dict['y'] 
                     yHi = angularHi_dict['y']
                     if (state.velocity.angular.y < yLo) or (yHi < state.velocity.angular.y):
                         rv = False   
                 if 'z' in angularLo_dict:
                     zLo = angularLo_dict['z'] 
                     zHi = angularHi_dict['z']
                     if (state.velocity.angular.z < zLo) or (zHi < state.velocity.angular.z):
                         rv = False
                     
        if 'speed' in statefilterLo_dict:
            speedLo = statefilterLo_dict['speed'] 
            speedHi = statefilterHi_dict['speed']
            #speed = N.linalg.norm([state.velocity.linear.x, state.velocity.linear.y, state.velocity.linear.z])
            if (state.speed < speedLo) or (speedHi < state.speed):
                rv = False   

        if (statefilterCriteria=="exclusive"):
            rv = not rv
            
        return rv
    
    
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
    
            # Operate panels until preempt or timeout.        
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
                            if self.paramsIn.ledpanels.frame_id == self.arenastate.flies[iFly].name:
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
                        bInStatefilterRange = self.InStatefilterRange(state, statefilterLo_dict, statefilterHi_dict, statefilterCriteria)
                        self.PublishStatefilterMarkers (state, statefilterLo_dict, statefilterHi_dict, statefilterCriteria)
                        
                    # Check if the filter state has changed.
                    if bInStatefilterRangePrev != bInStatefilterRange:
                        bFilterStateChanged = True
                        
                    #rospy.logwarn ('%s: %s' % (bFilterStateChanged, bInStatefilterRange))
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
                        
                        #rospy.logwarn('(x1,y1)=%f,%f,       (x2,y2)=%f,%f,      angle=%f' % (x1,y1,x2,y2,angle))
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
    
                #if self.paramsIn.ledpanels.timeout != -1:
                #    if (rospy.Time.now().to_sec()-self.timeStart.to_sec()) > self.paramsIn.ledpanels.timeout:
                #        rv = 'timeout'
                #        break
                
                self.rosrate.sleep()

                if (self.commandExperiment=='exitnow'):
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

            
            
