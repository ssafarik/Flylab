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
from galvodirector.msg import MsgGalvoCommand
from tracking.msg import ArenaState
from visualization_msgs.msg import Marker


#######################################################################################################
#######################################################################################################
class Reset (smach.State):
    def __init__(self, tfrx=None):
        smach.State.__init__(self, 
                             outcomes=['success','disabled','preempt','aborted'],
                             input_keys=['experimentparamsIn'])

        self.arenastate = None
        self.rosrate = rospy.Rate(rospy.get_param('experiment/looprate', 50))

        queue_size_arenastate = rospy.get_param('tracking/queue_size_arenastate', 1)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)

        rospy.on_shutdown(self.OnShutdown_callback)
        
        
        # Command messages.
        self.commandExperiment = 'continue'
        self.commandExperiment_list = ['continue','pause', 'exit', 'exitnow']
        self.subCommand = rospy.Subscriber('experiment/command', String, self.CommandExperiment_callback)


    def CommandExperiment_callback(self, msgString):
        self.commandExperiment = msgString.data
            
        
    def OnShutdown_callback(self):
        pass
        
        
    def ArenaState_callback(self, arenastate):
        self.arenastate = arenastate



    def execute(self, userdata):
        rospy.loginfo("EL State ResetGalvos()")

        # Reset the various hardware.
        rv = 'success'
            
        #rospy.loginfo ('EL Exiting ResetHardware()')
        return rv
    
        
# End class Reset()        

        

#######################################################################################################
#######################################################################################################
# Lasergalvos()
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
                             input_keys=['experimentparamsIn'])
        
        self.mode = mode
        self.rosrate = rospy.Rate(rospy.get_param('experiment/looprate', 50))
        self.arenastate = None
        self.dtVelocity = rospy.Duration(rospy.get_param('tracking/dtVelocity', 0.2)) # Interval over which to calculate velocity.

        self.pubMarker          = rospy.Publisher('visualization_marker', Marker)
        self.pubGalvoCommand    = rospy.Publisher('GalvoDirector/command', MsgGalvoCommand, latch=True)
        queue_size_arenastate = rospy.get_param('tracking/queue_size_arenastate', 1)
        self.subArenaState      = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)

        rospy.on_shutdown(self.OnShutdown_callback)
        
        # Command messages.
        self.commandExperiment = 'continue'
        self.commandExperiment_list = ['continue','pause', 'exit', 'exitnow']
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

        for pattern in self.paramsIn.lasergalvos.pattern_list:
            rospy.loginfo("EL State Lasergalvos(%s)" % pattern)

        if self.paramsIn.lasergalvos.enabled:
            rv = 'aborted'
            self.timeStart = rospy.Time.now()
    
            # Create the tracking command for the galvo director.
            command = MsgGalvoCommand()
            command.enable_laser = True
            command.units = 'millimeters' # 'millimeters' or 'volts'
            command.pattern_list = self.paramsIn.lasergalvos.pattern_list

            # Determine if we're showing patterns only for certain states.            
            nPatterns = len(self.paramsIn.lasergalvos.pattern_list)
            if len(self.paramsIn.lasergalvos.statefilterLo_list) == nPatterns and \
               len(self.paramsIn.lasergalvos.statefilterHi_list) == nPatterns:
                isStatefiltered = True
            else:
                isStatefiltered = False

            # Initialize statefilter vars.                
            bInStatefilterRangePrev = [False for i in range(nPatterns)]
            bInStatefilterRange     = [False for i in range(nPatterns)]

                             
            # If unfiltered, publish the command.
            if not isStatefiltered:
                self.pubGalvoCommand.publish(command)
    
            # Move galvos until preempt or timeout.        
            while not rospy.is_shutdown():
                
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
                                    if pattern.frameidPosition == self.arenastate.flies[iFly].name:
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
                            speed = N.linalg.norm([velocity.linear.x, velocity.linear.y, velocity.linear.z])

                                                        
                        # See if any of the states are in range of the filter.
                        if (pose is not None) and (velocity is not None) and (speed is not None):
                            state = MsgFrameState(pose = pose, 
                                                  velocity = velocity,
                                                  speed = speed)
    
                            bInStatefilterRangePrev[iPattern] = bInStatefilterRange[iPattern]
                            bInStatefilterRange[iPattern] = self.InStatefilterRange(state, statefilterLo_dict, statefilterHi_dict, statefilterCriteria)
                            self.PublishStatefilterMarkers (state, statefilterLo_dict, statefilterHi_dict, statefilterCriteria)
                            
                            # If any of the filter states have changed, then we need to update them all.
                            if bInStatefilterRangePrev[iPattern] != bInStatefilterRange[iPattern]:
                                bFilterStateChanged = True
                    # end for iPattern in range(nPatterns)
                    
                    #rospy.logwarn ('%s: %s' % (bFilterStateChanged, bInStatefilterRange))
                    
                    # If filter state has changed, then publish the new command                    
                    if bFilterStateChanged:
                        command.pattern_list = []
                        for iPattern in range(nPatterns):
                            if bInStatefilterRange[iPattern]:
                                pattern = self.paramsIn.lasergalvos.pattern_list[iPattern]
                                command.pattern_list.append(pattern)
                    
                        if len(command.pattern_list)>0:
                            command.enable_laser = True
                        else:
                            command.enable_laser = False
                            
                        self.pubGalvoCommand.publish(command)

                # else command.pattern_list contains all patterns, and has already been published.
                
                
                if self.preempt_requested():
                    rospy.loginfo('preempt requested: Lasergalvos()')
                    self.service_preempt()
                    rv = 'preempt'
                    break
    
                #if self.paramsIn.lasergalvos.timeout != -1:
                #    if (rospy.Time.now().to_sec()-self.timeStart.to_sec()) > self.paramsIn.lasergalvos.timeout:
                #        rv = 'timeout'
                #        break
                
                self.rosrate.sleep()

                if (self.commandExperiment=='exitnow'):
                    rv = 'aborted'
                    break
                
        else:
            rv = 'disabled'


                
        # Turn off the laser.
        command = MsgGalvoCommand()
        command.enable_laser = False
        self.pubGalvoCommand.publish(command)
        
                
        return rv
# End class Action()
    

            
