#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('motorarm')
import rospy
import numpy as N
import serial
import tf
import threading
import PyKDL
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from rosSimpleStep.srv import SrvCalibrate, SrvJointState, SrvSetZero
from flycore.msg import MsgFrameState
from flycore.srv import SrvFrameState, SrvFrameStateRequest, SrvFrameStateResponse
from patterngen.srv import SrvSignal, SrvSignalResponse


class MotorArm:

    def __init__(self):
        self.initialized = False
        self.initializedServices = False
        self.lock = threading.Lock()
        rospy.loginfo('Opening MotorArm device...')
        rospy.init_node('Motorarm')
        rospy.loginfo ('MA name=%s', __name__)
        rospy.on_shutdown(self.OnShutdown_callback)

        # Link lengths (millimeters)
        self.L1             = rospy.get_param('motorarm/L1', 9.9) # Length of link1

        # PID Gains & Parameters.
        self.kP             = rospy.get_param('motorarm/kP', 1.0)
        self.kI             = rospy.get_param('motorarm/kI', 0.0)
        self.kD             = rospy.get_param('motorarm/kD', 0.0)
        self.maxI           = rospy.get_param('motorarm/maxI', 40.0)
        self.kWindup        = rospy.get_param('motorarm/kWindup', 0.0)

        self.radiusReachable = self.L1
        self.radiusArena = rospy.get_param('arena/radius_inner', 25.4)

        self.names = ['joint1']
        self.jointstate1 = None
        self.js = JointState()
        self.js.header.seq = 0
        self.js.header.stamp = rospy.Time.now()
        self.js.header.frame_id = 'Stage'
        self.js.name = self.names
        #self.js.velocity = [0.0]
        
        # Publish & Subscribe
        rospy.Service('set_stage_state',    SrvFrameState, self.SetStageState_callback)
        rospy.Service('get_stage_state',    SrvFrameState, self.GetStageState_callback)
        rospy.Service('home_stage',         SrvFrameState, self.HomeStage_callback)
        rospy.Service('calibrate_stage',    SrvFrameState, self.Calibrate_callback)
        rospy.Service('signal_input',       SrvSignal,     self.SignalInput_callback) # For receiving input from a signal generator.


        self.subVisualPosition    = rospy.Subscriber('VisualPosition', PoseStamped, self.VisualPosition_callback)
        self.pubJointState        = rospy.Publisher('joint_states', JointState)
        #self.pubEndEffector       = rospy.Publisher('EndEffector', MsgFrameState)
        self.pubMarker            = rospy.Publisher('visualization_marker', Marker)
        
        self.tfrx = tf.TransformListener()
        self.tfbx = tf.TransformBroadcaster()

        self.ptsContourRefExternal = None #Point(0,0,0) # Where we want the 'tool'.
        self.ptsContourRef         = None
        self.ptEeSense          = Point(0,0,0)
        self.ptEeCommand        = Point(0,0,0) # Where to command the end-effector.
        self.posesContourVisual  = PoseStamped() # Visual position of the tool.
        self.vecEeError         = Point(0,0,0)
        self.vecEeErrorPrev     = Point(0,0,0)
        self.vecEeDError        = Point(0,0,0)
        self.vecEeIError        = Point(0,0,0)
        self.vecEeIErrorClipped = Point(0,0,0)
        
        self.unwind = 0.0
        self.anglePrev = 0.0
        self.speedCommandTool = None 
        self.speedStageMax  = rospy.get_param('motorarm/speed_max', 200.0)
        
        self.timePrev = rospy.Time.now()
        self.time = rospy.Time.now()

        self.request = SrvFrameStateRequest()
        self.request.state.header.frame_id = 'Stage'
        self.request.state.pose.position.x = 0.0
        self.request.state.pose.position.y = 0.0
        self.request.state.pose.position.z = 0.0


    def LoadServices(self):
        # Load joint1 services.
        # Wait for joint 1 to launch.
        stSrv = 'srvCalibrate_'+self.names[0]
        rospy.wait_for_service(stSrv)
        try:
            self.Calibrate_joint1 = rospy.ServiceProxy(stSrv, SrvCalibrate)
        except (rospy.ServiceException, IOError), e:
            rospy.loginfo ('MA FAILED %s: %s'%(stSrv,e))

        stSrv = 'srvGetState_'+self.names[0]
        rospy.wait_for_service(stSrv)
        try:
            self.GetState_joint1 = rospy.ServiceProxy(stSrv, SrvJointState)
        except (rospy.ServiceException, IOError), e:
            rospy.loginfo ('MA FAILED %s: %s'%(stSrv,e))

        stSrv = 'srvSetPositionAtVel_'+self.names[0]
        rospy.wait_for_service(stSrv)
        try:
            self.SetPositionAtVel_joint1 = rospy.ServiceProxy(stSrv, SrvJointState)
        except (rospy.ServiceException, IOError), e:
            rospy.loginfo ('MA FAILED %s: %s'%(stSrv,e))

        stSrv = 'srvPark_'+self.names[0]
        rospy.wait_for_service(stSrv)
        try:
            self.Park_joint1 = rospy.ServiceProxy(stSrv, SrvJointState)
        except (rospy.ServiceException, IOError), e:
            rospy.loginfo ('MA FAILED %s: %s'%(stSrv,e))

        stSrv = 'srvSetZero_'+self.names[0]
        rospy.wait_for_service(stSrv)
        try:
            self.SetZero_joint1 = rospy.ServiceProxy(stSrv, SrvSetZero)
        except (rospy.ServiceException, IOError), e:
            rospy.loginfo ('MA FAILED %s: %s'%(stSrv,e))


        self.initializedServices = True
        

    def TransformToFrame(self, frame_id, pts):
        try:
            pts.header.stamp = self.tfrx.getLatestCommonTime(frame_id, pts.header.frame_id)
            ptsOut = self.tfrx.transformPoint(frame_id, pts)
        except tf.Exception:
            rospy.logwarn ('MA Exception transforming in TransformToFrame()')
            ptsOut = pts
        
        return ptsOut

            
    def ClipVecToMag (self, pt, magMax):
        magPt = N.linalg.norm([pt.x, pt.y, pt.z])
        if magPt > magMax:
            r = magMax / magPt
        else:
            r = 1.0
            
        return Point(r*pt.x, r*pt.y, r*pt.z)
            
        
    # Clip the point onto the given circle.
    def ClipPtOntoCircle(self, pt, radius):
        r = N.linalg.norm([pt.x, pt.y])
        
        angle = N.arctan2(pt.y, pt.x)
        ptOut = Point(x = radius * N.cos(angle),
                      y = radius * N.sin(angle))
        bClipped = True
        
        return (ptOut, bClipped)
                

    # Clip the tool to the given limit.
    def ClipPtToRadius(self, pt, rLimit):
        r = N.linalg.norm([pt.x, pt.y])
        
        if rLimit < r:
            angle = N.arctan2(pt.y, pt.x)
            ptOut = Point(x = rLimit * N.cos(angle),
                          y = rLimit * N.sin(angle))
            bClipped = True
        else:
            ptOut = pt
            bClipped = False
        
        return (ptOut, bClipped)
                

    def ClipPtToArena(self, pt):
        return self.ClipPtToRadius(pt, self.radiusArena * 1.1)
                
    # Clip the tool to the hardware limit.
    def ClipPtToReachable(self, pt):
        return self.ClipPtOntoCircle(pt, self.radiusReachable) 
                
                
    def ClipPtsToReachable(self, ptsIn):        
        isInReachable = True
        
        # Transform to Arena frame.
        ptsStage = self.TransformToFrame('Stage', ptsIn)

        # Clip to arena radius.
        (ptClipped,bClipped) = self.ClipPtToReachable(ptsStage.point)
        if bClipped:
            ptsStage.point = ptClipped
            isInReachable = False
    
        # Transform back to original frame.
        ptsOut = self.TransformToFrame(ptsIn.header.frame_id, ptsStage)
    
        return (ptsOut, isInReachable)
        

    def ClipPtsToArena(self, ptsIn):        
        isInArena = True
        
        # Transform to Arena frame.
        ptsArena = self.TransformToFrame('Arena', ptsIn)

        # Clip to arena radius.
        (ptClipped,bClipped) = self.ClipPtToArena(ptsArena.point)
        if bClipped:
            ptsArena.point = ptClipped
            isInArena = False
    
        # Transform back to original frame.
        ptsOut = self.TransformToFrame(ptsIn.header.frame_id, ptsArena)
    
        return (ptsOut, isInArena)
        

    # Forward kinematics:  Get x,y from angle 1
    def Get1xyFromAngle (self, angle):
        # Rotate the frames from centered to east-pointing. 
        x = self.L1*N.cos(angle)#+N.pi)
        y = self.L1*N.sin(angle)#+N.pi)

        return (angle, x, y)
    
    
    def Get1FromPt (self, pt):
        angle = self.Get1FromXy(pt.x, pt.y)
        
        return angle
        
    # Get1FromXy()
    # Inverse Kinematics: Get Angle 1 from the (x,y) end-effector position,
    # where (x,y) is in the  frame of the workspace center.
    #
    def Get1FromXy (self, x, y):
        
        angle = (N.arctan2(y,x) % (2.0*N.pi)) + self.unwind
#        angle -= N.pi
        
        if (angle-self.anglePrev) > N.pi: # Clockwise across zero.
            angle -= 2.0*N.pi
            self.unwind -= 2.0*N.pi
            
        elif (angle-self.anglePrev) < -N.pi: # CCW across zero.
            angle += 2.0*N.pi
            self.unwind += 2.0*N.pi 
        
        self.anglePrev = angle
        
        #rospy.logwarn('angle=%0.2f, unwind=%0.2f' % (angle, self.unwind))
        return angle
        
        
    
    def GetStageState_callback(self, reqStageState):
        state = self.GetStageState()
        return state
    
    def GetStageState (self):
        while not self.initializedServices:
            rospy.sleep(0.1)
            
        x = 0.0
        y = 0.0
        

        rvStageState = SrvFrameStateResponse()
        if (self.jointstate1 is not None):                    
            #rospy.loginfo ('MA self.jointstate1=%s' % self.jointstate1)
            #(x,y) = self.GetXyFrom12 (self.jointstate1.position, self.jointstate2.position)
            
            # Transform the point into the requested frame.
            pt = Point()
            pt.x = self.ptEeSense.x + self.vecOffsetSense.x 
            pt.y = self.ptEeSense.y + self.vecOffsetSense.y 
            pt.z = self.ptEeSense.z + self.vecOffsetSense.z 
            
            #rvStageState.state.header.stamp = rospy.Time.now()
            rvStageState.state.header.frame_id = 'Stage' # Always return Stage frame coordinates.
            rvStageState.state.pose.position.x = pt.x 
            rvStageState.state.pose.position.y = pt.y 
            rvStageState.state.pose.position.z = pt.z
            rvStageState.state.velocity.linear.x = 0.0 # BUG: This should come from the hardware.
            rvStageState.state.velocity.linear.y = 0.0 # BUG: This should come from the hardware.
            rvStageState.state.velocity.linear.z = 0.0 # BUG: This should come from the hardware.
            rvStageState.state.velocity.angular.x = 0.0 # BUG: This should come from the hardware.
            rvStageState.state.velocity.angular.y = 0.0 # BUG: This should come from the hardware.
            rvStageState.state.velocity.angular.z = 0.0 # BUG: This should come from the hardware.
        
        return rvStageState.state

        
    # SetStageState_callback()
    #   Updates the target command.
    #
    def SetStageState_callback(self, reqStageState):
        while not self.initialized:
            rospy.sleep(0.5)
            
        self.ptsContourRefExternal = PointStamped(header=reqStageState.state.header,
                                                  point=reqStageState.state.pose.position)
        
        (self.ptsContourRefExternal, isInArena) = self.ClipPtsToArena(self.ptsContourRefExternal)
        self.ptsContourRef = self.TransformToFrame('Stage', self.ptsContourRefExternal)


        #rospy.logwarn('MA ptsContourRef=[%0.2f, %0.2f], ext=[%0.2f, %0.2f]' % (self.ptsContourRef.point.x,self.ptsContourRef.point.y,self.ptsContourRefExternal.point.x,self.ptsContourRefExternal.point.y))
        if reqStageState.speed is not None:
            self.speedCommandTool = reqStageState.speed # Requested speed for positioning.
        else:
            self.speedCommandTool = self.speedStageMax
        #rospy.logwarn('MA reqStageState.speed=%s' % reqStageState.speed)

        rvStageState = SrvFrameStateResponse()
        rvStageState.state.pose.position.x = reqStageState.state.pose.position.x
        rvStageState.state.pose.position.y = reqStageState.state.pose.position.y
        rvStageState.state.pose.position.z = reqStageState.state.pose.position.z

        return rvStageState
    

    def SignalInput_callback (self, srvSignalReq):
        rv = SrvSignalResponse()
        rv.success = True
        self.ptsContourRefExternal = PointStamped(header=srvSignalReq.state.header,
                                                  point=srvSignalReq.state.pose.position) 
        
        (self.ptsContourRefExternal, isInArena) = self.ClipPtsToArena(self.ptsContourRefExternal)
        self.ptsContourRef = self.TransformToFrame('Stage', self.ptsContourRefExternal)
        rv.success = isInArena

        try:
            self.speedCommandTool = self.speedStageMax #srvSignalReq.state.speed #self.speedStageMax #self.speedCommandTool #5.0 * (1.0/self.dtPoint) # Robot travels to target at twice the target speed.
        except AttributeError, e:
            rospy.logwarn('MA AttributeError: %s' % e)
            
        #rospy.logwarn('speed=% 6.1f' % self.speedCommandTool)
        
        return rv


    # Create a vector with direction of ptDir, and magnitude of ptMag.
    def ScaleVecToMag (self, ptDir, ptMag):
        magPtDir = N.linalg.norm([ptDir.x, ptDir.y, ptDir.z])
        magPtMag = N.linalg.norm([ptMag.x, ptMag.y, ptMag.z])
        
        ptNew = Point (x=magPtMag/magPtDir*ptDir.x,
                       y=magPtMag/magPtDir*ptDir.y,
                       z=magPtMag/magPtDir*ptDir.z)
        return ptNew
    

    def VisualPosition_callback(self, posesContour):
        # Transform to Stage frame.
        try:
            posesContour.header.stamp = self.tfrx.getLatestCommonTime('Stage', posesContour.header.frame_id)
            self.posesContourVisual = self.tfrx.transformPose('Stage', posesContour)
        except tf.Exception:
            rospy.logwarn ('MA Exception transforming to Stage frame in VisualPosition_callback()')
        
        
    def HomeStage_callback(self, reqStageState):
        while not self.initialized:
            rospy.sleep(0.1)
            
        with self.lock:
            try:
                self.Park_joint1()
            except (rospy.ServiceException, rospy.exceptions.ROSInterruptException, IOError), e:
                rospy.logwarn ('MA FAILED %s'%e)
                
        rvStageState = SrvFrameStateResponse()
        rospy.loginfo ('MA HomeStage_callback rvStageState=%s' % rvStageState)
        return rvStageState
        
        
    def Calibrate_callback(self, req):
        # Integer values for directions - used in usb set/get
        POSITIVE = 0
        NEGATIVE = 1
        
        # Convert parking coordinates to angles.
        self.xPark = 0.0
        self.yPark = 0.0
        with self.lock:
            q1park = self.Get1FromXy(self.xPark, self.yPark)

        q1park = 0.0
        q1origin = rospy.get_param('motorarm/angleOffset', 0.0) # Angle distance of index switch to 0 angle.

        rospy.loginfo ('MA Calibrating: q1origin=%s, q1park=%s' % (q1origin, q1park))
        with self.lock:
            rv = self.Calibrate_joint1(NEGATIVE, q1origin, q1park, True) # True=find index switch; False=don't find index switch.
            rospy.loginfo ('MA Calibrated joint1')
            q1index = rv.position

        rvStageState = self.GetStageState()
        rospy.loginfo ('MA Calibrate_callback rvStageState=%s' % rvStageState)
        return rvStageState
    

    def PublishMarker (self, pt, id, name):
        marker = Marker(header=Header(stamp=rospy.Time.now(),
                                      frame_id='Stage'),
                          ns=name,
                          id=id,
                          type=Marker.SPHERE,
                          action=0,
                          pose=Pose(position=Point(x=pt.x, 
                                                   y=pt.y, 
                                                   z=pt.z)),
                          scale=Vector3(x=2.0,
                                        y=2.0,
                                        z=2.0),
                          color=ColorRGBA(a=0.5,
                                          r=0.1,
                                          g=0.1,
                                          b=1.0),
                          lifetime=rospy.Duration(1.0))
        self.pubMarker.publish(marker)


    def SendTransforms(self):  
        if self.initialized:
            with self.lock:
                try:
                    self.jointstate1 = self.GetState_joint1()
                except (rospy.ServiceException, IOError), e:
                    rospy.logwarn ('MA FAILED %s'%e)
                    self.jointstate1 = None
                #rospy.logwarn('MA joint1=% 3.1f' % self.jointstate1.position)
            
            if (self.jointstate1 is not None):                 
                (angleStage, xStage, yStage) = self.Get1xyFromAngle(self.jointstate1.position)
                ptsStage = PointStamped(header=Header(frame_id='Stage'),
                                        point=Point(x=xStage, y=yStage, z=0.0))
                ptsArena = self.TransformToFrame('Arena', ptsStage)
                self.ptEeSense = ptsStage.point
#                rospy.logwarn('eeSense: % 4.1f -> (% 4.1f, % 4.1f)' % (self.jointstate1.position, self.ptEeSense.x, self.ptEeSense.y))

                # Publish the joint states (for rviz, etc)    
                self.js.header.seq = self.js.header.seq + 1
                self.js.header.stamp = self.time
                self.js.position = [angleStage]
                self.pubJointState.publish(self.js)
    
                qEE = tf.transformations.quaternion_from_euler(0.0, 0.0, angleStage)
                state = MsgFrameState()
                state.header.stamp = self.time
                state.header.frame_id = 'Stage'
                state.pose.position.x = ptsStage.point.x
                state.pose.position.y = ptsStage.point.y
                state.pose.position.z = ptsStage.point.z
                state.pose.orientation.x = qEE[0]
                state.pose.orientation.y = qEE[1]
                state.pose.orientation.z = qEE[2]
                state.pose.orientation.w = qEE[3]
#                self.pubEndEffector.publish (state)

                
                # Publish the link transforms.
                self.tfbx.sendTransform((0.0, 0.0, 0.0), 
                                        tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                                        self.time,
                                        'link0',     # child
                                        'Stage'      # parent
                                        )

                self.tfbx.sendTransform((0.0, 0.0, 0.0), 
                                        qEE,
                                        self.time,
                                        'link1',     # child
                                        'link0'      # parent
                                        )
                
                # Frame EndEffector
                self.tfbx.sendTransform((state.pose.position.x, state.pose.position.y, state.pose.position.z), 
                                        qEE,
                                        self.time,
                                        'EndEffector',     # child
                                        state.header.frame_id      # parent
                                        )
                
                
                # Frame Target
                if self.ptsContourRef is not None:
                    markerTarget = Marker(header=self.ptsContourRef.header,
                                          ns='target',
                                          id=2,
                                          type=Marker.SPHERE,
                                          action=0,
                                          pose=Pose(position=Point(x=self.ptsContourRef.point.x, 
                                                                   y=self.ptsContourRef.point.y, 
                                                                   z=self.ptsContourRef.point.z)),
                                          scale=Vector3(x=3.0,
                                                        y=3.0,
                                                        z=3.0),
                                          color=ColorRGBA(a=0.5,
                                                          r=1.0,
                                                          g=0.1,
                                                          b=0.1),
                                          lifetime=rospy.Duration(1.0))
                    self.pubMarker.publish(markerTarget)

                    markerToolOffset   = Marker(header=state.header,
                                              ns='tooloffset',
                                              id=3,
                                              type=Marker.ARROW,
                                              action=0,
                                              scale=Vector3(x=0.1, # Shaft diameter
                                                            y=0.2, # Head diameter
                                                            z=0.0),
                                              color=ColorRGBA(a=0.8,
                                                              r=1.0,
                                                              g=1.0,
                                                              b=1.0),
                                              lifetime=rospy.Duration(1.0),
                                              points=[Point(x=state.pose.position.x, 
                                                            y=state.pose.position.y, 
                                                            z=state.pose.position.z),
                                                        Point(x=self.posesContourVisual.pose.position.x, 
                                                              y=self.posesContourVisual.pose.position.y, 
                                                              z=self.posesContourVisual.pose.position.z)])
                    self.pubMarker.publish(markerToolOffset)



    # UpdateMotorCommandFromTarget()
    #   Updates the motor command with the current target.
    #
    def UpdateMotorCommandFromTarget(self):
        #rospy.loginfo ('MA ptToolRef=%s' % self.ptToolRef)
        if self.ptsContourRef is not None:
            # PID Gains & Parameters.
            self.kP      = rospy.get_param('motorarm/kP', 1.0)
            self.kI      = rospy.get_param('motorarm/kI', 0.0)
            self.kD      = rospy.get_param('motorarm/kD', 0.0)
            self.maxI    = rospy.get_param('motorarm/maxI', 40.0)
            self.kWindup = rospy.get_param('motorarm/kWindup', 0.0)
            self.kAll    = rospy.get_param('motorarm/kAll', 1.0)
            
            self.kP *= self.kAll
            self.kI *= self.kAll
            self.kD *= self.kAll
            
            # The contour error.
            self.vecEeErrorPrev.x = self.vecEeError.x 
            self.vecEeErrorPrev.y = self.vecEeError.y 
            self.vecEeError.x = self.ptsContourRef.point.x - self.posesContourVisual.pose.position.x
            self.vecEeError.y = self.ptsContourRef.point.y - self.posesContourVisual.pose.position.y

            # PID control of the contour error.
            self.vecEeIError.x = self.vecEeIError.x + self.vecEeError.x
            self.vecEeIError.y = self.vecEeIError.y + self.vecEeError.y
            self.vecEeDError.x = self.vecEeError.x - self.vecEeErrorPrev.x
            self.vecEeDError.y = self.vecEeError.y - self.vecEeErrorPrev.y
            vecPID = Point(x = self.kP*self.vecEeError.x + self.kI*self.vecEeIError.x + self.kD*self.vecEeDError.x,
                           y = self.kP*self.vecEeError.y + self.kI*self.vecEeIError.y + self.kD*self.vecEeDError.y)
            
            # Clip the integral error to the reachable workspace (prevents accumulating error due to singularity in radial direction).
            (ptClipped, bClipped) = self.ClipPtToReachable(Point(x = self.ptEeSense.x + self.kI*self.vecEeIError.x,
                                                                 y = self.ptEeSense.y + self.kI*self.vecEeIError.y))
            if self.kI != 0:
                self.vecEeIError.x = (ptClipped.x - self.ptEeSense.x) / self.kI
                self.vecEeIError.y = (ptClipped.y - self.ptEeSense.y) / self.kI
            else:
                self.vecEeIError.x = (ptClipped.x - self.ptEeSense.x)
                self.vecEeIError.y = (ptClipped.y - self.ptEeSense.y)
            
            
            # Anti-windup
            self.vecEeIErrorClipped = self.ClipVecToMag (self.vecEeIError, self.maxI)
            vecExcessI = Point(self.vecEeIError.x - self.vecEeIErrorClipped.x,
                               self.vecEeIError.y - self.vecEeIErrorClipped.y,
                               self.vecEeIError.z - self.vecEeIErrorClipped.z)
            self.vecEeIError.x -= self.kWindup * vecExcessI.x
            self.vecEeIError.y -= self.kWindup * vecExcessI.y
            
            # Print the PID component values.
            magP = N.linalg.norm([self.vecEeError.x, self.vecEeError.y])
            magI = N.linalg.norm([self.vecEeIError.x, self.vecEeIError.y])
            magD = N.linalg.norm([self.vecEeDError.x, self.vecEeDError.y])
            magPID = N.linalg.norm([vecPID.x, vecPID.y])
            


            # Get the command for the hardware, clipped to arena coords.
            ptEeCommandRaw = Point(x = self.ptEeSense.x + vecPID.x,
                                   y = self.ptEeSense.y + vecPID.y)
            ptsEeCommandRaw = PointStamped(header=Header(stamp=self.posesContourVisual.header.stamp,
                                                         frame_id='Stage'),
                                           point=ptEeCommandRaw)
            
            (ptsEeCommandArena, isInArena) = self.ClipPtsToArena(ptsEeCommandRaw)
            (ptsEeCommandClipped, isInReachable) = self.ClipPtsToReachable(ptsEeCommandArena)
            self.ptEeCommand = ptsEeCommandClipped.point
#            rospy.logwarn('contourRef:(% 4.1f, % 4.1f), contourSense:(% 4.1f, % 4.1f)' % (self.ptsContourRef.point.x, self.ptsContourRef.point.y, 
#                                                                                          self.posesContourVisual.pose.position.x, self.posesContourVisual.pose.position.y))
#            rospy.logwarn('(% 4.1f, % 4.1f)' % (self.ptEeCommand.x, self.ptEeCommand.y))
            
            vecPIDclipped = Point(x=ptsEeCommandClipped.point.x-self.ptEeSense.x,
                                  y=ptsEeCommandClipped.point.y-self.ptEeSense.y)
            rospy.logwarn('[P,I,D]=[% 6.2f,% 6.2f,% 6.2f], PID=% 7.2f, % 7.2f' % (magP,magI,magD, magPID, N.linalg.norm([vecPIDclipped.x,vecPIDclipped.y])))
            
            # Display a vector in rviz.
            ptBase = self.ptEeSense
            ptEnd = self.ptEeCommand
            markerCommand= Marker(header=Header(stamp = self.time,
                                                frame_id='Stage'),
                                  ns='command',
                                  id=4,
                                  type=Marker.ARROW,
                                  action=0,
                                  scale=Vector3(x=0.1, # Shaft diameter
                                                y=0.2, # Head diameter
                                                z=0.0),
                                  color=ColorRGBA(a=0.9,
                                                  r=0.5,
                                                  g=1.0,
                                                  b=0.5),
                                  lifetime=rospy.Duration(1.0),
                                  points=[ptBase, ptEnd])
            self.pubMarker.publish(markerCommand)
            
#            # Display P,I,D vectors in rviz.
#            ptBase = self.ptEeSense
#            ptEnd = Point(x = self.ptEeSense.x + self.kP*self.vecEeError.x,
#                          y = self.ptEeSense.y + self.kP*self.vecEeError.y)
#            markerCommand= Marker(header=Header(stamp = self.time, frame_id='Stage'),
#                                  ns='P',
#                                  id=5, type=Marker.ARROW, action=0,
#                                  scale=Vector3(x=0.1, y=0.2, z=0.0),
#                                  color=ColorRGBA(a=0.9, r=1.0, g=0.0, b=0.0),
#                                  lifetime=rospy.Duration(1.0), points=[ptBase, ptEnd])
#            self.pubMarker.publish(markerCommand)
#            
#            ptBase = self.ptEeSense
#            ptEnd = Point(x = self.ptEeSense.x + self.kI*self.vecEeIError.x,
#                          y = self.ptEeSense.y + self.kI*self.vecEeIError.y)
#            markerCommand= Marker(header=Header(stamp = self.time, frame_id='Stage'),
#                                  ns='I',
#                                  id=6, type=Marker.ARROW, action=0,
#                                  scale=Vector3(x=0.1, y=0.2, z=0.0),
#                                  color=ColorRGBA(a=0.9, r=0.0, g=1.0, b=0.0),
#                                  lifetime=rospy.Duration(1.0), points=[ptBase, ptEnd])
#            self.pubMarker.publish(markerCommand)
#            ptBase = self.ptEeSense
#            ptEnd = Point(x = self.ptEeSense.x + self.kD*self.vecEeDError.x,
#                          y = self.ptEeSense.y + self.kD*self.vecEeDError.y)
#            markerCommand= Marker(header=Header(stamp = self.time, frame_id='Stage'),
#                                  ns='D',
#                                  id=7, type=Marker.ARROW, action=0,
#                                  scale=Vector3(x=0.1, y=0.2, z=0.0),
#                                  color=ColorRGBA(a=0.9, r=0.0, g=0.0, b=1.0),
#                                  lifetime=rospy.Duration(1.0), points=[ptBase, ptEnd])
#            self.pubMarker.publish(markerCommand)

            
            
            angleNext = self.Get1FromPt(self.ptEeCommand)
            
            # Compute deltas for speed calc.
            if (self.jointstate1 is not None):
                speedNext = self.speedCommandTool / self.L1 # Convert linear speed to angular speed.
                #rospy.logwarn('angleNext=% 3.1f, lin/ang speedNext=% 3.1f/% 3.1f' % (angleNext, self.speedCommandTool, speedNext))
                
                with self.lock:
                    try:
                        self.SetPositionAtVel_joint1(Header(frame_id=self.names[0]), angleNext, speedNext)
                    except (rospy.ServiceException, IOError), e:
                        rospy.logwarn ('MA FAILED %s'%e)

        
    def OnShutdown_callback(self):
        self.Calibrate_callback(None)
        rospy.logwarn('MA Closed MotorArm device.')
        

        
    def Mainloop(self):
        self.LoadServices()
        self.Calibrate_callback(None)
        self.initialized = True

        # Process messages forever.
        rosrate = rospy.Rate(100)
        #try:
        while not rospy.is_shutdown():
            self.time = rospy.Time.now()
            self.dt = self.time - self.timePrev
            self.timePrev = self.time

            #rospy.logwarn('jointstate1=%s' % self.jointstate1)
            self.SendTransforms()
            self.UpdateMotorCommandFromTarget()
            rosrate.sleep()
                
        self.Calibrate_callback(None)
    
    

if __name__ == '__main__':
    try:
        motorarm = MotorArm()
        motorarm.Mainloop()
    except rospy.exceptions.ROSInterruptException: 
        pass
    
    
    
