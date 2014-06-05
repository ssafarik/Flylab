#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('motorarm')
import rospy
import copy
import numpy as np
import tf
import threading
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Vector3, Vector3Stamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, ColorRGBA, String
from visualization_msgs.msg import Marker
from rosSimpleStep.srv import SrvCalibrate, SrvJointState
from flycore.msg import MsgFrameState
from flycore.srv import SrvFrameState, SrvFrameStateRequest, SrvFrameStateResponse
from patterngen.srv import SrvSignal, SrvSignalResponse


class MotorArm:

    def __init__(self):
        self.bInitialized = False
        self.bInitializedServices = False
        self.lock = threading.Lock()
        
        rospy.loginfo('Opening MotorArm device...')
        rospy.init_node('Motorarm')
        rospy.loginfo ('MA name=%s', __name__)
        rospy.on_shutdown(self.OnShutdown_callback)

        # Link lengths (millimeters)
        self.L1       = rospy.get_param('motorarm/L1', 9.9)  # Length of link1
        self.T        = rospy.get_param('motorarm/T', 1 / 50)  # Nominal motor update period, i.e. sample rate.

        # PID Gains & Parameters.
        self.kP       = rospy.get_param('motorarm/kP', 1.0)
        self.kI       = rospy.get_param('motorarm/kI', 0.0)
        self.kD       = rospy.get_param('motorarm/kD', 0.0)
        self.maxI     = rospy.get_param('motorarm/maxI', 40.0)
        self.kWindup  = rospy.get_param('motorarm/kWindup', 0.0)

        self.kPv      = rospy.get_param('motorarm/kPv', 0.0)
        self.kIv      = rospy.get_param('motorarm/kIv', 0.0)
        self.kDv      = rospy.get_param('motorarm/kDv', 0.0)
        self.maxIv    = rospy.get_param('motorarm/maxIv', 40.0)
        self.kWindupv = rospy.get_param('motorarm/kWindupv', 0.0)

        
        self.bTune = rospy.get_param('/tune', False)

        self.radiusReachable = self.L1
        self.radiusArena = rospy.get_param('arena/radius_inner', 25.4)

        self.names = ['joint1']
        self.jointstate1 = None
        self.js = JointState()
        self.js.header.seq = 0
        self.js.header.stamp = rospy.Time.now()
        self.js.header.frame_id = 'motor'
        self.js.name = self.names

        # Services.        
        self.services = {}
        self.services['set_stage_state_ref'] = rospy.Service('set_stage_state_ref', SrvFrameState, self.SetStageStateRef_callback)
        self.services['get_stage_state']     = rospy.Service('get_stage_state',     SrvFrameState, self.GetStageState_callback)
        self.services['home_stage']          = rospy.Service('home_stage',          SrvFrameState, self.HomeStage_callback)
        self.services['calibrate_stage']     = rospy.Service('calibrate_stage',     SrvFrameState, self.Calibrate_callback)
        self.services['signal_input']        = rospy.Service('signal_input',        SrvSignal,     self.SignalInput_callback) # For receiving input from a signal generator.


        # Command messages.
        self.commandExperiment = 'continue'
        self.subCommand = rospy.Subscriber('experiment/command', String, self.CommandExperiment_callback)

        # Publish & Subscribe.
        self.subVisualState = rospy.Subscriber('visual_state', MsgFrameState, self.VisualState_callback)
        self.pubJointState = rospy.Publisher('joint_states', JointState)
        self.pubMarker = rospy.Publisher('visualization_marker', Marker)
        
        self.tfrx = tf.TransformListener()
        self.tfbx = tf.TransformBroadcaster()

        self.stateRef = None #MsgFrameState()
        self.stateVisual = None
        self.stateKinematic = MsgFrameState(pose=Pose(position=Point(0,0,0)))

        self.statePError = MsgFrameState()
        self.stateIError = MsgFrameState()
        self.stateDError = MsgFrameState()
        
        self.statePID = MsgFrameState()

        self.angposRef = 0.0
        self.angvelRef = 0.0
        
        self.angposMech = None
        self.angposPError = 0.0
        self.angvelPError = 0.0
        self.angposIError = 0.0
        self.angvelIError = 0.0
        self.angposDError = 0.0
        self.angvelDError = 0.0

        self.unwind = 0.0
        self.angleInvKinPrev = 0.0
        self.speedLinearMax = rospy.get_param('motorarm/speed_max', 200.0)
        
        self.timePrev = rospy.Time.now()
        self.time = rospy.Time.now()
        
        self.request = SrvFrameStateRequest()
        self.request.state.header.frame_id = 'Stage'
        self.request.state.pose.position.x = 0.0
        self.request.state.pose.position.y = 0.0
        self.request.state.pose.position.z = 0.0


    def CommandExperiment_callback(self, msgString):
        self.commandExperiment = msgString.data
            


    def AttachServices(self):
        # Load joint1 services.
        # Wait for joint 1 to launch.
        stSrv = 'srvCalibrate_' + self.names[0]
        try:
            rospy.wait_for_service(stSrv)
            self.Calibrate_joint1 = rospy.ServiceProxy(stSrv, SrvCalibrate)
        except (rospy.ServiceException, IOError), e:
            rospy.logwarn ('MA FAILED %s: %s' % (stSrv, e))

        stSrv = 'srvGetState_' + self.names[0]
        try:
            rospy.wait_for_service(stSrv)
            self.GetState_joint1 = rospy.ServiceProxy(stSrv, SrvJointState, persistent=True)
        except (rospy.ServiceException, IOError), e:
            rospy.logwarn ('MA FAILED %s: %s' % (stSrv, e))

        stSrv = 'srvSetVelocity_' + self.names[0]
        try:
            rospy.wait_for_service(stSrv)
            self.SetVelocity_joint1 = rospy.ServiceProxy(stSrv, SrvJointState, persistent=True)
        except (rospy.ServiceException, IOError), e:
            rospy.logwarn ('MA FAILED %s: %s' % (stSrv, e))

        stSrv = 'srvPark_' + self.names[0]
        try:
            rospy.wait_for_service(stSrv)
            self.Park_joint1 = rospy.ServiceProxy(stSrv, SrvJointState)
        except (rospy.ServiceException, IOError), e:
            rospy.logwarn ('MA FAILED %s: %s' % (stSrv, e))


        self.bInitializedServices = True
        

    # Transform a PointStamped to the given frame.
    def TransformPointToFrame(self, frame_id, pts):
        try:
            pts.header.stamp = self.tfrx.getLatestCommonTime(frame_id, pts.header.frame_id)
            ptsOut = self.tfrx.transformPoint(frame_id, pts)
        except tf.Exception:
            rospy.logwarn ('MA Exception transforming in TransformPointToFrame()')
            ptsOut = pts
            
        return ptsOut

        
    # Transform a MsgFrameState to the given frame.
    def TransformStateToFrame(self, frame_id, state, doClipToArena=True):
        stateOut = MsgFrameState()
        isInArena = True

        # Get the time.        
        try:
            stamp = self.tfrx.getLatestCommonTime(frame_id, state.header.frame_id)
        except tf.Exception, e:
            rospy.logwarn ('MA Exception1 transforming in TransformStateToFrame(): %s' % e)
        else:
            stateOut.name = state.name
            
            # Transform the pose
            poses = PoseStamped(header=state.header,
                                pose=state.pose) 
            poses.header.stamp = stamp
            if (doClipToArena):
                pts = PointStamped(header=poses.header, point=state.pose.position)
                (pts, isInArena) = self.ClipPtsToArena(pts)
                #(pts, isInReachable) = self.ClipPtsToReachable(pts)
                poses.pose.position = pts.point
            else:
                isInArena = True
                
            poses.header.stamp = stamp
            try:
                posesOut = self.tfrx.transformPose(frame_id, poses)
            except tf.Exception, e:
                rospy.logwarn ('MA Exception2 transforming in TransformStateToFrame(): %s' % e)
                posesOut = poses
            stateOut.header = posesOut.header
            stateOut.pose = posesOut.pose
            
            
            # Transform the linear velocity
            v3s = Vector3Stamped(header=state.header,
                                 vector=state.velocity.linear) 
            v3s.header.stamp = stamp
            try:
                v3sOut = self.tfrx.transformVector3(frame_id, v3s)
            except tf.Exception, e:
                rospy.logwarn ('MA Exception3 transforming in TransformStateToFrame(): %s' % e)
                v3sOut = v3s
            stateOut.velocity.linear = v3sOut.vector
                    
                    
            # Transform the angular velocity
            v3s = Vector3Stamped(header=state.header,
                                 vector=state.velocity.angular) 
            v3s.header.stamp = stamp
            try:
                v3sOut = self.tfrx.transformVector3(frame_id, v3s)
            except tf.Exception, e:
                rospy.logwarn ('MA Exception4 transforming in TransformStateToFrame(): %s' % e)
                v3sOut = v3s
            stateOut.velocity.angular = v3sOut.vector
                    
                    
            # Transform the speed.
            stateOut.speed = state.speed
            
            
        return (stateOut, isInArena)
        

            
    def ClipPtMag (self, pt, magMax):
        magPt = np.linalg.norm([pt.x, pt.y, pt.z])
        if magPt > magMax:
            r = magMax / magPt
        else:
            r = 1.0
            
        return Point(r*pt.x, r*pt.y, r*pt.z)
            
        
    # Clip the point onto the given circle.
    def ClipPtOntoCircle(self, pt, radius):
        r = np.linalg.norm([pt.x, pt.y])
        
        angle = np.arctan2(pt.y, pt.x)
        ptOut = Point(x=radius * np.cos(angle),
                      y=radius * np.sin(angle))
        bClipped = True
        
        return (ptOut, bClipped)
                

    # Clip the tool to the given limit.
    def ClipPtToRadius(self, pt, rLimit):
        r = np.linalg.norm([pt.x, pt.y])
        
        if rLimit < r:
            angle = np.arctan2(pt.y, pt.x)
            ptOut = Point(x=rLimit * np.cos(angle),
                          y=rLimit * np.sin(angle))
            bClipped = True
        else:
            ptOut = pt
            bClipped = False
        
        return (ptOut, bClipped)
                

    # Clip the tool to the hardware limit.
    def ClipPtToReachable(self, pt):
        return self.ClipPtOntoCircle(pt, self.radiusReachable) 
                
                
    def ClipPtsToReachable(self, ptsIn):        
        isInReachable = True
        
        # Transform to Stage frame.
        ptsStage = self.TransformPointToFrame('Stage', ptsIn)

        # Clip to reachable radius.
        (ptClipped, bClipped) = self.ClipPtToReachable(ptsStage.point)
        if bClipped:
            ptsStage.point = ptClipped
            isInReachable = False
    
        # Transform back to original frame.
        ptsOut = self.TransformPointToFrame(ptsIn.header.frame_id, ptsStage)
    
        return (ptsOut, isInReachable)
        

    def ClipPtsToArena(self, ptsIn):        
        isInArena = True
        
        # Transform to Arena frame.
        ptsArena = self.TransformPointToFrame('Arena', ptsIn)

        # Clip to arena radius.
        (ptClipped, bClipped) = self.ClipPtToRadius(ptsArena.point, self.radiusArena)
        if bClipped:
            ptsArena.point = ptClipped
            isInArena = False
    
        # Transform back to original frame.
        ptsOut = self.TransformPointToFrame(ptsIn.header.frame_id, ptsArena)
    
        return (ptsOut, isInArena)
        

    # Forward kinematics:  Get x,y from angle 1
    def Get1xyFrom1 (self, angle):
        x = self.L1 * np.cos(angle)
        y = self.L1 * np.sin(angle)

        return (angle, x, y)
    
    
    # Inverse Kinematics: Get theta 1 from the (x,y) end-effector position,
    # where (x,y) is in the coordinate frame of the workspace center.
    def Get1FromPt (self, pt):
        angle = self.Get1FromXy(pt.x, pt.y)
        
        return angle
        
        
    # Get1FromXy()
    # Inverse Kinematics: Get Angle 1 from the (x,y) end-effector position,
    # where (x,y) is in the  frame of the workspace center.
    #
    def Get1FromXy (self, x, y):
        
        angle = (np.arctan2(y, x) % (2.0 * np.pi)) + self.unwind
        
        if (angle - self.angleInvKinPrev) > np.pi:  # Clockwise across zero.
            angle -= 2.0 * np.pi
            self.unwind -= 2.0 * np.pi
            
        elif (angle - self.angleInvKinPrev) < -np.pi:  # CCW across zero.
            angle += 2.0 * np.pi
            self.unwind += 2.0 * np.pi 
        
        self.angleInvKinPrev = angle
        
        # rospy.logwarn('angle=%0.2f, unwind=%0.2f' % (angle, self.unwind))
        return angle
        
        
    # Return the transform (at angle1) such that (dx,dy) = J*angvel
    def JacobianFwd (self, angle):
        j11 = -self.L1 * np.sin(angle)
        j21 =  self.L1 * np.cos(angle)
    
        return np.array([j11,j21])


    # Return the transform (at angle1) such that angvel = J*(dx,dy)
    def JacobianInv (self, angle1):
        t1 = angle1 % np.pi
        if (np.pi/4.0 < t1 < 3.0*np.pi/4.0):  # Decide whether to use dx or dy for the calculation.
            j11 = -1.0 / (self.L1 * np.sin(angle1))
            j21 = 0
        else:
            j11 = 0
            j21 = 1.0 / (self.L1 * np.cos(angle1))

        return np.array([j11,j21])

        
    def GetStageState_callback(self, reqStageState):
        state = self.GetStageState()
        return state
    
    def GetStageState (self):
        while not self.bInitializedServices:
            rospy.sleep(0.5)
            
        rvStageState = SrvFrameStateResponse()
        rvStageState.state = None
        if (self.jointstate1 is not None):
            if (self.stateVisual is not None):             
                rvStageState.state = self.stateVisual
            else:
                rvStageState.state = self.stateKinematic
        
        return rvStageState.state

        
    # SetStageStateRef_callback()
    #   Updates the target command.
    #
    def SetStageStateRef_callback(self, reqStageState):
        while not self.bInitialized:
            rospy.sleep(0.5)
            
        Jinv = self.JacobianInv(self.angposMech)
            
        (self.stateRef, isInArena) = self.TransformStateToFrame('Stage', reqStageState.state)
        self.angposRef = self.Get1FromPt(self.stateRef.pose.position)
        self.angvelRef = Jinv.dot(np.array([[self.stateRef.velocity.linear.x],[self.stateRef.velocity.linear.y]]))


        rvStageState = SrvFrameStateResponse()
        rvStageState.state = reqStageState.state

        return rvStageState
    

    # Take the target point from a signal generator, and set it as the tool reference.  
    # If the point falls outside the workspace, then clip it to the workspace, and return False.
    def SignalInput_callback (self, srvSignalReq):
        rv = SrvSignalResponse()

        if (self.angposMech is not None):
            Jinv = self.JacobianInv(self.angposMech)
            
            (self.stateRef, isInArena) = self.TransformStateToFrame('Stage', srvSignalReq.state)
            self.angposRef = self.Get1FromPt(self.stateRef.pose.position)
            self.angvelRef = Jinv.dot(np.array([[self.stateRef.velocity.linear.x],[self.stateRef.velocity.linear.y]]))

        rv.success = True #isInArena
        
        return rv


    # Create a vector with direction of ptDir, and magnitude of ptMag.
    def ScaleVecToMag (self, ptDir, ptMag):
        magPtDir = np.linalg.norm([ptDir.x, ptDir.y, ptDir.z])
        magPtMag = np.linalg.norm([ptMag.x, ptMag.y, ptMag.z])
        
        ptNew = Point (x=magPtMag / magPtDir * ptDir.x,
                       y=magPtMag / magPtDir * ptDir.y,
                       z=magPtMag / magPtDir * ptDir.z)
        return ptNew
    

    def VisualState_callback(self, state):
        with self.lock:
            (self.stateVisual, isInArena) = self.TransformStateToFrame('Stage', state, doClipToArena=False)
            self.posVisual = self.Get1FromPt(self.stateVisual.pose.position)
        
        
    def HomeStage_callback(self, reqStageState):
        while not self.bInitialized:
            rospy.sleep(0.1)
            
        with self.lock:
            try:
                self.Park_joint1()
            except (rospy.ServiceException, rospy.exceptions.ROSInterruptException, IOError), e:
                rospy.logwarn ('MA FAILED %s' % e)
                
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
        q1park = 0.0 #self.Get1FromXy(self.xPark, self.yPark)
        q1origin = rospy.get_param('motorarm/angleOffset', 0.0)  # Angle distance of index switch to 0 angle.

        rospy.loginfo ('MA Calibrating: q1origin=%s, q1park=%s' % (q1origin, q1park))
        with self.lock:
            try:
                rv = self.Calibrate_joint1(NEGATIVE, q1origin, q1park, True)  # True=find index switch; False=don't find index switch.
            except Exception:
                rospy.logwarn('FAILED to calibrate joint1')
            else:
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
        if (self.bInitialized):
            with self.lock:
                try:
                    self.jointstate1 = self.GetState_joint1()
                except rospy.ServiceException, e:
                    stSrv = 'srvGetState_' + self.names[0]
                    try:
                        rospy.wait_for_service(stSrv, timeout=5)
                        self.GetState_joint1 = rospy.ServiceProxy(stSrv, SrvJointState, persistent=True)
                    except rospy.ServiceException, e:
                        rospy.logwarn ('MA FAILED to reconnect service %s(): %s' % (stSrv, e))
            
            if (self.jointstate1 is not None):                 
                (angle1, self.stateKinematic.pose.position.x, self.stateKinematic.pose.position.y) = self.Get1xyFrom1(self.jointstate1.position)
                self.angposMech = self.Get1FromPt(self.stateKinematic.pose.position)
                

                # Publish the joint states (for rviz, etc)    
                self.js.header.seq = self.js.header.seq + 1
                self.js.header.stamp = self.time
                self.js.position = [angle1]
                self.pubJointState.publish(self.js)
    
                qEE = tf.transformations.quaternion_from_euler(0.0, 0.0, angle1)
                state = MsgFrameState()
                state.header.stamp = self.time
                state.header.frame_id = 'Stage'
                state.pose.position = self.stateKinematic.pose.position
                state.pose.orientation.x = qEE[0]
                state.pose.orientation.y = qEE[1]
                state.pose.orientation.z = qEE[2]
                state.pose.orientation.w = qEE[3]


                
                # Publish the link transforms.
                self.tfbx.sendTransform((0.0, 0.0, 0.0),
                                        tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                                        state.header.stamp,
                                        'link0',  # child
                                        'Stage'  # parent
                                        )

                self.tfbx.sendTransform((0.0, 0.0, 0.0),
                                        qEE,
                                        state.header.stamp,
                                        'link1',  # child
                                        'link0'  # parent
                                        )
                
                # Frame EndEffector (i.e. the driver magnet)
                self.tfbx.sendTransform((state.pose.position.x, state.pose.position.y, state.pose.position.z),
                                        qEE,
                                        state.header.stamp,
                                        'EndEffector',  # child
                                        'Stage'  # parent
                                        )
                
                
                # Frame Target
                if (self.stateRef is not None):
                    markerTarget = Marker(header=self.stateRef.header,
                                          ns='target',
                                          id=2,
                                          type=Marker.SPHERE,
                                          action=0,
                                          pose=Pose(position=self.stateRef.pose.position),
                                          scale=Vector3(x=3.0,
                                                        y=3.0,
                                                        z=3.0),
                                          color=ColorRGBA(a=0.5,
                                                          r=1.0,
                                                          g=0.1,
                                                          b=0.1),
                                          lifetime=rospy.Duration(1.0))
                    self.pubMarker.publish(markerTarget)

                if (self.stateVisual is not None):
                    markerToolOffset = Marker(header=state.header,
                                              ns='tooloffset',
                                              id=3,
                                              type=Marker.ARROW,
                                              action=0,
                                              scale=Vector3(x=0.1,  # Shaft diameter
                                                            y=0.2,  # Head diameter
                                                            z=0.0),
                                              color=ColorRGBA(a=0.8,
                                                              r=1.0,
                                                              g=1.0,
                                                              b=1.0),
                                              lifetime=rospy.Duration(1.0),
                                              points=[state.pose.position,
                                                      self.stateVisual.pose.position])
                    self.pubMarker.publish(markerToolOffset)



    # UpdateMotorCommandFromTarget()
    #   Updates the motor command with the current target.
    #
    def UpdateMotorCommandFromTarget(self):
        if self.stateRef is not None:
            # PID Gains & Parameters.
            if (self.bTune):
                self.kP      = rospy.get_param('motorarm/kP', 1.0)
                self.kI      = rospy.get_param('motorarm/kI', 0.0)
                self.kD      = rospy.get_param('motorarm/kD', 0.0)
                self.maxI    = rospy.get_param('motorarm/maxI', 40.0)
                self.kWindup = rospy.get_param('motorarm/kWindup', 0.0)

                self.kPv      = rospy.get_param('motorarm/kPv', 0.0)
                self.kIv      = rospy.get_param('motorarm/kIv', 0.0)
                self.kDv      = rospy.get_param('motorarm/kDv', 0.0)
                self.maxIv    = rospy.get_param('motorarm/maxIv', 40.0)
                self.kWindupv = rospy.get_param('motorarm/kWindupv', 0.0)

                self.kAll    = rospy.get_param('motorarm/kAll', 1.0)
                self.speedLinearMax = rospy.get_param('motorarm/speed_max', 200.0)
            
                self.kP *= self.kAll
                self.kI *= self.kAll
                self.kD *= self.kAll

                a = rospy.get_param('/a', 0.05) # This is the filter constant for the derivative term.
            else:
                a = 0.05
            
            # The previous error.
            self.angposPErrorPrev = self.angposPError
            self.angvelPErrorPrev = self.angvelPError
            #self.statePErrorPrev = copy.deepcopy(self.statePError)
            
            # Use either visual servoing or not, as the case may be, depending on if we've received a visual state callback.
            if (self.stateVisual is not None):
                stateActuator = self.stateVisual
            else:
                stateActuator = self.stateKinematic


            if (self.stateRef is not None):
                Jfwd = self.JacobianFwd(self.angposMech)
                Jinv = self.JacobianInv(self.angposMech)

                # Convert 2D (x,y) position into a 1D x position.
                angposActuator = self.Get1FromPt(stateActuator.pose.position)
                angvelActuator = Jinv.dot(np.array([[stateActuator.velocity.linear.x],[stateActuator.velocity.linear.y]]))
                
                # Error terms.
                self.angposPError = self.angposRef - angposActuator
                self.angvelPError = self.angvelRef - angvelActuator
                
                self.angposIError += self.angposPError
                self.angvelIError += self.angvelPError
                 
                self.angposDError = (1-a)*self.angposDError + a*(self.angposPError - self.angposPErrorPrev)
                self.angvelDError = self.angvelPError - self.angvelPErrorPrev
                
#                 self.statePError.pose.position.x = self.stateRef.pose.position.x - stateActuator.pose.position.x
#                 self.statePError.pose.position.y = self.stateRef.pose.position.y - stateActuator.pose.position.y
#                 self.statePError.pose.position.z = self.stateRef.pose.position.z - stateActuator.pose.position.z
#                 self.statePError.velocity.linear.x = self.stateRef.velocity.linear.x - stateActuator.velocity.linear.x
#                 self.statePError.velocity.linear.y = self.stateRef.velocity.linear.y - stateActuator.velocity.linear.y
#                 self.statePError.velocity.linear.z = self.stateRef.velocity.linear.z - stateActuator.velocity.linear.z
# 
#                 self.stateIError.pose.position.x += self.statePError.pose.position.x
#                 self.stateIError.pose.position.y += self.statePError.pose.position.y
#                 self.stateIError.pose.position.z += self.statePError.pose.position.z
#                 self.stateIError.velocity.linear.x += self.statePError.velocity.linear.x
#                 self.stateIError.velocity.linear.y += self.statePError.velocity.linear.y
#                 self.stateIError.velocity.linear.z += self.statePError.velocity.linear.z
#                 
#                 self.stateDError.pose.position.x = (1-a)*self.stateDError.pose.position.x + a*(self.statePError.pose.position.x - self.statePErrorPrev.pose.position.x) # filtered.
#                 self.stateDError.pose.position.y = (1-a)*self.stateDError.pose.position.y + a*(self.statePError.pose.position.y - self.statePErrorPrev.pose.position.y)
#                 self.stateDError.pose.position.z = (1-a)*self.stateDError.pose.position.z + a*(self.statePError.pose.position.z - self.statePErrorPrev.pose.position.z)
#                 self.stateDError.velocity.linear.x = self.statePError.velocity.linear.x - self.statePErrorPrev.velocity.linear.x
#                 self.stateDError.velocity.linear.y = self.statePError.velocity.linear.y - self.statePErrorPrev.velocity.linear.y
#                 self.stateDError.velocity.linear.z = self.statePError.velocity.linear.z - self.statePErrorPrev.velocity.linear.z


                # PID control of the position error.
                angposPID = self.kP*self.angposPError + self.kI*self.angposIError + self.kD*self.angposDError
#                 self.statePID.pose.position = Point(x=self.kP*self.statePError.pose.position.x + self.kI*self.stateIError.pose.position.x + self.kD*self.stateDError.pose.position.x,
#                                                     y=self.kP*self.statePError.pose.position.y + self.kI*self.stateIError.pose.position.y + self.kD*self.stateDError.pose.position.y,
#                                                     z=self.kP*self.statePError.pose.position.z + self.kI*self.stateIError.pose.position.z + self.kD*self.stateDError.pose.position.z)
                                    
                # Clip the integral error to the reachable workspace (prevents accumulating error due to singularity in radial direction).
#                 (ptClipped, bClipped) = self.ClipPtToReachable(Point(x=self.stateKinematic.pose.position.x + self.kI * self.stateIError.pose.position.x,
#                                                                      y=self.stateKinematic.pose.position.y + self.kI * self.stateIError.pose.position.y))
#                 if self.kI != 0:
#                     self.stateIError.pose.position.x = (ptClipped.x - self.stateKinematic.pose.position.x) / self.kI
#                     self.stateIError.pose.position.y = (ptClipped.y - self.stateKinematic.pose.position.y) / self.kI
#                 else:
#                     self.stateIError.pose.position.x = (ptClipped.x - self.stateKinematic.pose.position.x)
#                     self.stateIError.pose.position.y = (ptClipped.y - self.stateKinematic.pose.position.y)
            

                # Anti-windup
                clippedIError = np.clip(self.angposIError, -self.maxI, self.maxI)
                excessIError = self.angposIError - clippedIError
                self.angposIError -= self.kWindup * excessIError
#                 ptEeIErrorClipped = self.ClipPtMag (self.stateIError.pose.position, self.maxI)
#                 ptExcessI = Point(self.stateIError.pose.position.x - ptEeIErrorClipped.x,
#                                   self.stateIError.pose.position.y - ptEeIErrorClipped.y,
#                                   self.stateIError.pose.position.z - ptEeIErrorClipped.z)
#                 self.stateIError.pose.position.x -= self.kWindup * ptExcessI.x
#                 self.stateIError.pose.position.y -= self.kWindup * ptExcessI.y
#                 self.stateIError.pose.position.z -= self.kWindup * ptExcessI.z

            

                # PID control of the velocity error.
                angvelPID = self.kPv*self.angvelPError + self.kIv*self.angvelIError + self.kDv*self.angvelDError
#                 self.statePID.velocity.linear = Point(x=self.kPv*self.statePError.velocity.linear.x + self.kIv*self.stateIError.velocity.linear.x + self.kDv*self.stateDError.velocity.linear.x,
#                                                       y=self.kPv*self.statePError.velocity.linear.y + self.kIv*self.stateIError.velocity.linear.y + self.kDv*self.stateDError.velocity.linear.y,
#                                                       z=self.kPv*self.statePError.velocity.linear.z + self.kIv*self.stateIError.velocity.linear.z + self.kDv*self.stateDError.velocity.linear.z)
                                    
                # Anti-windup
                clippedIError = np.clip(self.angvelIError, -self.maxIv, self.maxIv)
                excessIError = self.angvelIError - clippedIError
                self.angvelIError -= self.kWindup * excessIError
#                 ptEeIErrorClipped = self.ClipPtMag (self.stateIError.velocity.linear, self.maxIv)
#                 ptExcessI = Point(self.stateIError.velocity.linear.x - ptEeIErrorClipped.x,
#                                   self.stateIError.velocity.linear.y - ptEeIErrorClipped.y,
#                                   self.stateIError.velocity.linear.z - ptEeIErrorClipped.z)
#                 self.stateIError.velocity.linear.x -= self.kWindup * ptExcessI.x
#                 self.stateIError.velocity.linear.y -= self.kWindup * ptExcessI.y
#                 self.stateIError.velocity.linear.z -= self.kWindup * ptExcessI.z
                
            
                # Compute the velocity command.
                angvel = self.angvelRef + angvelPID + angposPID
#                 xDot = (self.stateRef.velocity.linear.x + self.statePID.velocity.linear.x) + self.statePID.pose.position.x
#                 yDot = (self.stateRef.velocity.linear.y + self.statePID.velocity.linear.y) + self.statePID.pose.position.y


                # Pull the mechanical position back under the visual position.
                angvel += angposActuator - self.angposMech
#                 xDot += (stateActuator.pose.position.x - self.stateKinematic.pose.position.x)
#                 yDot += (stateActuator.pose.position.y - self.stateKinematic.pose.position.y)

                # Clip to max speed.
                if (self.stateRef.speed != 0.0):  # 0.0 means the speed is unspecified.
                    speedMax = min(self.stateRef.speed, self.speedLinearMax)
                else:
                    speedMax = self.speedLinearMax
                    
                angspeedMax = speedMax / self.L1
                angvel = np.clip(angvel,-angspeedMax, angspeedMax)
                
#                 pt = self.ClipPtMag(Point(x=xDot,y=yDot), speedMax)
#                 xDot = pt.x
#                 yDot = pt.y
                
                
                # Clip to stay in workspace.
#                 pts = PointStamped(header=Header(stamp=stateActuator.header.stamp,
#                                                  frame_id='Stage'),
#                                    point=Point(x = xDot + self.stateKinematic.pose.position.x,
#                                                y = yDot + self.stateKinematic.pose.position.y))
#                 (ptsArena, isInArena) = (pts,True)#self.ClipPtsToArena(pts) causes position error surges at certain points around the circle.
#                 xDot = ptsArena.point.x - self.stateKinematic.pose.position.x
#                 yDot = ptsArena.point.y - self.stateKinematic.pose.position.y
                
                
                # Convert to motor coordinates.
                theta1Dot = angvel 
                xyDot = Jfwd * angvel
                xDot = xyDot[0]
                yDot = xyDot[1]
#                 theta1Dot = Jinv.dot(np.array([[xDot],[yDot]])) 

#                 rospy.logwarn('angle1Mech=%0.2f, theta1Dot=%s, Perror=%s, Verror=%s, x,yDot=%s' % (angle1Mech, 
#                                                                                         theta1Dot, 
#                                                                                         (self.statePError.pose.position.x,self.statePError.pose.position.y), 
#                                                                                         (self.statePError.velocity.linear.x,self.statePError.velocity.linear.y), 
#                                                                                         (xDot, yDot)))
                # Display the velocity vector in rviz.
                ptBase = self.stateKinematic.pose.position #stateActuator.pose.position
                ptEnd = Point(x = ptBase.x + xDot,
                              y = ptBase.y + yDot,
                              z = ptBase.z + 0
                              ) 
                markerVelocity = Marker(header=Header(stamp=self.time,
                                                    frame_id='Stage'),
                                      ns='velocityCommand',
                                      id=3,
                                      type=Marker.ARROW,
                                      action=0,
                                      scale=Vector3(x=0.1,  # Shaft diameter
                                                    y=0.2,  # Head diameter
                                                    z=0.0),
                                      color=ColorRGBA(a=0.9,
                                                      r=0.5,
                                                      g=0.5,
                                                      b=1.0),
                                      lifetime=rospy.Duration(1.0),
                                      points=[ptBase, ptEnd])
                self.pubMarker.publish(markerVelocity)


                if (self.bTune):
                    # Display the command vector in rviz.
#                     ptBase = stateActuator.pose.position
#                     ptEnd = Point(x = ptBase.x + self.statePID.pose.position.x,
#                                   y = ptBase.y + self.statePID.pose.position.y,
#                                   z = ptBase.z + self.statePID.pose.position.z
#                                   ) 
#                     markerCommand = Marker(header=Header(stamp=self.time,
#                                                         frame_id='Stage'),
#                                           ns='pidPosition',
#                                           id=2,
#                                           type=Marker.ARROW,
#                                           action=0,
#                                           scale=Vector3(x=0.1,  # Shaft diameter
#                                                         y=0.2,  # Head diameter
#                                                         z=0.0),
#                                           color=ColorRGBA(a=0.8,
#                                                           r=1.0,
#                                                           g=1.0,
#                                                           b=1.0),
#                                           lifetime=rospy.Duration(1.0),
#                                           points=[ptBase, ptEnd])
#                     self.pubMarker.publish(markerCommand)
#             
#                     # Display P,I,D vectors in rviz.
#                     ptBase = stateActuator.pose.position #self.stateKinematic.pose.position
#                     ptEnd = Point(x = ptBase.x + self.kP*self.statePError.pose.position.x,
#                                   y = ptBase.y + self.kP*self.statePError.pose.position.y)
#                     markerCommand= Marker(header=Header(stamp = self.time, frame_id='Stage'),
#                                           ns='P',
#                                           id=5, type=Marker.ARROW, action=0,
#                                           scale=Vector3(x=0.1, y=0.2, z=0.0),
#                                           color=ColorRGBA(a=0.9, r=1.0, g=0.0, b=0.0),
#                                           lifetime=rospy.Duration(1.0), points=[ptBase, ptEnd])
#                     self.pubMarker.publish(markerCommand)
#                 
#                     ptEnd = Point(x = ptBase.x + self.kI*self.stateIError.pose.position.x,
#                                   y = ptBase.y + self.kI*self.stateIError.pose.position.y)
#                     markerCommand= Marker(header=Header(stamp = self.time, frame_id='Stage'),
#                                           ns='I',
#                                           id=6, type=Marker.ARROW, action=0,
#                                           scale=Vector3(x=0.1, y=0.2, z=0.0),
#                                           color=ColorRGBA(a=0.9, r=0.0, g=1.0, b=0.0),
#                                           lifetime=rospy.Duration(1.0), points=[ptBase, ptEnd])
#                     self.pubMarker.publish(markerCommand)
#                     ptEnd = Point(x = ptBase.x + self.kD*self.stateDError.pose.position.x,
#                                   y = ptBase.y + self.kD*self.stateDError.pose.position.y)
#                     markerCommand= Marker(header=Header(stamp = self.time, frame_id='Stage'),
#                                           ns='D',
#                                           id=7, type=Marker.ARROW, action=0,
#                                           scale=Vector3(x=0.1, y=0.2, z=0.0),
#                                           color=ColorRGBA(a=0.9, r=0.0, g=0.0, b=1.0),
#                                           lifetime=rospy.Duration(1.0), points=[ptBase, ptEnd])
#                     self.pubMarker.publish(markerCommand)


                    # Print the PID component values.
                    magP = self.kP * self.angposPError
                    magI = self.kI * self.angposIError
                    magD = self.kD * self.angposDError
                    magPID = angposPID
                    
                    magPv = self.kPv * self.angvelPError
                    magIv = self.kIv * self.angvelIError
                    magDv = self.kDv * self.angvelDError
                    magPIDv = angvelPID
#                     magP = self.kP * np.linalg.norm([self.statePError.pose.position.x, self.statePError.pose.position.y])
#                     magI = self.kI * np.linalg.norm([self.stateIError.pose.position.x, self.stateIError.pose.position.y])
#                     magD = self.kD * np.linalg.norm([self.stateDError.pose.position.x, self.stateDError.pose.position.y])
#                     magPID = np.linalg.norm([self.statePID.pose.position.x, self.statePID.pose.position.y])
#                     
#                     magPv = self.kPv * np.linalg.norm([self.statePError.velocity.linear.x, self.statePError.velocity.linear.y])
#                     magIv = self.kIv * np.linalg.norm([self.stateIError.velocity.linear.x, self.stateIError.velocity.linear.y])
#                     magDv = self.kDv * np.linalg.norm([self.stateDError.velocity.linear.x, self.stateDError.velocity.linear.y])
#                     magPIDv = np.linalg.norm([self.statePID.velocity.linear.x, self.statePID.velocity.linear.y])
                    
                    rospy.logwarn('[P,I,D]=[% 5.2f,% 5.2f,% 5.2f]=% 5.2f, [% 5.2f,% 5.2f,% 5.2f]=% 5.2f, v=% 5.2f' % (magP,magI,magD, magPID, magPv,magIv,magDv, magPIDv, theta1Dot))
                    
            
            else:
                theta1Dot = 0.0
                theta2Dot = 0.0
                
    
            if (self.jointstate1 is not None):
                with self.lock:
                    try:
                        self.SetVelocity_joint1(Header(frame_id=self.names[0]), None, theta1Dot)
                    except rospy.ServiceException, e:
                        stSrv = 'srvSetVelocity_' + self.names[0]
                        try:
                            rospy.wait_for_service(stSrv, timeout=5)
                            self.SetVelocity_joint1 = rospy.ServiceProxy(stSrv, SrvJointState, persistent=True)
                        except rospy.ServiceException, e:
                            rospy.logwarn ('MA FAILED to reconnect service %s(): %s' % (stSrv, e))

        
    def OnShutdown_callback(self):
        self.Calibrate_callback(None)
        rospy.logwarn('MA Closed MotorArm device.')
        

        
    def Main(self):
        self.AttachServices()
        self.Calibrate_callback(None)
        self.bInitialized = True

        # Process messages forever.
        rosrate = rospy.Rate(1 / self.T)
        while (not rospy.is_shutdown()) and (self.commandExperiment != 'exit_now'):
            if (self.commandExperiment != 'pause_now'):
                self.time = rospy.Time.now()
                self.dt = self.time - self.timePrev
                self.timePrev = self.time
                
                self.SendTransforms()
                self.UpdateMotorCommandFromTarget()
            else:
                angle1Mech   = self.Get1FromPt(self.stateKinematic.pose.position)
                with self.lock:
                    try:
                        self.SetVelocity_joint1(Header(frame_id=self.names[0]), angle1Mech, 0.0)
                    except (rospy.ServiceException, rospy.exceptions.ROSInterruptException, IOError), e:
                        rospy.logwarn ("MA Exception:  %s" % e)
            rosrate.sleep()

            
            
        # Shutdown all the services we offered.
        for key in self.services:
            self.services[key].shutdown()
            
    

if __name__ == '__main__':
    try:
        motorarm = MotorArm()
        motorarm.Main()
    except rospy.exceptions.ROSInterruptException: 
        pass
    
    
    
