#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('fivebar')
import rospy
import copy
import numpy as N
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


# The following settings are the best I've found for the BAI controller with the Fivebar mechanism.
# Unlisted parameters are set to their default.
# Jan 12, 2012
#
#PRM:#     Parameter                  Value         Default 
#----------------------------------------------------------------
#PRM:0:    KP                         2000000       750000
#PRM:1:    KI                         0             35000
#PRM:2:    KPOS                       10000         15000
#PRM:8:    RMS current limt           40            20
#PRM:11:   integral clamp             500           5000
#PRM:12:   position error trap        400           100
#PRM:20:   operating mode             5             4
#PRM:26:   lowpass filter             1             0
#PRM:90:   baud rate                  38400         9600
#PRM:95:   daisy chain                1             0
#PRM:101:  fault output               1             0
#PRM:202:  filter cutoff              100.0         500.0
#PRM:204:  autotune distance          500.0         32000.0
#PRM:205:  autotune bandwidth         7.0           20.0




class Fivebar:

    def __init__(self):
        self.initialized = False
        self.initializedServices = False
        self.lock = threading.Lock()
        rospy.loginfo("Opening Fivebar device...")
        rospy.init_node('Fivebar')
        rospy.loginfo ("Fivebar name=%s", __name__)
        rospy.on_shutdown(self.OnShutdown_callback)

        # Link lengths (millimeters)
        self.linkwidth = rospy.get_param('fivebar/linkwidth', 0.1)
        self.L0 = rospy.get_param('fivebar/L0', 1.0) #4.24*2 # Distance between motor shafts
        self.L1 = rospy.get_param('fivebar/L1', 1.0)  # Link 1
        self.L2 = rospy.get_param('fivebar/L2', 1.0)  # Link 2
        self.L3 = rospy.get_param('fivebar/L3', 1.0) # Link 3
        self.L4 = rospy.get_param('fivebar/L4', 1.0) # Link 4

        self.T = rospy.get_param('fivebar/T', 1/50)  # Nominal motor update period, i.e. sample rate.

        # PID Gains & Parameters.
        self.kP = rospy.get_param('fivebar/kP', 0.1)
        self.kI = rospy.get_param('fivebar/kI', 0.0)
        self.kD = rospy.get_param('fivebar/kD', 0.0)
        self.maxI = rospy.get_param('fivebar/maxI', 40.0)
        self.kWindup = rospy.get_param('fivebar/kWindup', 0.0)

        self.kPv = rospy.get_param('fivebar/kPv', 0.0)
        self.kIv = rospy.get_param('fivebar/kIv', 0.0)
        self.kDv = rospy.get_param('fivebar/kDv', 0.0)
        self.maxIv = rospy.get_param('fivebar/maxIv', 40.0)
        self.kWindupv = rospy.get_param('fivebar/kWindupv', 0.0)

        
        self.bTune = rospy.get_param('/tune', False)

        # Parking spot (millimeters)
        self.xPark = 0.0
        self.yPark = 0.0
        threading
        FUDGEX = 0.0
        FUDGEY = -20.0  # Fudge factor to center things better in the reachable workspace.
        FUDGEQ1 = -0.15052268 # These are the angles that move down 20mm at center.
        FUDGEQ2 =  0.15052268
        
        alpha = N.arctan(self.linkwidth/self.L1) # Smallest angle two links can collapse to.
        r = N.sqrt(self.L1**2.0 + self.L3**2.0 - 2.0*self.L1*self.L3*N.cos(alpha)) # Distance from joint 1 to EE when most open.
        beta = (N.pi - N.arcsin(self.L3*N.sin(alpha)/r)) % (2.0*N.pi) # Angle between link1 and EE when most open.
        
        # Joint angles CCW from East (i.e. "east" coordinates).
        self.q1MinE    = (N.arccos((self.L0/2.0)/(self.L3+self.L1)) % (2.0*N.pi))
        self.q1MaxE    = (N.arccos((self.L0/2.0)/r) + beta) % (2.0*N.pi)
        self.q1CenterE = (self.q1MaxE+self.q1MinE)/2.0 + FUDGEQ1
        self.q2MinE    = (N.pi - N.arccos((self.L0/2.0)/r) - beta) 
        self.q2MaxE    = (N.pi - N.arccos((self.L0/2.0)/(self.L3+self.L1))) % (2.0*N.pi)
        self.q2CenterE = (self.q2MaxE+self.q2MinE)/2.0 + FUDGEQ2
        
        # Joint angles from farthest reachable point to center of workspace (i.e. "index" coordinates)
        self.q1CenterI   = self.q1CenterE - self.q1MinE
        self.q2CenterI   = self.q2CenterE - self.q2MaxE
        
        # Joint angles at center of workspace (i.e. "zero" coordinates)
        q1MaxZ      = self.q1MaxE - self.q1CenterE
        q1MinZ      = self.q1MinE - self.q1CenterE
        q2MaxZ      = self.q2MaxE - self.q2CenterE
        q2MinZ      = self.q2MinE - self.q2CenterE
        
        # Center of workspace (x,y)
        self.xCenter    = self.L0/2.0 + FUDGEX # Between motor shafts
        TMPx            = N.cos(N.pi - self.q1CenterE) * self.L1 + self.L0/2.0
        TMPy            = N.sin(N.pi - self.q1CenterE) * self.L1
        self.yCenter    = N.sqrt(self.L3**2.0 - TMPx**2.0) + TMPy + FUDGEY
        self.radiusReachable = 0.98*(self.L1+self.L3-r)/2.0 # Really it's about 90.5.  Limits in xy are at roughly (64,64), (-64,64), (-64,-64), (64,-64).
        self.radiusArena = rospy.get_param('arena/radius_inner', 25.4)



        self.names = ['joint1','joint2','joint3','joint4']
        self.jointstate1 = None
        self.jointstate2 = None
        self.js = JointState()
        self.js.header.seq = 0
        self.js.header.stamp = rospy.Time.now()
        self.js.header.frame_id = 'motor'
        self.js.name = self.names
        #self.js.velocity = [0.0, 0.0, 0.0, 0.0]
        
        
        # Publish & Subscribe
        self.services = {}
        self.services['set_stage_state_ref'] = rospy.Service('set_stage_state_ref', SrvFrameState, self.SetStageStateRef_callback)
        self.services['get_stage_state'] = rospy.Service('get_stage_state',         SrvFrameState, self.GetStageState_callback)
        self.services['home_stage']      = rospy.Service('home_stage',              SrvFrameState, self.HomeStage_callback)
        self.services['calibrate_stage'] = rospy.Service('calibrate_stage',         SrvFrameState, self.Calibrate_callback)
        self.services['signal_input']    = rospy.Service('signal_input',            SrvSignal,     self.SignalInput_callback) # For receiving input from a signal generator.


        # Command messages.
        self.command = 'continue'
        self.command_list = ['continue','exit_now']
        self.subCommand = rospy.Subscriber('experiment/command', String, self.Command_callback)

        self.subVisualState = rospy.Subscriber('visual_state', MsgFrameState, self.VisualState_callback)
        self.pubJointState = rospy.Publisher('joint_states', JointState)
        self.pubMarker = rospy.Publisher('visualization_marker', Marker)
        
        self.tfrx = tf.TransformListener()
        self.tfbx = tf.TransformBroadcaster()

        self.stateRef = None
        self.stateVisual = None
        self.stateMech = MsgFrameState(pose=Pose(position=Point(0,0,0)))

        self.statePError = MsgFrameState()
        self.stateIError = MsgFrameState()
        self.stateDError = MsgFrameState()
        
        self.statePID = MsgFrameState()


        self.unwind = 0.0
        self.angleInvKinPrev = 0.0
        self.speedLinearMax = rospy.get_param('fivebar/speed_max', 200.0)
        
        self.timePrev = rospy.Time.now()
        self.time = rospy.Time.now()
        
        self.anglePrev = 0.0
        self.angleNext = 0.0
        self.speedNextFiltered = None
        self.dAFiltered = 0.0
        self.dA2Filtered = 0.0
        self.dASum = 0.0
        self.xMin = 99999
        self.xMax = 0
        self.yMin = 99999
        self.yMax = 0
        
        self.request = SrvFrameStateRequest()
        self.request.state.header.frame_id = 'Stage'
        self.request.state.pose.position.x = 0.0
        self.request.state.pose.position.y = 0.0
        self.request.state.pose.position.z = 0.0
        
        self.n = 0


    def Command_callback(self, msgString):
        rospy.logwarn('Command_callback()')
        self.command = msgString.data
            

    def LoadServices(self):
        # Joint1 services.
        stSrv = 'srvCalibrate_' + self.names[0]
        try:
            rospy.wait_for_service(stSrv)
            self.Calibrate_joint1 = rospy.ServiceProxy(stSrv, SrvCalibrate)
        except (rospy.ServiceException, IOError), e:
            rospy.logwarn ("5B FAILED %s: %s"%(stSrv,e))


        stSrv = 'srvGetState_' + self.names[0]
        try:
            rospy.wait_for_service(stSrv)
            self.GetState_joint1 = rospy.ServiceProxy(stSrv, SrvJointState, persistent=True)
        except (rospy.ServiceException, IOError), e:
            rospy.logwarn ("5B FAILED %s: %s"%(stSrv,e))

        stSrv = 'srvSetVelocity_' + self.names[0]
        try:
            rospy.wait_for_service(stSrv)
            self.SetVelocity_joint1 = rospy.ServiceProxy(stSrv, SrvJointState, persistent=True)
        except (rospy.ServiceException, IOError), e:
            rospy.logwarn ('5B FAILED %s: %s' % (stSrv, e))

        stSrv = 'srvPark_' + self.names[0]
        try:
            rospy.wait_for_service(stSrv)
            self.Park_joint1 = rospy.ServiceProxy(stSrv, SrvJointState)
        except (rospy.ServiceException, IOError), e:
            rospy.logwarn ("5B FAILED %s: %s"%(stSrv,e))

        # Joint2 services.
        stSrv = 'srvCalibrate_'+self.names[1]
        try:
            rospy.wait_for_service(stSrv)
            self.Calibrate_joint2 = rospy.ServiceProxy(stSrv, SrvCalibrate)
        except (rospy.ServiceException, IOError), e:
            rospy.logwarn ("5B FAILED %s: %s"%(stSrv,e))

        stSrv = 'srvGetState_'+self.names[1]
        try:
            rospy.wait_for_service(stSrv)
            self.GetState_joint2 = rospy.ServiceProxy(stSrv, SrvJointState, persistent=True)
        except (rospy.ServiceException, IOError), e:
            rospy.logwarn ("5B FAILED %s: %s"%(stSrv,e))

        stSrv = 'srvSetVelocity_' + self.names[1]
        try:
            rospy.wait_for_service(stSrv)
            self.SetVelocity_joint2 = rospy.ServiceProxy(stSrv, SrvJointState, persistent=True)
        except (rospy.ServiceException, IOError), e:
            rospy.logwarn ('5B FAILED %s: %s' % (stSrv, e))

        stSrv = 'srvPark_'+self.names[1]
        try:
            rospy.wait_for_service(stSrv)
            self.Park_joint2 = rospy.ServiceProxy(stSrv, SrvJointState)
        except (rospy.ServiceException, IOError), e:
            rospy.logwarn ("5B FAILED %s: %s"%(stSrv,e))

        self.initializedServices = True
        

    # Transform a PointStamped to the given frame.
    def TransformPointToFrame(self, frame_id, pts):
        try:
            pts.header.stamp = self.tfrx.getLatestCommonTime(frame_id, pts.header.frame_id)
            ptsOut = self.tfrx.transformPoint(frame_id, pts)
        except tf.Exception:
            rospy.logwarn ('5B Exception transforming in TransformPointToFrame()')
            ptsOut = pts
            
        return ptsOut

        
    # Transform a MsgFrameState to the given frame.
    def TransformStateToFrame(self, frame_id, state, doClipToArena=True):
        if (state.header.frame_id != ''):
            stateOut = MsgFrameState()
            isInArena = True
    
            # Get the time.        
            try:
                stamp = self.tfrx.getLatestCommonTime(frame_id, state.header.frame_id)
            except tf.Exception, e:
                rospy.logwarn ('5B Exception1 transforming in TransformStateToFrame(): %s' % e)
                rospy.logwarn('state=%s' % state)
            else:
                stateOut.name = state.name
                
                # Transform the pose
                poses = PoseStamped(header=state.header,
                                    pose=state.pose) 
                poses.header.stamp = stamp
                if (doClipToArena):
                    pts = PointStamped(header=poses.header, point=state.pose.position)
                    (pts, isInArena) = self.ClipPtsToArena(pts)
                    poses.pose.position = pts.point
                else:
                    isInArena = True
                    
                poses.header.stamp = stamp
                try:
                    posesOut = self.tfrx.transformPose(frame_id, poses)
                except tf.Exception, e:
                    rospy.logwarn ('5B Exception2 transforming in TransformStateToFrame(): %s' % e)
                    rospy.logwarn('poses=%s' % poses)
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
                    rospy.logwarn ('5B Exception3 transforming in TransformStateToFrame(): %s' % e)
                    rospy.logwarn('v3s=%s' % v3s)
                    v3sOut = v3s
                stateOut.velocity.linear = v3sOut.vector
                        
                        
                # Transform the angular velocity
                v3s = Vector3Stamped(header=state.header,
                                     vector=state.velocity.angular) 
                v3s.header.stamp = stamp
                try:
                    v3sOut = self.tfrx.transformVector3(frame_id, v3s)
                except tf.Exception, e:
                    rospy.logwarn ('5B Exception4 transforming in TransformStateToFrame(): %s' % e)
                    rospy.logwarn('v3s=%s' % v3s)
                    v3sOut = v3s
                stateOut.velocity.angular = v3sOut.vector
                        
                        
                # Transform the speed.
                stateOut.speed = state.speed
        else:
            (stateOut, isInArena) = (None, False)

            
            
        return (stateOut, isInArena)
        

            
    def ClipPtMag (self, pt, magMax):
        magPt = N.linalg.norm([pt.x, pt.y, pt.z])
        if magPt > magMax:
            r = magMax / magPt
        else:
            r = 1.0
            
        return Point(r*pt.x, r*pt.y, r*pt.z)
            
        
    # Clip the tool to the given limit.
    def ClipPtToRadius(self, pt, rLimit):
        r = N.linalg.norm([pt.x, pt.y])
        
        if rLimit < r:
            angle = N.arctan2(pt.y, pt.x)
            ptOut = Point(x=rLimit * N.cos(angle),
                          y=rLimit * N.sin(angle))
            bClipped = True
        else:
            ptOut = pt
            bClipped = False
        
        return (ptOut, bClipped)
                
                
    # Clip the tool to the hardware limit.
    def ClipPtToReachable(self, pt):
        r = N.sqrt(pt.x*pt.x + pt.y*pt.y)
        rLimit = self.radiusReachable #N.min([self.radiusArena, self.radiusReachable])
        
        if rLimit < r:
            angle = N.arctan2(pt.y, pt.x)
            ptOut = Point(x = rLimit * N.cos(angle),
                          y = rLimit * N.sin(angle))
            bClipped = True
        else:
            ptOut = pt
            bClipped = False
        
        return (ptOut, bClipped)
                
                
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
        

    # Forward kinematics; close the kinematics chain:  Get thetas 1,2,3,4 from thetas 1,2
    def Get1234xyFrom12 (self, angle1, angle2):
        # Rotate the frames from centered to east-pointing. 
        angle1 = angle1 + self.q1CenterE
        angle2 = angle2 + self.q2CenterE
        
        # Set the crossover points at 0/2pi and pi/-pi
        angle1 = ( angle1       % (2.0*N.pi))
        angle2 = ((angle2+N.pi) % (2.0*N.pi)) - N.pi
        
        s1 = N.sin(angle1)
        c1 = N.cos(angle1)
        
        s2 = N.sin(angle2)
        c2 = N.cos(angle2)
        
        j3x =       self.L1 * c1
        j3y =       self.L1 * s1
        j4x =  self.L0 + self.L2 * c2
        j4y =       self.L2 * s2
        cx = j4x - j3x
        cy = N.abs(j3y-j4y)
        Lc2 = cx*cx+cy*cy
        Lc = N.sqrt(Lc2)

        # Intersection of circles.
        a = (self.L3*self.L3-self.L4*self.L4+Lc*Lc)/(2.0*Lc)
        h = N.sqrt(self.L3*self.L3-a*a)
        j2 = N.array([self.L0,0.0])
        j3 = N.array([j3x,j3y])
        j4 = N.array([j4x,j4y])
        p2 = j3 + (a/Lc)*(j4-j3)
        xEa = p2[0]+(h/Lc)*(j4[1]-j3[1])
        xEb = p2[0]-(h/Lc)*(j4[1]-j3[1])
        yEa = p2[1]-(h/Lc)*(j4[0]-j3[0])
        yEb = p2[1]+(h/Lc)*(j4[0]-j3[0])
        
        if yEa>yEb:
            jEx = xEa
            jEy = yEa
        else:
            jEx = xEb
            jEy = yEb
        jE = N.array([jEx,jEy])
        
        # Now get the joint angles.
        v3E = jE-j3
        v31 = -j3
        angle3 = N.arccos(N.dot(v3E,v31)/(N.linalg.norm(v3E)*N.linalg.norm(v31))) - N.pi

        v4E = jE-j4
        v42 = j2-j4
        angle4 = N.pi - N.arccos(N.dot(v4E,v42)/(N.linalg.norm(v4E)*N.linalg.norm(v42))) 
            
        
        x = jEx - self.xCenter
        y = jEy - self.yCenter

        
        # Put angles into the proper range for their limits.
        if self.q1MinE<0.0:
            lo = -N.pi
            hi =  N.pi
        else:
            lo = 0.0
            hi = 2.0*N.pi
        angle1 = ((angle1-lo)%(2.0*N.pi))+lo

        if self.q2MinE<0.0:
            lo = -N.pi
            hi =  N.pi
        else:
            lo = 0.0
            hi = 2.0*N.pi
        angle2 = ((angle2-lo)%(2.0*N.pi))+lo
        

        angle1 = N.clip(angle1,self.q1MinE,self.q1MaxE)
        angle2 = N.clip(angle2,self.q2MinE,self.q2MaxE)

        angle1 = angle1 - self.q1CenterE
        angle2 = angle2 - self.q2CenterE
        
        return (angle1, angle2, angle3, angle4, x, y)
    
    
    # Get1234FromPt()
    # Inverse Kinematics: Get thetas (1,2,3,4) from the (x,y) end-effector position,
    # where (x,y) is in the coordinate frame of the workspace center,
    # and (angle1,angle2)==(0,0) maps to (x,y)==(0,0).
    #
    def Get1234FromPt (self, pt):
        (ptClipped, bClipped) = self.ClipPtToReachable(pt)

        # Flip the axes if necessary.
        x = ptClipped.x
        y = ptClipped.y
        
        # Translate to frame at joint1.
        x = x + self.xCenter
        y = y + self.yCenter
        
        #rospy.logwarn('5B reachable: (x,y)=%s, norm0E=%s, norm2E=%s, self.L1+self.L3=%s, self.L2+self.L4=%s' % ([x,y], 
        #                                                                 N.linalg.norm(N.array([x,y])), 
        #                                                                 N.linalg.norm(N.array([x-self.L0,y])), 
        #                                                                 self.L1+self.L3, 
        #                                                                 self.L2+self.L4))
        self.L0sq = self.L0*self.L0
        self.L1sq = self.L1*self.L1
        self.L2sq = self.L2*self.L2
        self.L3sq = self.L3*self.L3
        self.L4sq = self.L4*self.L4
        xsq = x*x
        ysq = y*y
        x3rd = x**3.0
        x4th = x**4.0
        y4th = y**4.0
        y6th = y**6.0
        
        Aa = (2.0*xsq*self.L3sq*ysq+2.0*xsq*self.L1sq*ysq+2.0*ysq*self.L3sq*self.L1sq-2.0*xsq*y4th-x4th*ysq-ysq*self.L1**4.0-ysq*self.L3**4.0+2.0*y4th*self.L3sq+2.0*y4th*self.L1sq-y6th)
        Bb = (-4.0*self.L2sq*self.L0*x*ysq-4.0*x*ysq*self.L4sq*self.L0-2.0*xsq*y4th-x4th*ysq+2.0*xsq*self.L2sq*ysq+2.0*self.L2sq*self.L0sq*ysq-6.0*xsq*ysq*self.L0sq+2.0*xsq*ysq*self.L4sq+4.0*x3rd*ysq*self.L0+4.0*x*y4th*self.L0+4.0*self.L0**3.0*x*ysq+2.0*ysq*self.L0sq*self.L4sq+2.0*ysq*self.L4sq*self.L2sq-2.0*y4th*self.L0sq-self.L0**4.0*ysq+2.0*y4th*self.L4sq-ysq*self.L2**4.0-ysq*self.L4**4.0+2.0*y4th*self.L2sq-y6th)
        if (Aa<0.0) or(Bb<0):
            rospy.logerr('**********************************************************')
            rospy.logerr('Fivebar actuator tried to move out of reachable workspace!')
            rospy.logerr('**********************************************************')
            
        A = Aa**0.5
        B = Bb**0.5
        C = -self.L2sq*self.L0+x*self.L2sq-self.L0**3.0+x*ysq-ysq*self.L0-3*self.L0*xsq+self.L4sq*self.L0+x3rd-x*self.L4sq+3*self.L0sq*x
        D = 0.5*(xsq*y-2.0*y*self.L0*x+y*self.L0sq-y*self.L4sq+y**3.0+y*self.L2sq)
        F = x*self.L1sq+x3rd-x*self.L3sq+x*ysq
        G = 0.5*(-2.0*self.L2*self.L4*x+2.0*self.L2*self.L4*self.L0)
        H = self.L2*self.L4*xsq-2.0*self.L2*self.L4*self.L0*x+self.L2*self.L4*self.L0sq-self.L2*self.L4**3.0+self.L2*self.L4*ysq+self.L2**3.0*self.L4
        J = 2.0*y*self.L2sq*self.L0-2.0*y*self.L2sq*x
        K = 0.5*(y**3.0+xsq*y+y*self.L1sq-y*self.L3sq)
        M = self.L3*self.L1**3.0-self.L3**3.0*self.L1+self.L3*self.L1*ysq+xsq*self.L3*self.L1
        P = (ysq+self.L0sq+xsq-2.0*self.L0*x)
        Q = xsq-2.0*self.L0*x+self.L0sq-self.L4sq+ysq+self.L2sq
        R = 0.5*(xsq-2.0*self.L0*x+self.L0sq-self.L4sq+ysq-self.L2sq)/self.L2/self.L4
                
        q1 = [ \
                N.arctan2((-x*(F+A)/(ysq+xsq)+self.L1sq-self.L3sq+ysq+xsq)/self.L1/y, \
                          (    F+A)/(ysq+xsq)/self.L1), \
                N.arctan2((-x*(F-A)/(ysq+xsq)+self.L1sq-self.L3sq+ysq+xsq)/self.L1/y, \
                          (    F-A)/(ysq+xsq)/self.L1) \
              ]

        q3 = [ \
                N.arctan2((-2.0*y*x*self.L1sq+K*(F+A)/(ysq+xsq))/(-self.L3*self.L1*x*(F+A)/(ysq+xsq)+M), \
                          1.0/2.0*(xsq-self.L3sq+ysq-self.L1sq)/self.L1/self.L3), \
                N.arctan2((-2.0*y*x*self.L1sq+K*(F-A)/(ysq+xsq))/(-self.L3*self.L1*x*(F-A)/(ysq+xsq)+M), \
                          1.0/2.0*(xsq-self.L3sq+ysq-self.L1sq)/self.L1/self.L3) \
              ]

        q2 = [ \
                N.arctan2((Q+0.5*(-2.0*x+2.0*self.L0)*(C+B)/P)/self.L2/y, \
                          (C+B)/P/self.L2), \
                N.arctan2((Q+0.5*(-2.0*x+2.0*self.L0)*(C-B)/P)/self.L2/y, \
                          (C-B)/P/self.L2) \
              ]
        
        q4 = [ \
                N.arctan2((J+D*(C+B)/P)/(H+G*(C+B)/P), 
                          R), \
                N.arctan2((J+D*(C-B)/P)/(H+G*(C-B)/P), 
                          R) \
              ]

        # Put angles into the proper 2pi range for their limits.
        if self.q1MinE<0.0:
            lo = -N.pi
            hi =  N.pi
        else:
            lo = 0.0
            hi = 2.0*N.pi
        q1[0] = ((q1[0]-lo)%(2.0*N.pi))+lo
        q1[1] = ((q1[1]-lo)%(2.0*N.pi))+lo

        if self.q2MinE<0.0:
            lo = -N.pi
            hi =  N.pi
        else:
            lo = 0.0
            hi = 2.0*N.pi
        q2[0] = ((q2[0]-lo)%(2.0*N.pi))+lo
        q2[1] = ((q2[1]-lo)%(2.0*N.pi))+lo
        
        
        # Select the proper configuration (it always seems to be k=1 and k=0, respectively)
        angle1 = 999.0
        angle2 = 999.0
        angle3 = 999.0
        angle4 = 999.0
        #for k in range(len(q1)):
        #    rospy.logwarn('5B Q1: %s < %s < %s and pi < %s' % (self.q1MinE, q1[k], self.q1MaxE, q3[k] %(2.0*N.pi)))
        for k in range(len(q1)):
            #if (self.q1MinE < q1[k] < self.q1MaxE) and (N.pi < (q3[k] %(2.0*N.pi))):
            if (N.pi < (q3[k] %(2.0*N.pi))):
                angle1 = q1[k]
                angle3 = q3[k]
                k1 = k
                break
            
        
        #for k in range(len(q2)):
        #    rospy.logwarn('5B Q2: %s < %s < %s and %s < pi' % (self.q2MinE, q2[k], self.q2MaxE, q4[k] %(2.0*N.pi)))
        for k in range(len(q2)):
            #if (self.q2MinE < q2[k] < self.q2MaxE) and ((q4[k] %(2.0*N.pi)) < N.pi):
            if ((q4[k] %(2.0*N.pi)) < N.pi):
                angle2 = q2[k]
                angle4 = q4[k]
                k2 = k
                break
        
        #if (angle1==999.0) or (angle2==999.0) or (angle3==999.0) or (angle4==999.0):
        #    rospy.logwarn('5B thetas=%s, x,y=%s' % ([angle1,angle2,angle3,angle4],[x0,y0]))
            
        angle1 = N.clip(angle1,self.q1MinE,self.q1MaxE)
        angle2 = N.clip(angle2,self.q2MinE,self.q2MaxE)

        angle1 = angle1 - self.q1CenterE
        angle2 = angle2 - self.q2CenterE

        return (angle1,angle2,angle3,angle4)


    # JacobianFwd()
    # The forward jacobian matrix, i.e. transform from joint anglular velocities to cartesian velocities.
    # Note that the equations below assume a 3/1 ratio in L3/L1, and in L4/L2, and that L0=L3=L4.
    #
    def JacobianFwd (self, angle1, angle2):
        t1 = angle1 + self.q1CenterE
        t2 = angle2 + self.q2CenterE
        
        A = N.tan(t1/2.0)
        B = N.tan(t2/2.0)
        C = (A**2/2.0 + 0.5)
        D = (B**2/2.0 + 0.5)
        E = A**2*B**2
        G = (7.0*B**2 - 5.0*A**2 + 27.0*E + 16.0*A*B - 9.0)
        H = (11.0*A**2 + 35.0*B**2 + 27.0*E + 8.0*A*B + 27.0)
        J = (25.0*A**2 + B**2 + 9.0*E - 8.0*A*B + 9.0)
        S = (108.0*A + 216.0*B - 108.0*A*B**2 + 432.0*A**2*B - 9.0*(H*J)**(0.5))
        P = (12.0*A + 24.0*B - 12.0*A*B**2 + 48.0*A**2*B - (H*J)**(0.5))
        Z = (36.0*A - 36.0*B + 36.0*A*B**2 - 36.0*A**2*B - S/G - (5.0*A**2*P)/G + (7.0*B**2*P)/G + (27.0*E*P)/G + (16.0*A*B*P)/G)
        K = (22.0*A*C + 8.0*B*C + 54.0*A*B**2*C)
        L = (7.0*B**2 - 5.0*A**2 - 9.0*E + 16.0*A*B + 27.0)
        F = N.arctan2(Z,L)
        X = (5.0*(Z**2/L**2 + 1.0))
        M = (50.0*A*C - 8.0*B*C + 18.0*A*B**2*C)
        W = (K*J + M*H)
        Q = (10.0*A*C - 16.0*B*C + 18.0*A*B**2*C)
        R = (16.0*A*D + 14.0*B*D + 54.0*A**2*B*D)
        T = (8.0*A*D + 70.0*B*D + 54.0*A**2*B*D)
        U = (2.0*B*D - 8.0*A*D + 18.0*A**2*B*D)
        V = (54.0*A**2 - 108.0*B**2*C - (9.0*W)/(2*(H*J)**(0.5)) + 864.0*A*B*C + 54.0)
        Y = ((16.0*A*D + 14.0*B*D - 18.0*A**2*B*D)*Z)
        AA = (432.0*A**2*D - (9.0*(T*J + U*H))/(2.0*(H*J)**(0.5)) + 108.0*B**2 - 216.0*A*B*D + 108.0)
        AD = (6.0*A**2 - 12.0*B**2*C - W/(2.0*(H*J)**(0.5)) + 96.0*A*B*C + 6.0)
        AE = (16.0*B*C - 10.0*A*C + 54.0*A*B**2*C)
        AC = (AE*S)
        AF = (48.0*A**2*D - (T*J + U*H)/(2.0*(H*J)**(0.5)) + 12.0*B**2 - 24.0*A*B*D + 12.0)
        AB = (5.0*A**2*AF)
        AG = (36.0*B**2*C + 18.0*A**2 - V/G + AC/G**2 - 72.0*A*B*C - (5.0*A**2*AD)/G + (7.0*B**2*AD)/G + (27.0*E*AD)/G + (16.0*A*B*AD)/G - (10.0*A*C*P)/G + (16.0*B*C*P)/G + (5.0*A**2*AE*P)/G**2 - (7.0*B**2*AE*P)/G**2 + (54.0*A*B**2*C*P)/G - (27.0*E*AE*P)/G**2 - (16.0*A*B*AE*P)/G**2 + 18.0)
        AH = ((R*S)/G**2 - 18.0*B**2 - AA/G - 36.0*A**2*D + 72.0*A*B*D - AB/G + (7.0*B**2*AF)/G + (27.0*E*AF)/G + (16.0*A*B*AF)/G + (16.0*A*D*P)/G + (14.0*B*D*P)/G + (5.0*A**2*R*P)/G**2 - (7.0*B**2*R*P)/G**2 + (54.0*A**2*B*D*P)/G - (27.0*E*R*P)/G**2 - (16.0*A*B*R*P)/G**2 - 18.0)
        
        
        j11= (2667.0*N.cos(t1)*N.sin(2.0*F))/10.0 - (889.0*N.sin(t1))/10.0 - (2667.0*N.cos(2.0*F)*N.sin(t1))/10.0 - (2667.0*N.cos(t1)*N.sin(2.0*F)*(AG/L + (Q*Z)/L**2))/X + (2667.0*N.cos(2*F)*N.sin(t1)*(AG/L + (Q*Z)/L**2))/X
        j12=   (2667.0*N.cos(2.0*F)*N.sin(t1)*(AH/L - Y/L**2))/X - (2667.0*N.cos(t1)*N.sin(2.0*F)*(AH/L - Y/L**2))/X
        j21= (889.0*N.cos(t1))/10.0 + (2667.0*N.cos(t1)*N.cos(2.0*F))/10.0 + (2667.0*N.sin(t1)*N.sin(2.0*F))/10.0 - (2667.0*N.cos(t1)*N.cos(2.0*F)*(AG/L + (Q*Z)/L**2))/X - (2667.0*N.sin(t1)*N.sin(2.0*F)*(AG/L + (Q*Z)/L**2))/X
        j22= - (2667.0*N.cos(t1)*N.cos(2.0*F)*(AH/L - Y/L**2))/X - (2667.0*N.sin(t1)*N.sin(2*F)*(AH/L - Y/L**2))/X
    
        return N.array([[j11,j12],[j21,j22]])


    def JacobianInv (self, angle1, angle2):
        return N.linalg.inv(self.JacobianFwd (angle1, angle2))

        
    def GetXyFrom12 (self, angle1, angle2):
        (angle1, angle2, angle3, angle4, x, y) = self.Get1234xyFrom12(angle1, angle2)
        return (x, y)
    
    
    def GetStageState_callback(self, reqStageState):
        #rospy.logwarn('GetStageState_callback()')
        state = self.GetStageState()
        return state
    
    def GetStageState (self):
        while not self.initializedServices:
            rospy.sleep(0.5)
            
        rvStageState = SrvFrameStateResponse()
        rvStageState.state = None
        if (self.jointstate1 is not None) and (self.jointstate2 is not None):
            if (self.stateVisual is not None):                     
                rvStageState.state = self.stateVisual
            else:
                rvStageState.state = self.stateMech
        
        return rvStageState.state

        
    # SetStageStateRef_callback()
    #   Updates the target command.
    #
    def SetStageStateRef_callback(self, reqStageState):
        #rospy.logwarn('SetStageStateRef_callback()')
        while not self.initialized:
            rospy.sleep(0.5)
            
        (self.stateRef, isInArena) = self.TransformStateToFrame('Stage', reqStageState.state)

        rvStageState = SrvFrameStateResponse()
        rvStageState.state = reqStageState.state

        return rvStageState
    

    # Take the target point from a signal generator, and set it as the tool reference.  
    # If the point falls outside the workspace, then clip it to the workspace, and return False.
    def SignalInput_callback (self, reqSignal):
        #rospy.logwarn('SignalInput_callback()')
        with self.lock:
            #rospy.logwarn('SignalInput_callback() locked')
            rvSignal = SrvSignalResponse()
            
            (self.stateRef, isInArena) = self.TransformStateToFrame('Stage', reqSignal.state)
            rvSignal.success = isInArena
        
        #rospy.logwarn('SignalInput_callback() free')
        return rvSignal


    # Create a vector with direction of ptDir, and magnitude of ptMag.
    def ScaleVecToMag (self, ptDir, ptMag):
        magPtDir = N.linalg.norm([ptDir.x, ptDir.y, ptDir.z])
        magPtMag = N.linalg.norm([ptMag.x, ptMag.y, ptMag.z])
        
        ptNew = Point (x=magPtMag / magPtDir * ptDir.x,
                       y=magPtMag / magPtDir * ptDir.y,
                       z=magPtMag / magPtDir * ptDir.z)
        return ptNew
    

    def VisualState_callback(self, state):
        #rospy.logwarn('VisualState_callback()')
        with self.lock:
            #rospy.logwarn('VisualState_callback() locked')
            (self.stateVisual, isInArena) = self.TransformStateToFrame('Stage', state, doClipToArena=False)
                
        #rospy.logwarn('VisualState_callback() free')
        
        
    def HomeStage_callback(self, reqStageState):
        #rospy.logwarn('HomeStage_callback()')
        while not self.initialized:
            rospy.sleep(0.1)
            
        with self.lock:
            try:
                self.Park_joint1()
                self.Park_joint2()
            except (rospy.ServiceException, rospy.exceptions.ROSInterruptException, IOError), e:
                rospy.logwarn ("5B FAILED %s"%e)
                
        rvStageState = SrvFrameStateResponse()
        rospy.loginfo ('5B HomeStage_callback rvStageState=%s' % rvStageState)
        return rvStageState
        
        
    def Calibrate_callback(self, req):
        #rospy.logwarn('Calibrate_callback()')
        # Integer values for directions - used in usb set/get
        POSITIVE = 0
        NEGATIVE = 1
        
        # Convert parking coordinates to angles.
        self.xPark = self.xPark
        self.yPark = self.yPark
        with self.lock:
            [q1park,q2park,q3park,q4park] = self.Get1234FromPt(Point(x=self.xPark, y=self.yPark))

        q1origin = self.q1CenterI # From the index switch
        q2origin = self.q2CenterI # From the index switch

        rospy.loginfo ('5B Center = %s, %s' % (self.xCenter, self.yCenter))
        rospy.loginfo ('5B Calibrating: q1origin=%s, q1park=%s, q2origin=%s, q2park=%s' % (q1origin, q1park, q2origin, q2park))
        with self.lock:
            try:
                rv = self.Calibrate_joint1(NEGATIVE, q1origin, q1park, True)
                rospy.loginfo ('5B Calibrated joint1')
                q1index = rv.position
                rv = self.Calibrate_joint2(POSITIVE, q2origin, q2park, True)
                rospy.loginfo ('5B Calibrated joint2')
                q2index = rv.position
            except (rospy.ServiceException, rospy.exceptions.ROSInterruptException, IOError), e:
                rospy.logwarn ("5B Exception:  %s" % e)
           

        rvStageState = self.GetStageState()
        rospy.loginfo ('5B Calibrate_callback rvStageState=%s' % rvStageState)
        
        # Set zero velocity after calibration.
        self.stateRef = rvStageState

        return rvStageState
    

    def SendTransforms(self):  
        if self.initialized:
            #rospy.logwarn('SendTransforms()')
            with self.lock:
                #rospy.logwarn('SendTransforms() locked')
                try:
                    self.jointstate1 = self.GetState_joint1()
                except rospy.ServiceException, e:
                    stSrv = 'srvGetState_' + self.names[0]
                    try:
                        rospy.wait_for_service(stSrv, timeout=5)
                        self.GetState_joint1 = rospy.ServiceProxy(stSrv, SrvJointState, persistent=True)
                    except rospy.ServiceException, e:
                        rospy.logwarn ('5B FAILED to reconnect service %s(): %s' % (stSrv, e))
                    
                try:
                    self.jointstate2 = self.GetState_joint2()
                except rospy.ServiceException, e:
                    stSrv = 'srvGetState_' + self.names[1]
                    try:
                        rospy.wait_for_service(stSrv, timeout=5)
                        self.GetState_joint2 = rospy.ServiceProxy(stSrv, SrvJointState, persistent=True)
                    except rospy.ServiceException, e:
                        rospy.logwarn ('5B FAILED to reconnect service %s(): %s' % (stSrv, e))
                        
            if (self.jointstate1 is not None) and (self.jointstate2 is not None):                    
                (angle1,angle2,angle3,angle4, self.stateMech.pose.position.x, self.stateMech.pose.position.y) = self.Get1234xyFrom12(self.jointstate1.position, self.jointstate2.position)

                # Publish the joint states (for rviz, etc)    
                self.js.header.seq = self.js.header.seq + 1
                self.js.header.stamp = self.time
                self.js.position = [angle1,angle2,angle3,angle4]
                self.pubJointState.publish(self.js)
    
                qEE = tf.transformations.quaternion_from_euler(0.0, 0.0, angle1+self.q1CenterE+angle3)
                state = MsgFrameState()
                state.header.stamp = self.time
                state.header.frame_id = 'Stage'
                state.pose.position = self.stateMech.pose.position
                state.pose.orientation.x = qEE[0]
                state.pose.orientation.y = qEE[1]
                state.pose.orientation.z = qEE[2]
                state.pose.orientation.w = qEE[3]
                
                #rospy.logwarn ('A')
                #rospy.logwarn ('A: %s, %s, %s' % ((-self.L3/2, -246.31423, 0.0),tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0), state.header.stamp))
                # Publish the link transforms.
                self.tfbx.sendTransform((-self.L3/2, -246.31423, 0.0), 
                                        tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                                        state.header.stamp,
                                        'link0',  # child
                                        'Stage'  # parent
                                        )
                #rospy.logwarn ('B')
                #rospy.logwarn ('B: %s, %s, %s' % ((0.0, 0.0, 0.0), tf.transformations.quaternion_from_euler(0.0, 0.0, angle1+self.q1CenterE), state.header.stamp))
                self.tfbx.sendTransform((0.0, 0.0, 0.0), 
                                        tf.transformations.quaternion_from_euler(0.0, 0.0, angle1+self.q1CenterE),
                                        state.header.stamp,
                                        'link1',  # child
                                        'link0'  # parent
                                        )
                #rospy.logwarn ('C')
                #rospy.logwarn ('C: %s, %s, %s' % ((self.L1, 0.0, 0.0), tf.transformations.quaternion_from_euler(0.0, 0.0, angle3), state.header.stamp))
                self.tfbx.sendTransform((self.L1, 0.0, 0.0), 
                                        tf.transformations.quaternion_from_euler(0.0, 0.0, angle3),
                                        state.header.stamp,
                                        "link3",     # child
                                        "link1"      # parent
                                        )
                #rospy.logwarn ('D')
                #rospy.logwarn ('D: %s, %s, %s' % ((self.L3, 0.0, 0.0), tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0), state.header.stamp))
                self.tfbx.sendTransform((self.L3, 0.0, 0.0), 
                                        tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                                        state.header.stamp,
                                        "link5",     # child
                                        "link3"      # parent
                                        )
                #rospy.logwarn ('E')
                #rospy.logwarn ('E: %s, %s, %s' % ((self.L0, 0.0, 0.0), tf.transformations.quaternion_from_euler(0.0, 0.0, angle2+self.q2CenterE), state.header.stamp))
                self.tfbx.sendTransform((self.L0, 0.0, 0.0), 
                                        tf.transformations.quaternion_from_euler(0.0, 0.0, angle2+self.q2CenterE),
                                        state.header.stamp,
                                        "link2",     # child
                                        "link0"      # parent
                                        )
                #rospy.logwarn ('F')
                #rospy.logwarn ('F: %s, %s, %s' % ((self.L2, 0.0, 0.0), tf.transformations.quaternion_from_euler(0.0, 0.0, angle4), state.header.stamp))
                self.tfbx.sendTransform((self.L2, 0.0, 0.0), 
                                        tf.transformations.quaternion_from_euler(0.0, 0.0, angle4),
                                        state.header.stamp,
                                        "link4",     # child
                                        "link2"      # parent
                                        )
                #rospy.logwarn ('G')
                #rospy.logwarn('G: %s, %s, %s' % ((state.pose.position.x, state.pose.position.y, state.pose.position.z), qEE, state.header.stamp))
                
                # Frame EndEffector
                self.tfbx.sendTransform((state.pose.position.x, state.pose.position.y, state.pose.position.z),
                                        qEE,
                                        state.header.stamp,
                                        'EndEffector',  # child
                                        state.header.frame_id  # parent
                                        )

                #rospy.logwarn ('H')
                # Frame Target
                if self.stateRef is not None:
                    markerTarget = Marker(header=self.stateRef.header,
                                          ns='target',
                                          id=0,
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

                if self.stateVisual is not None:
                    markerToolOffset = Marker(header=state.header,
                                              ns='tooloffset',
                                              id=1,
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
                #rospy.logwarn ('K')

            #rospy.logwarn('SendTransforms() free')


    # UpdateMotorCommandFromTarget()
    #   Updates the motor command with the current target.
    #
    def UpdateMotorCommandFromTarget(self):
        if self.stateRef is not None:
            # PID Gains & Parameters.
            if (self.bTune):
                self.kP      = rospy.get_param('fivebar/kP', 1.0)
                self.kI      = rospy.get_param('fivebar/kI', 0.0)
                self.kD      = rospy.get_param('fivebar/kD', 0.0)
                self.maxI    = rospy.get_param('fivebar/maxI', 40.0)
                self.kWindup = rospy.get_param('fivebar/kWindup', 0.0)

                self.kPv      = rospy.get_param('fivebar/kPv', 0.0)
                self.kIv      = rospy.get_param('fivebar/kIv', 0.0)
                self.kDv      = rospy.get_param('fivebar/kDv', 0.0)
                self.maxIv    = rospy.get_param('fivebar/maxIv', 40.0)
                self.kWindupv = rospy.get_param('fivebar/kWindupv', 0.0)

                self.kAll    = rospy.get_param('fivebar/kAll', 1.0)
                self.speedLinearMax = rospy.get_param('fivebar/speed_max', 100.0)
                
                self.kP *= self.kAll
                self.kI *= self.kAll
                self.kD *= self.kAll

                a = rospy.get_param('/a', 0.05) # This is the filter constant for the derivative term.
            else:
                a = 0.05
            
            # The previous error.
            self.statePErrorPrev = copy.deepcopy(self.statePError)
            
            
            # Use either visual servoing, or not, as the case may be, depending on if we've received a visual state callback.
            if (self.stateVisual is not None):
                stateActuator = self.stateVisual
            else:
                stateActuator = self.stateMech
                

            if (self.stateRef is not None):
                # Error terms.
                self.statePError.pose.position.x = self.stateRef.pose.position.x - stateActuator.pose.position.x
                self.statePError.pose.position.y = self.stateRef.pose.position.y - stateActuator.pose.position.y
                self.statePError.pose.position.z = self.stateRef.pose.position.z - stateActuator.pose.position.z
                self.statePError.velocity.linear.x = self.stateRef.velocity.linear.x - stateActuator.velocity.linear.x
                self.statePError.velocity.linear.y = self.stateRef.velocity.linear.y - stateActuator.velocity.linear.y
                self.statePError.velocity.linear.z = self.stateRef.velocity.linear.z - stateActuator.velocity.linear.z

                self.stateIError.pose.position.x += self.statePError.pose.position.x
                self.stateIError.pose.position.y += self.statePError.pose.position.y
                self.stateIError.pose.position.z += self.statePError.pose.position.z
                self.stateIError.velocity.linear.x += self.statePError.velocity.linear.x
                self.stateIError.velocity.linear.y += self.statePError.velocity.linear.y
                self.stateIError.velocity.linear.z += self.statePError.velocity.linear.z
                
                self.stateDError.pose.position.x = (1-a)*self.stateDError.pose.position.x + a*(self.statePError.pose.position.x - self.statePErrorPrev.pose.position.x) # filtered.
                self.stateDError.pose.position.y = (1-a)*self.stateDError.pose.position.y + a*(self.statePError.pose.position.y - self.statePErrorPrev.pose.position.y)
                self.stateDError.pose.position.z = (1-a)*self.stateDError.pose.position.z + a*(self.statePError.pose.position.z - self.statePErrorPrev.pose.position.z)
                self.stateDError.velocity.linear.x = self.statePError.velocity.linear.x - self.statePErrorPrev.velocity.linear.x
                self.stateDError.velocity.linear.y = self.statePError.velocity.linear.y - self.statePErrorPrev.velocity.linear.y
                self.stateDError.velocity.linear.z = self.statePError.velocity.linear.z - self.statePErrorPrev.velocity.linear.z
                
                
#                 xRef = N.array([self.stateRef.pose.position.x, 
#                                 self.stateRef.pose.position.y, 
#                                 self.stateRef.pose.position.z,
#                                 self.stateRef.velocity.linear.x, 
#                                 self.stateRef.velocity.linear.y, 
#                                 self.stateRef.velocity.linear.z])
#                 
#                 xVis = N.array([stateActuator.pose.position.x, 
#                                 stateActuator.pose.position.y, 
#                                 stateActuator.pose.position.z,
#                                 stateActuator.velocity.linear.x, 
#                                 stateActuator.velocity.linear.y, 
#                                 stateActuator.velocity.linear.z])
                
                
                # PID control of the visual position error.
                self.statePID.pose.position = Point(x=self.kP*self.statePError.pose.position.x + self.kI*self.stateIError.pose.position.x + self.kD*self.stateDError.pose.position.x,
                                                    y=self.kP*self.statePError.pose.position.y + self.kI*self.stateIError.pose.position.y + self.kD*self.stateDError.pose.position.y,
                                                    z=self.kP*self.statePError.pose.position.z + self.kI*self.stateIError.pose.position.z + self.kD*self.stateDError.pose.position.z)
                                    
                # Anti-windup
                ptEeIErrorClipped = self.ClipPtMag (self.stateIError.pose.position, self.maxI)
                ptExcessI = Point(self.stateIError.pose.position.x - ptEeIErrorClipped.x,
                                  self.stateIError.pose.position.y - ptEeIErrorClipped.y,
                                  self.stateIError.pose.position.z - ptEeIErrorClipped.z)
                self.stateIError.pose.position.x -= self.kWindup * ptExcessI.x
                self.stateIError.pose.position.y -= self.kWindup * ptExcessI.y
                self.stateIError.pose.position.z -= self.kWindup * ptExcessI.z

            

                # PID control of the visual velocity error.
                self.statePID.velocity.linear = Point(x=self.kPv*self.statePError.velocity.linear.x + self.kIv*self.stateIError.velocity.linear.x + self.kDv*self.stateDError.velocity.linear.x,
                                                      y=self.kPv*self.statePError.velocity.linear.y + self.kIv*self.stateIError.velocity.linear.y + self.kDv*self.stateDError.velocity.linear.y,
                                                      z=self.kPv*self.statePError.velocity.linear.z + self.kIv*self.stateIError.velocity.linear.z + self.kDv*self.stateDError.velocity.linear.z)
                                    
                # Anti-windup
                ptEeIErrorClipped = self.ClipPtMag (self.stateIError.velocity.linear, self.maxIv)
                ptExcessI = Point(self.stateIError.velocity.linear.x - ptEeIErrorClipped.x,
                                  self.stateIError.velocity.linear.y - ptEeIErrorClipped.y,
                                  self.stateIError.velocity.linear.z - ptEeIErrorClipped.z)
                self.stateIError.velocity.linear.x -= self.kWindup * ptExcessI.x
                self.stateIError.velocity.linear.y -= self.kWindup * ptExcessI.y
                self.stateIError.velocity.linear.z -= self.kWindup * ptExcessI.z
                
            
                # Get the angular positions for each joint.
                (angle1Mech,   angle2Mech,   angle3Mech,   angle4Mech)    = self.Get1234FromPt(self.stateMech.pose.position)

                
                # Compute the velocity command.
                xDot = (self.stateRef.velocity.linear.x + self.statePID.velocity.linear.x) + self.statePID.pose.position.x
                yDot = (self.stateRef.velocity.linear.y + self.statePID.velocity.linear.y) + self.statePID.pose.position.y


                # Pull the mechanical position back under the visual position.
                xDot += (stateActuator.pose.position.x - self.stateMech.pose.position.x)
                yDot += (stateActuator.pose.position.y - self.stateMech.pose.position.y)

                # Clip to max allowed speed.
                if (self.stateRef.speed != 0.0):  # If speed is unspecified, then this field is set to 0.0
                    speedMax = min(self.stateRef.speed, self.speedLinearMax)
                else:
                    speedMax = self.speedLinearMax
                    
                pt = self.ClipPtMag(Point(x=xDot,y=yDot), speedMax)
                xDot = pt.x
                yDot = pt.y
                
                
                # Clip to stay in workspace.
                pts = PointStamped(header=Header(stamp=stateActuator.header.stamp,
                                                 frame_id='Stage'),
                                   point=Point(x = xDot + self.stateMech.pose.position.x,
                                               y = yDot + self.stateMech.pose.position.y))
                (ptsArena, isInArena)         = self.ClipPtsToArena(pts)
                xDot = ptsArena.point.x - self.stateMech.pose.position.x
                yDot = ptsArena.point.y - self.stateMech.pose.position.y
                
                
                # Convert to motor coordinates.
                jInv = self.JacobianInv(angle1Mech, angle2Mech)
                (theta1Dot, theta2Dot) = jInv.dot(N.array([[xDot],[yDot]])) 
                
                
                # Display the velocity vector in rviz.
                ptBase = self.stateMech.pose.position # self.ptEeMech #stateActuator.pose.position
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
                    ptBase = stateActuator.pose.position
                    ptEnd = Point(x = ptBase.x + self.statePID.pose.position.x,
                                  y = ptBase.y + self.statePID.pose.position.y,
                                  z = ptBase.z + self.statePID.pose.position.z
                                  ) 
                    markerCommand = Marker(header=Header(stamp=self.time,
                                                        frame_id='Stage'),
                                          ns='pidPosition',
                                          id=2,
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
                                          points=[ptBase, ptEnd])
                    self.pubMarker.publish(markerCommand)
            
                    # Display P,I,D vectors in rviz.
                    ptBase = stateActuator.pose.position #self.ptEeMech
                    ptEnd = Point(x = ptBase.x + self.kP*self.statePError.pose.position.x,
                                  y = ptBase.y + self.kP*self.statePError.pose.position.y)
                    markerCommand= Marker(header=Header(stamp = self.time, frame_id='Stage'),
                                          ns='P',
                                          id=5, type=Marker.ARROW, action=0,
                                          scale=Vector3(x=0.1, y=0.2, z=0.0),
                                          color=ColorRGBA(a=0.9, r=1.0, g=0.0, b=0.0),
                                          lifetime=rospy.Duration(1.0), points=[ptBase, ptEnd])
                    self.pubMarker.publish(markerCommand)
                
                    ptEnd = Point(x = ptBase.x + self.kI*self.stateIError.pose.position.x,
                                  y = ptBase.y + self.kI*self.stateIError.pose.position.y)
                    markerCommand= Marker(header=Header(stamp = self.time, frame_id='Stage'),
                                          ns='I',
                                          id=6, type=Marker.ARROW, action=0,
                                          scale=Vector3(x=0.1, y=0.2, z=0.0),
                                          color=ColorRGBA(a=0.9, r=0.0, g=1.0, b=0.0),
                                          lifetime=rospy.Duration(1.0), points=[ptBase, ptEnd])
                    self.pubMarker.publish(markerCommand)
                    ptEnd = Point(x = ptBase.x + self.kD*self.stateDError.pose.position.x,
                                  y = ptBase.y + self.kD*self.stateDError.pose.position.y)
                    markerCommand= Marker(header=Header(stamp = self.time, frame_id='Stage'),
                                          ns='D',
                                          id=7, type=Marker.ARROW, action=0,
                                          scale=Vector3(x=0.1, y=0.2, z=0.0),
                                          color=ColorRGBA(a=0.9, r=0.0, g=0.0, b=1.0),
                                          lifetime=rospy.Duration(1.0), points=[ptBase, ptEnd])
                    self.pubMarker.publish(markerCommand)


                    # Print the PID component values.
                    magP = self.kP * N.linalg.norm([self.statePError.pose.position.x, self.statePError.pose.position.y])
                    magI = self.kI * N.linalg.norm([self.stateIError.pose.position.x, self.stateIError.pose.position.y])
                    magD = self.kD * N.linalg.norm([self.stateDError.pose.position.x, self.stateDError.pose.position.y])
                    magPID = N.linalg.norm([self.statePID.pose.position.x, self.statePID.pose.position.y])
                    
                    magPv = self.kPv * N.linalg.norm([self.statePError.velocity.linear.x, self.statePError.velocity.linear.y])
                    magIv = self.kIv * N.linalg.norm([self.stateIError.velocity.linear.x, self.stateIError.velocity.linear.y])
                    magDv = self.kDv * N.linalg.norm([self.stateDError.velocity.linear.x, self.stateDError.velocity.linear.y])
                    magPIDv = N.linalg.norm([self.statePID.velocity.linear.x, self.statePID.velocity.linear.y])
                    
                    rospy.logwarn('[P,I,D]=[% 5.2f,% 5.2f,% 5.2f]=% 5.2f, [% 5.2f,% 5.2f,% 5.2f]=% 5.2f, v=% 5.2f, % 5.2f' % (magP,magI,magD, magPID, magPv,magIv,magDv, magPIDv, theta1Dot, theta2Dot))

            
            else:
                theta1Dot = 0.0
                theta2Dot = 0.0
                
    
            if (self.jointstate1 is not None) and (self.jointstate2 is not None):
                with self.lock:
                    try:
                        self.SetVelocity_joint1(Header(frame_id=self.names[0]), None, theta1Dot)
                    except rospy.ServiceException, e:
                        stSrv = 'srvSetVelocity_' + self.names[0]
                        try:
                            rospy.wait_for_service(stSrv, timeout=5)
                            self.SetVelocity_joint1 = rospy.ServiceProxy(stSrv, SrvJointState, persistent=True)
                        except rospy.ServiceException, e:
                            rospy.logwarn ('5B FAILED to reconnect service %s(): %s' % (stSrv, e))

                    try:
                        self.SetVelocity_joint2(Header(frame_id=self.names[1]), None, theta2Dot)
                    except rospy.ServiceException, e:
                        stSrv = 'srvSetVelocity_' + self.names[1]
                        try:
                            rospy.wait_for_service(stSrv, timeout=5)
                            self.SetVelocity_joint2 = rospy.ServiceProxy(stSrv, SrvJointState, persistent=True)
                        except rospy.ServiceException, e:
                            rospy.logwarn ('5B FAILED to reconnect service %s(): %s' % (stSrv, e))

        
    def OnShutdown_callback(self):
        rospy.logwarn('OnShutdown_callback()')
        self.GetState_joint1.close()
        self.SetVelocity_joint1.close()

        self.GetState_joint2.close()
        self.SetVelocity_joint2.close()
        
        rospy.loginfo("5B Closed Fivebar device.")


        
    def Main(self):
        self.LoadServices()
        self.Calibrate_callback(None)
        self.initialized = True

#        rospy.loginfo ('5B QMIN,QMAX: %s, %s, %s, %s' % (self.q1MinE, self.q1MaxE, self.q2MinE, self.q2MaxE))
#        rospy.loginfo ('5B self.q1CenterEiz: %s, %s' % (self.q1CenterE, self.q1CenterI))
#        rospy.loginfo ('5B self.q2CenterEiz: %s, %s' % (self.q2CenterE, self.q2CenterI))
        # Process messages forever.
        rosrate = rospy.Rate(1 / self.T)

        while (not rospy.is_shutdown()) and (self.command != 'exit_now'):
            #rospy.logwarn ('Loop---------------')
            if (self.command != 'pause_now'):
                self.time = rospy.Time.now()
                self.dt = self.time - self.timePrev
                self.timePrev = self.time
                
                self.SendTransforms()
                self.UpdateMotorCommandFromTarget()

            else:
                (angle1Mech,  angle2Mech,  angle3Mech,  angle4Mech)   = self.Get1234FromPt(self.stateMech.pose.position)
                with self.lock:
                    try:
                        self.SetVelocity_joint1(Header(frame_id=self.names[0]), angle1Mech, 0.0)
                        self.SetVelocity_joint2(Header(frame_id=self.names[1]), angle2Mech, 0.0)
                    except (rospy.ServiceException, rospy.exceptions.ROSInterruptException, IOError), e:
                        rospy.logwarn ("5B Exception:  %s" % e)
            rosrate.sleep()
            
            
        # Shutdown all the services we offered.
        for key in self.services:
            self.services[key].shutdown()
            
    

if __name__ == '__main__':
    try:
        fivebar = Fivebar()
        fivebar.Main()
    except rospy.exceptions.ROSInterruptException:
        pass
    
    
