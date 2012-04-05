#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('fivebar')
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
#PRM:0:    KP                         4537740       750000
#PRM:1:    KI                         84154         35000
#PRM:2:    KPOS                       1729          15000
#PRM:11:   integral clamp             500           5000
#PRM:20:   operating mode             5             4
#PRM:26:   lowpass filter             1             0
#PRM:90:   baud rate                  38400         9600
#PRM:95:   daisy chain                1             0
#PRM:101:  fault output               1             0
#PRM:202:  filter cutoff              100.0         500.0
#PRM:204:  autotune distance          500.0         32000.0
#PRM:205:  autotune bandwidth         7.0           20.0


# Integer values for directions - used in usb set/get
POSITIVE = 0
NEGATIVE = 1



class RosFivebar:

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

        # Parking spot (millimeters)
        self.xPark = 0.0
        self.yPark = 0.0
        
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
        
        # Joint angles from furthest reachable point to center of workspace (i.e. "index" coordinates)
        self.q1CenterI   = self.q1CenterE - self.q1MinE
        self.q2CenterI   = self.q2CenterE - self.q2MaxE
        
        # Joint angles at center of workspace (i.e. "zero" coordinates)
        self.q1CenterZ   = 0.0
        self.q2CenterZ   = 0.0
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



        self.names = ['joint1','joint2','joint3','joint4']
        self.jointstate1 = None
        self.jointstate2 = None
        
        # Publish & Subscribe
        rospy.Service('set_stage_state',    SrvFrameState, self.SetStageState_callback)
        rospy.Service('get_stage_state',    SrvFrameState, self.GetStageState_callback)
        rospy.Service('home_stage',         SrvFrameState, self.HomeStage_callback)
        rospy.Service('calibrate_stage',    SrvFrameState, self.Calibrate_callback)
        
        rospy.Service('signal_input',       SrvSignal,     self.SignalInput_callback) # For receiving input from a signal generator.


        self.subEndEffectorOffset = rospy.Subscriber('EndEffectorOffset', Point, self.EndEffectorOffset_callback)
        self.pubJointState        = rospy.Publisher('joint_states', JointState)
        self.pubEndEffector       = rospy.Publisher('EndEffector', MsgFrameState)
        self.pubMarker            = rospy.Publisher('visualization_marker', Marker)
        
        self.tfrx = tf.TransformListener()
        self.tfbx = tf.TransformBroadcaster()

        self.js = JointState()
        self.js.header.seq = 0
        self.js.header.stamp.secs = rospy.get_time()
        self.js.header.frame_id = "1"
        self.js.name = self.names
        #self.js.velocity = [0.0, 0.0, 0.0, 0.0]
        
        self.xc = self.xCenter        
        self.yc = self.yCenter
        self.ptsToolRefExternal = None #Point(0,0,0) # Where we want the "tool".
        self.ptsToolRef = None
        self.ptToolRefClipped = Point(0,0,0) 
        self.ptContourSense = Point(0,0,0)
        self.ptContourError = Point(0,0,0)
        self.ptToolRefError = Point(0,0,0)
        self.ptOffsetSense = Point(0,0,0) # Vector from end-effector to the "tool"
        self.ptEeRef = Point(0,0,0)
        self.ptEeError = Point(0,0,0)
        self.ptEeErrorPrev = Point(0,0,0)
        self.ptEeDerror = Point(0,0,0)
        self.ptEeIerror = Point(0,0,0)
        self.ptEeIerrorClipped = Point(0,0,0)
        self.ptEeSense = Point(0,0,0)
        self.ptEeCommand = Point(0,0,0) # Where to command the end-effector.
        self.ptAntiwindup = Point(0,0,0)
        
        self.speedCommandTool = None 
        self.speedStageMax = rospy.get_param('fivebar/speed_max', 200.0)
        self.radiusMovement = rospy.get_param('arena/radius_movement', 25.4)
        
        self.timePrev = rospy.Time.now()
        self.time = rospy.Time.now()
        
        self.request = SrvFrameStateRequest()
        #self.request.state.header.stamp = self.time
        self.request.state.header.frame_id = 'Stage'
        self.request.state.pose.position.x = 0.0
        self.request.state.pose.position.y = 0.0
        self.request.state.pose.position.z = 0.0

        self.iCount = 0


    def LoadServices(self):
        # Load joint1 services.
        # Wait for joint 1 to launch.
        stSrv = 'srvCalibrate_'+self.names[0]
        rospy.wait_for_service(stSrv)
        try:
            self.calibrate_joint1 = rospy.ServiceProxy(stSrv, SrvCalibrate)
        except (rospy.ServiceException, IOError), e:
            rospy.loginfo ("5B FAILED %s: %s"%(stSrv,e))

        # Wait for joint 2 to launch.
        stSrv = 'srvCalibrate_'+self.names[1]
        rospy.wait_for_service(stSrv)
        try:
            self.calibrate_joint2 = rospy.ServiceProxy(stSrv, SrvCalibrate)
        except (rospy.ServiceException, IOError), e:
            rospy.loginfo ("5B FAILED %s: %s"%(stSrv,e))

        stSrv = 'srvGetState_'+self.names[0]
        rospy.wait_for_service(stSrv)
        try:
            self.getState_joint1 = rospy.ServiceProxy(stSrv, SrvJointState)
        except (rospy.ServiceException, IOError), e:
            rospy.loginfo ("5B FAILED %s: %s"%(stSrv,e))

        stSrv = 'srvSetPosition_'+self.names[0]
        rospy.wait_for_service(stSrv)
        try:
            self.setPosition_joint1 = rospy.ServiceProxy(stSrv, SrvJointState)
        except (rospy.ServiceException, IOError), e:
            rospy.loginfo ("5B FAILED %s: %s"%(stSrv,e))

        stSrv = 'srvSetPositionAtVel_'+self.names[0]
        rospy.wait_for_service(stSrv)
        try:
            self.setPositionAtVel_joint1 = rospy.ServiceProxy(stSrv, SrvJointState)
        except (rospy.ServiceException, IOError), e:
            rospy.loginfo ("5B FAILED %s: %s"%(stSrv,e))

        stSrv = 'srvPark_'+self.names[0]
        rospy.wait_for_service(stSrv)
        try:
            self.park_joint1 = rospy.ServiceProxy(stSrv, SrvJointState)
        except (rospy.ServiceException, IOError), e:
            rospy.loginfo ("5B FAILED %s: %s"%(stSrv,e))

        stSrv = 'srvGetState_'+self.names[1]
        rospy.wait_for_service(stSrv)
        try:
            self.getState_joint2 = rospy.ServiceProxy(stSrv, SrvJointState)
        except (rospy.ServiceException, IOError), e:
            rospy.loginfo ("5B FAILED %s: %s"%(stSrv,e))

        stSrv = 'srvSetPosition_'+self.names[1]
        rospy.wait_for_service(stSrv)
        try:
            self.setPosition_joint2 = rospy.ServiceProxy(stSrv, SrvJointState)
        except (rospy.ServiceException, IOError), e:
            rospy.loginfo ("5B FAILED %s: %s"%(stSrv,e))

        stSrv = 'srvSetPositionAtVel_'+self.names[1]
        rospy.wait_for_service(stSrv)
        try:
            self.setPositionAtVel_joint2 = rospy.ServiceProxy(stSrv, SrvJointState)
        except (rospy.ServiceException, IOError), e:
            rospy.loginfo ("5B FAILED %s: %s"%(stSrv,e))

        stSrv = 'srvPark_'+self.names[1]
        rospy.wait_for_service(stSrv)
        try:
            self.park_joint2 = rospy.ServiceProxy(stSrv, SrvJointState)
        except (rospy.ServiceException, IOError), e:
            rospy.loginfo ("5B FAILED %s: %s"%(stSrv,e))

        self.initializedServices = True
        

    # Clip the tool to the min of hardware & software radius limits.
    def ClipPtToRadius(self, pt):
        (pt.x,pt.y) = self.ClipXyToRadius(pt.x,pt.y)
        return pt
    
    def ClipXyToRadius(self, x, y):
        r = N.sqrt(x*x+y*y)
        rLimit = N.min(self.radiusMovement, self.radiusReachable)
        
        if rLimit < r:
            angle = N.arctan2(y,x)
            x = rLimit * N.cos(angle)
            y = rLimit * N.sin(angle)
            #rospy.logwarn('5B CLIPPING x,y to %s' % [x,y])

        return x,y
                
                
    # Close the kinematics chain:  Get thetas 1,2,3,4 from thetas 1,2
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
        
        #s13 = N.sin(angle1+angle3)
        #c13 = N.cos(angle1+angle3)
        
        #s24 = N.sin(angle2+angle4)
        #c24 = N.cos(angle2+angle4)
        
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

        #rospy.loginfo ('angle1=%s, angle2=%s' % (angle1,angle2))
        #rospy.loginfo ('Lc=%s, Ld=%s, tc=%s, td=%s, te=%s, tf=%s, tg=%s, th=%s' % (Lc, Ld, tc, td, te, tf, tg, th))
        #rospy.loginfo ('angle1=%s, angle2=%s, angle3=%s, angle4=%s' % (angle1, angle2, angle3, angle4))
        #rospy.loginfo ('j3x=%s, j3y=%s, j4x=%s, j4y=%s, jEx=%s, jEy=%s, jFx=%s, jFy=%s' % (j3x, j3y, j4x, j4y, jEx, jEy, jFx, jFy))
        
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
    
    
    # Get1234FromXY()
    # Inverse Kinematics: Get thetas (1,2,3,4) from the (x,y) end-effector position,
    # where (x,y) is in the coordinate frame of the workspace center,
    # and (angle1,angle2)==(0,0) maps to (x,y)==(0,0).
    #
    def Get1234FromXY (self, x0, y0):
        x0,y0 = self.ClipXyToRadius(x0, y0)
        
        # Flip the axes if necessary.
        x = x0
        y = y0
        
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
        
        q1 = [ \
                N.arctan2((-x*(x*self.L1sq+x3rd-x*self.L3sq+x*ysq+(2.0*xsq*self.L3sq*ysq+2.0*xsq*self.L1sq*ysq+2.0*ysq*self.L3sq*self.L1sq-2.0*xsq*y4th-x4th*ysq-ysq*self.L1**4.0-ysq*self.L3**4.0+2.0*y4th*self.L3sq+2.0*y4th*self.L1sq-y6th)**0.5)/(ysq+xsq)+self.L1sq-self.L3sq+ysq+xsq)/self.L1/y, \
                          (x*self.L1sq+x3rd-x*self.L3sq+x*ysq+(2.0*xsq*self.L3sq*ysq+2.0*xsq*self.L1sq*ysq+2.0*ysq*self.L3sq*self.L1sq-2.0*xsq*y4th-x4th*ysq-ysq*self.L1**4.0-ysq*self.L3**4.0+2.0*y4th*self.L3sq+2.0*y4th*self.L1sq-y6th)**0.5)/(ysq+xsq)/self.L1), \
                N.arctan2((-x*(x*self.L1sq+x3rd-x*self.L3sq+x*ysq-(2.0*xsq*self.L3sq*ysq+2.0*xsq*self.L1sq*ysq+2.0*ysq*self.L3sq*self.L1sq-2.0*xsq*y4th-x4th*ysq-ysq*self.L1**4.0-ysq*self.L3**4.0+2.0*y4th*self.L3sq+2.0*y4th*self.L1sq-y6th)**0.5)/(ysq+xsq)+self.L1sq-self.L3sq+ysq+xsq)/self.L1/y, \
                          (x*self.L1sq+x3rd-x*self.L3sq+x*ysq-(2.0*xsq*self.L3sq*ysq+2.0*xsq*self.L1sq*ysq+2.0*ysq*self.L3sq*self.L1sq-2.0*xsq*y4th-x4th*ysq-ysq*self.L1**4.0-ysq*self.L3**4.0+2.0*y4th*self.L3sq+2.0*y4th*self.L1sq-y6th)**0.5)/(ysq+xsq)/self.L1) \
              ]
        
        q3 = [ \
                N.arctan2((-2.0*y*x*self.L1sq+1.0/2.0*(y**3.0+xsq*y+y*self.L1sq-y*self.L3sq)*(x*self.L1sq+x3rd-x*self.L3sq+x*ysq+(2.0*xsq*self.L3sq*ysq+2.0*xsq*self.L1sq*ysq+2.0*ysq*self.L3sq*self.L1sq-2.0*xsq*y4th-x4th*ysq-ysq*self.L1**4.0-ysq*self.L3**4.0+2.0*y4th*self.L3sq+2.0*y4th*self.L1sq-y6th)**0.5)/(ysq+xsq))/(-self.L3*self.L1*x*(x*self.L1sq+x3rd-x*self.L3sq+x*ysq+(2.0*xsq*self.L3sq*ysq+2.0*xsq*self.L1sq*ysq+2.0*ysq*self.L3sq*self.L1sq-2.0*xsq*y4th-x4th*ysq-ysq*self.L1**4.0-ysq*self.L3**4.0+2.0*y4th*self.L3sq+2.0*y4th*self.L1sq-y6th)**0.5)/(ysq+xsq)+self.L3*self.L1**3.0-self.L3**3.0*self.L1+self.L3*self.L1*ysq+xsq*self.L3*self.L1), \
                          1.0/2.0*(xsq-self.L3sq+ysq-self.L1sq)/self.L1/self.L3), \
                N.arctan2((-2.0*y*x*self.L1sq+1.0/2.0*(y**3.0+xsq*y+y*self.L1sq-y*self.L3sq)*(x*self.L1sq+x3rd-x*self.L3sq+x*ysq-(2.0*xsq*self.L3sq*ysq+2.0*xsq*self.L1sq*ysq+2.0*ysq*self.L3sq*self.L1sq-2.0*xsq*y4th-x4th*ysq-ysq*self.L1**4.0-ysq*self.L3**4.0+2.0*y4th*self.L3sq+2.0*y4th*self.L1sq-y6th)**0.5)/(ysq+xsq))/(-self.L3*self.L1*x*(x*self.L1sq+x3rd-x*self.L3sq+x*ysq-(2.0*xsq*self.L3sq*ysq+2.0*xsq*self.L1sq*ysq+2.0*ysq*self.L3sq*self.L1sq-2.0*xsq*y4th-x4th*ysq-ysq*self.L1**4.0-ysq*self.L3**4.0+2.0*y4th*self.L3sq+2.0*y4th*self.L1sq-y6th)**0.5)/(ysq+xsq)+self.L3*self.L1**3.0-self.L3**3.0*self.L1+self.L3*self.L1*ysq+xsq*self.L3*self.L1), \
                          1.0/2.0*(xsq-self.L3sq+ysq-self.L1sq)/self.L1/self.L3) \
              ]
        
        q2 = [ \
                N.arctan2((xsq-2.0*self.L0*x+self.L0sq-self.L4sq+ysq+self.L2sq+1.0/2.0*(-2.0*x+2.0*self.L0)*(-self.L2sq*self.L0+x*self.L2sq-self.L0**3.0+x*ysq-ysq*self.L0-3*self.L0*xsq+self.L4sq*self.L0+x3rd-x*self.L4sq+3*self.L0sq*x+(-4.0*self.L2sq*self.L0*x*ysq-4.0*x*ysq*self.L4sq*self.L0-2.0*xsq*y4th-x4th*ysq+2.0*xsq*self.L2sq*ysq+2.0*self.L2sq*self.L0sq*ysq-6.0*xsq*ysq*self.L0sq+2.0*xsq*ysq*self.L4sq+4.0*x3rd*ysq*self.L0+4.0*x*y4th*self.L0+4.0*self.L0**3.0*x*ysq+2.0*ysq*self.L0sq*self.L4sq+2.0*ysq*self.L4sq*self.L2sq-2.0*y4th*self.L0sq-self.L0**4.0*ysq+2.0*y4th*self.L4sq-ysq*self.L2**4.0-ysq*self.L4**4.0+2.0*y4th*self.L2sq-y6th)**0.5)/(ysq+self.L0sq+xsq-2.0*self.L0*x))/self.L2/y, \
                          (-self.L2sq*self.L0+x*self.L2sq-self.L0**3.0+x*ysq-ysq*self.L0-3*self.L0*xsq+self.L4sq*self.L0+x3rd-x*self.L4sq+3*self.L0sq*x+(-4.0*self.L2sq*self.L0*x*ysq-4.0*x*ysq*self.L4sq*self.L0-2.0*xsq*y4th-x4th*ysq+2.0*xsq*self.L2sq*ysq+2.0*self.L2sq*self.L0sq*ysq-6.0*xsq*ysq*self.L0sq+2.0*xsq*ysq*self.L4sq+4.0*x3rd*ysq*self.L0+4.0*x*y4th*self.L0+4.0*self.L0**3.0*x*ysq+2.0*ysq*self.L0sq*self.L4sq+2.0*ysq*self.L4sq*self.L2sq-2.0*y4th*self.L0sq-self.L0**4.0*ysq+2.0*y4th*self.L4sq-ysq*self.L2**4.0-ysq*self.L4**4.0+2.0*y4th*self.L2sq-y6th)**0.5)/(ysq+self.L0sq+xsq-2.0*self.L0*x)/self.L2), \
                N.arctan2((xsq-2.0*self.L0*x+self.L0sq-self.L4sq+ysq+self.L2sq+1.0/2.0*(-2.0*x+2.0*self.L0)*(-self.L2sq*self.L0+x*self.L2sq-self.L0**3.0+x*ysq-ysq*self.L0-3*self.L0*xsq+self.L4sq*self.L0+x3rd-x*self.L4sq+3*self.L0sq*x-(-4.0*self.L2sq*self.L0*x*ysq-4.0*x*ysq*self.L4sq*self.L0-2.0*xsq*y4th-x4th*ysq+2.0*xsq*self.L2sq*ysq+2.0*self.L2sq*self.L0sq*ysq-6.0*xsq*ysq*self.L0sq+2.0*xsq*ysq*self.L4sq+4.0*x3rd*ysq*self.L0+4.0*x*y4th*self.L0+4.0*self.L0**3.0*x*ysq+2.0*ysq*self.L0sq*self.L4sq+2.0*ysq*self.L4sq*self.L2sq-2.0*y4th*self.L0sq-self.L0**4.0*ysq+2.0*y4th*self.L4sq-ysq*self.L2**4.0-ysq*self.L4**4.0+2.0*y4th*self.L2sq-y6th)**0.5)/(ysq+self.L0sq+xsq-2.0*self.L0*x))/self.L2/y, \
                          (-self.L2sq*self.L0+x*self.L2sq-self.L0**3.0+x*ysq-ysq*self.L0-3*self.L0*xsq+self.L4sq*self.L0+x3rd-x*self.L4sq+3*self.L0sq*x-(-4.0*self.L2sq*self.L0*x*ysq-4.0*x*ysq*self.L4sq*self.L0-2.0*xsq*y4th-x4th*ysq+2.0*xsq*self.L2sq*ysq+2.0*self.L2sq*self.L0sq*ysq-6.0*xsq*ysq*self.L0sq+2.0*xsq*ysq*self.L4sq+4.0*x3rd*ysq*self.L0+4.0*x*y4th*self.L0+4.0*self.L0**3.0*x*ysq+2.0*ysq*self.L0sq*self.L4sq+2.0*ysq*self.L4sq*self.L2sq-2.0*y4th*self.L0sq-self.L0**4.0*ysq+2.0*y4th*self.L4sq-ysq*self.L2**4.0-ysq*self.L4**4.0+2.0*y4th*self.L2sq-y6th)**0.5)/(ysq+self.L0sq+xsq-2.0*self.L0*x)/self.L2) \
              ]

        q4 = [ \
                N.arctan2((2.0*y*self.L2sq*self.L0-2.0*y*self.L2sq*x+1.0/2.0*(xsq*y-2.0*y*self.L0*x+y*self.L0sq-y*self.L4sq+y**3.0+y*self.L2sq)*(-self.L2sq*self.L0+x*self.L2sq-self.L0**3.0+x*ysq-ysq*self.L0-3*self.L0*xsq+self.L4sq*self.L0+x3rd-x*self.L4sq+3*self.L0sq*x+(-4.0*self.L2sq*self.L0*x*ysq-4.0*x*ysq*self.L4sq*self.L0-2.0*xsq*y4th-x4th*ysq+2.0*xsq*self.L2sq*ysq+2.0*self.L2sq*self.L0sq*ysq-6.0*xsq*ysq*self.L0sq+2.0*xsq*ysq*self.L4sq+4.0*x3rd*ysq*self.L0+4.0*x*y4th*self.L0+4.0*self.L0**3.0*x*ysq+2.0*ysq*self.L0sq*self.L4sq+2.0*ysq*self.L4sq*self.L2sq-2.0*y4th*self.L0sq-self.L0**4.0*ysq+2.0*y4th*self.L4sq-ysq*self.L2**4.0-ysq*self.L4**4.0+2.0*y4th*self.L2sq-y6th)**0.5)/(ysq+self.L0sq+xsq-2.0*self.L0*x))/(self.L2*self.L4*xsq-2.0*self.L2*self.L4*self.L0*x+self.L2*self.L4*self.L0sq-self.L2*self.L4**3.0+self.L2*self.L4*ysq+self.L2**3.0*self.L4+1.0/2.0*(-2.0*self.L2*self.L4*x+2.0*self.L2*self.L4*self.L0)*(-self.L2sq*self.L0+x*self.L2sq-self.L0**3.0+x*ysq-ysq*self.L0-3*self.L0*xsq+self.L4sq*self.L0+x3rd-x*self.L4sq+3*self.L0sq*x+(-4.0*self.L2sq*self.L0*x*ysq-4.0*x*ysq*self.L4sq*self.L0-2.0*xsq*y4th-x4th*ysq+2.0*xsq*self.L2sq*ysq+2.0*self.L2sq*self.L0sq*ysq-6.0*xsq*ysq*self.L0sq+2.0*xsq*ysq*self.L4sq+4.0*x3rd*ysq*self.L0+4.0*x*y4th*self.L0+4.0*self.L0**3.0*x*ysq+2.0*ysq*self.L0sq*self.L4sq+2.0*ysq*self.L4sq*self.L2sq-2.0*y4th*self.L0sq-self.L0**4.0*ysq+2.0*y4th*self.L4sq-ysq*self.L2**4.0-ysq*self.L4**4.0+2.0*y4th*self.L2sq-y6th)**0.5)/(ysq+self.L0sq+xsq-2.0*self.L0*x)), \
                          1.0/2.0*(xsq-2.0*self.L0*x+self.L0sq-self.L4sq+ysq-self.L2sq)/self.L2/self.L4), \
                N.arctan2((2.0*y*self.L2sq*self.L0-2.0*y*self.L2sq*x+1.0/2.0*(xsq*y-2.0*y*self.L0*x+y*self.L0sq-y*self.L4sq+y**3.0+y*self.L2sq)*(-self.L2sq*self.L0+x*self.L2sq-self.L0**3.0+x*ysq-ysq*self.L0-3*self.L0*xsq+self.L4sq*self.L0+x3rd-x*self.L4sq+3*self.L0sq*x-(-4.0*self.L2sq*self.L0*x*ysq-4.0*x*ysq*self.L4sq*self.L0-2.0*xsq*y4th-x4th*ysq+2.0*xsq*self.L2sq*ysq+2.0*self.L2sq*self.L0sq*ysq-6.0*xsq*ysq*self.L0sq+2.0*xsq*ysq*self.L4sq+4.0*x3rd*ysq*self.L0+4.0*x*y4th*self.L0+4.0*self.L0**3.0*x*ysq+2.0*ysq*self.L0sq*self.L4sq+2.0*ysq*self.L4sq*self.L2sq-2.0*y4th*self.L0sq-self.L0**4.0*ysq+2.0*y4th*self.L4sq-ysq*self.L2**4.0-ysq*self.L4**4.0+2.0*y4th*self.L2sq-y6th)**0.5)/(ysq+self.L0sq+xsq-2.0*self.L0*x))/(self.L2*self.L4*xsq-2.0*self.L2*self.L4*self.L0*x+self.L2*self.L4*self.L0sq-self.L2*self.L4**3.0+self.L2*self.L4*ysq+self.L2**3.0*self.L4+1.0/2.0*(-2.0*self.L2*self.L4*x+2.0*self.L2*self.L4*self.L0)*(-self.L2sq*self.L0+x*self.L2sq-self.L0**3.0+x*ysq-ysq*self.L0-3*self.L0*xsq+self.L4sq*self.L0+x3rd-x*self.L4sq+3*self.L0sq*x-(-4.0*self.L2sq*self.L0*x*ysq-4.0*x*ysq*self.L4sq*self.L0-2.0*xsq*y4th-x4th*ysq+2.0*xsq*self.L2sq*ysq+2.0*self.L2sq*self.L0sq*ysq-6.0*xsq*ysq*self.L0sq+2.0*xsq*ysq*self.L4sq+4.0*x3rd*ysq*self.L0+4.0*x*y4th*self.L0+4.0*self.L0**3.0*x*ysq+2.0*ysq*self.L0sq*self.L4sq+2.0*ysq*self.L4sq*self.L2sq-2.0*y4th*self.L0sq-self.L0**4.0*ysq+2.0*y4th*self.L4sq-ysq*self.L2**4.0-ysq*self.L4**4.0+2.0*y4th*self.L2sq-y6th)**0.5)/(ysq+self.L0sq+xsq-2.0*self.L0*x)), \
                          1.0/2.0*(xsq-2.0*self.L0*x+self.L0sq-self.L4sq+ysq-self.L2sq)/self.L2/self.L4) \
              ]

        #rospy.logwarn ('5B q1=%s' % q1)
        #rospy.logwarn ('5B q2=%s' % q2)
        #rospy.logwarn ('5B q3=%s' % q3)
        #rospy.logwarn ('5B q4=%s' % q4)
                
        
        #(angle1,angle2) = (q1[1]-self.q1CenterE,q2[0]-self.q2CenterE)
        
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
        
        
    def GetXyFrom12 (self, angle1, angle2):
        (angle1, angle2, angle3, angle4, x, y) = self.Get1234xyFrom12(angle1, angle2)
        return (x, y)
    
    
    def GetStageState_callback(self, reqStageState):
        state = self.GetStageState()
        return state
    
    def GetStageState (self):
        while not self.initializedServices:
            rospy.sleep(0.1)
            
        x = 0.0
        y = 0.0
        
#        with self.lock:
#            try:
#                self.jointstate1 = self.getState_joint1()
#                self.jointstate2 = self.getState_joint2()
#            except (rospy.ServiceException, IOError), e:
#                rospy.logwarn ("5B FAILED %s"%e)
#                self.jointstate1 = None
#                self.jointstate2 = None


        rvStageState = SrvFrameStateResponse()
        if (self.jointstate1 is not None) and (self.jointstate2 is not None):                    
            #rospy.loginfo ('5B self.jointstate1=%s' % self.jointstate1)
            #rospy.loginfo ('5B self.jointstate2=%s' % self.jointstate2)
            #(x,y) = self.GetXyFrom12 (self.jointstate1.position, self.jointstate2.position)
            
            # Transform the point into the requested frame.
            pt = Point()
            pt.x = self.ptEeSense.x + self.ptOffsetSense.x 
            pt.y = self.ptEeSense.y + self.ptOffsetSense.y 
            pt.z = self.ptEeSense.z + self.ptOffsetSense.z 
            pt.point = self.ClipPtToRadius(pt.point)
            
            #rvStageState.state.header.stamp = self.time
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
            
        #rospy.loginfo ('5B SetStageState_callback req=%s' % reqStageState)
        
        #rospy.loginfo ('5B SetStageState_callback Request x,y=%s' % ([reqStageState.state.pose.position.x, 
        #                                                                 reqStageState.state.pose.position.y]))
        #(angle1,angle2,angle3,angle4) = self.Get1234FromXY(reqStageState.state.pose.position.x-self.xCenter, 
        #                                   reqStageState.state.pose.position.y-self.yCenter)
        self.ptsToolRefExternal = PointStamped(Header(frame_id=reqStageState.state.header.frame_id),
                                               Point(x=reqStageState.state.pose.position.x,
                                                     y=reqStageState.state.pose.position.y,
                                                     z=reqStageState.state.pose.position.z))
        try:
            self.tfrx.waitForTransform("Stage", self.ptsToolRefExternal.header.frame_id, rospy.Time(), rospy.Duration(1.0))
            self.ptsToolRef = self.tfrx.transformPoint('Stage', self.ptsToolRefExternal)
        except tf.Exception:
            rospy.logwarn ('5B Exception in SetStageState_callback()')
            pass

        #rospy.loginfo('5B ptsToolRef=[%0.2f, %0.2f], [%0.2f, %0.2f]' % (self.ptsToolRef.point.x,self.ptsToolRef.point.y,self.ptsToolRefExternal.point.x,self.ptsToolRefExternal.point.y))
        if reqStageState.speed is not None:
            self.speedCommandTool = reqStageState.speed # Requested speed for positioning.
        else:
            self.speedCommandTool = self.speedStageMax
        #rospy.logwarn('5B reqStageState.speed=%s' % reqStageState.speed)

        rvStageState = SrvFrameStateResponse()
        rvStageState.state.pose.position.x = reqStageState.state.pose.position.x
        rvStageState.state.pose.position.y = reqStageState.state.pose.position.y
        rvStageState.state.pose.position.z = reqStageState.state.pose.position.z

        return rvStageState
    

    def SignalInput_callback (self, srvSignal):
        #rospy.logwarn('A')
        rv = SrvSignalResponse()
        rv.success = False
        self.ptsToolRefExternal = srvSignal.pts #self.pattern.points[self.iPoint]
        try:
            #rospy.logwarn('B')
            self.tfrx.waitForTransform('Stage', self.ptsToolRefExternal.header.frame_id, self.time, rospy.Duration(1.0))
            #rospy.logwarn('C')
            self.ptsToolRef = self.tfrx.transformPoint('Stage', self.ptsToolRefExternal)
            #rospy.logwarn('D')

            try:
                #rospy.logwarn('E')
                self.speedCommandTool = self.speedStageMax #self.speedCommandTool #5.0 * (1.0/self.dtPoint) # Robot travels to target at twice the target speed.
                #rospy.logwarn('F')
                rv.success = True
                #rospy.logwarn ('pt(%d)=[%0.2f,%0.2f]'%(self.iCount,self.ptsToolRefExternal.point.x,self.ptsToolRefExternal.point.y))
            except AttributeError:
                pass
            #rospy.logwarn ('5B signal pt=%s' % [self.ptToolRef.x,self.ptToolRef.y])
        except tf.Exception:
            pass
        #rospy.logwarn('G')
        self.iCount += 1
        return rv


    # Create a vector with direction of ptDir, and magnitude of ptMag.
    def ScaleVecToMag (self, ptDir, ptMag):
        magPtDir = N.linalg.norm([ptDir.x, ptDir.y, ptDir.z])
        magPtMag = N.linalg.norm([ptMag.x, ptMag.y, ptMag.z])
        
        ptNew = Point (x=magPtMag/magPtDir*ptDir.x,
                       y=magPtMag/magPtDir*ptDir.y,
                       z=magPtMag/magPtDir*ptDir.z)
        return ptNew
    

    def ClipPtMag (self, pt, magMax):
        magPt = N.linalg.norm([pt.x, pt.y, pt.z])
        if magPt > magMax:
            r = magMax / magPt
        else:
            r = 1.0
            
        return Point(r*pt.x, r*pt.y, r*pt.z)
            
        
    def EndEffectorOffset_callback(self, ptOffset):
        self.ptOffsetSense = ptOffset #Point(0,0,0)#
        #rospy.logwarn ('5B ptOffset=%s' % ptOffset)
        
        
    def HomeStage_callback(self, reqStageState):
        while not self.initialized:
            rospy.sleep(0.1)
            
        with self.lock:
            try:
                self.park_joint1()
                self.park_joint2()
            except (rospy.ServiceException, IOError), e:
                rospy.logwarn ("5B FAILED %s"%e)
                
        rvStageState = SrvFrameStateResponse()
        rospy.loginfo ('5B HomeStage_callback rvStageState=%s' % rvStageState)
        return rvStageState
        
        
    def Calibrate_callback(self, req):
        # Convert parking coordinates to angles.
        self.xPark = self.xPark
        self.yPark = self.yPark
        with self.lock:
            [q1park,q2park,q3park,q4park] = self.Get1234FromXY(self.xPark, self.yPark)

        q1origin = self.q1CenterI # From the index switch
        q2origin = self.q2CenterI # From the index switch

        rospy.loginfo ('5B Center = %s, %s' % (self.xCenter, self.yCenter))
        rospy.loginfo ('5B Calibrating: q1origin=%s, q1park=%s, q2origin=%s, q2park=%s' % (q1origin, q1park, q2origin, q2park))
        with self.lock:
            rv = self.calibrate_joint1(NEGATIVE, q1origin, q1park, True)
            rospy.loginfo ('5B Calibrated joint1')
            q1index = rv.position
            rv = self.calibrate_joint2(POSITIVE, q2origin, q2park, True)
            rospy.loginfo ('5B Calibrated joint2')
            q2index = rv.position

        rvStageState = self.GetStageState()
        rospy.loginfo ('5B Calibrate_callback rvStageState=%s' % rvStageState)
        return rvStageState
    

    def SendTransforms(self):  
        if self.initialized:
            with self.lock:
                try:
                    self.jointstate1 = self.getState_joint1()
                    self.jointstate2 = self.getState_joint2()
                except (rospy.ServiceException, IOError), e:
                    rospy.logwarn ("5B FAILED %s"%e)
                    self.jointstate1 = None
                    self.jointstate2 = None
                #rospy.logwarn('5B [j1,j2]=%s' % [self.jointstate1.position,self.jointstate2.position])
                    

            if (self.jointstate1 is not None) and (self.jointstate2 is not None):                    
                #rospy.loginfo ('5B self.jointstate1=%s' % self.jointstate1)
                (angle1,angle2,angle3,angle4, self.ptEeSense.x, self.ptEeSense.y) = self.Get1234xyFrom12(self.jointstate1.position, self.jointstate2.position)
                #rospy.logwarn('5B compare [xc,yc]-[xa,ya]=%s' % [self.ptEeCommand.x-self.ptEeSense.x,self.ptEeCommand.y-self.ptEeSense.y])
                #rospy.logwarn('5B actual [x,y]=%s' % [self.ptEeSense.x,self.ptEeSense.y])

                #with self.lock:
                #    (angle1a,angle2a,angle3a,angle4a) = self.Get1234FromXY(self.state.pose.position.x, self.state.pose.position.y)
                #rospy.loginfo ('5B xy=%s, angle1-angle1a=%0.5f, angle2-angle2a=%0.5f, angle3-angle3a=%0.5f, angle4-angle4a=%0.5f' % ([self.state.pose.position.x, self.state.pose.position.y], angle1-angle1a, angle2-angle2a, angle3-angle3a, angle4-angle4a))
                
                # Move the crossover point from pi/-pi to 0/2pi
                #angle1 = angle1 % (2.0*N.pi) 
                #angle2 = angle2 % (2.0*N.pi)
    
                # Publish the joint states (for rviz, etc)    
                self.js.header.seq = self.js.header.seq + 1
                self.js.header.stamp.secs = self.time #rospy.get_time()
                self.js.position = [angle1,angle2,angle3,angle4]
                self.pubJointState.publish(self.js)
    
                state = MsgFrameState()
                state.header.stamp = self.time
                state.header.frame_id = 'Stage'
                state.pose.position.x = self.ptEeSense.x
                state.pose.position.y = self.ptEeSense.y
                qEE = tf.transformations.quaternion_from_euler(0.0, 0.0, angle1+self.q1CenterE+angle3)
                state.pose.orientation.x = qEE[0]
                state.pose.orientation.y = qEE[1]
                state.pose.orientation.z = qEE[2]
                state.pose.orientation.w = qEE[3]
                self.pubEndEffector.publish (state)
                #rospy.loginfo ('5B publish state=%s' % state)
            
        
        
                # Publish the link transforms.
                
        
                self.tfbx.sendTransform((-self.L3/2, -246.31423, 0.0), 
                                        tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                                        state.header.stamp,
                                        "link0",     # child
                                        "Stage"      # parent
                                        )

                self.tfbx.sendTransform((0.0, 0.0, 0.0), 
                                        tf.transformations.quaternion_from_euler(0.0, 0.0, angle1+self.q1CenterE),
                                        state.header.stamp,
                                        "link1",     # child
                                        "link0"      # parent
                                        )
                self.tfbx.sendTransform((self.L1, 0.0, 0.0), 
                                        tf.transformations.quaternion_from_euler(0.0, 0.0, angle3),
                                        state.header.stamp,
                                        "link3",     # child
                                        "link1"      # parent
                                        )
        
                self.tfbx.sendTransform((self.L3, 0.0, 0.0), 
                                        tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                                        state.header.stamp,
                                        "link5",     # child
                                        "link3"      # parent
                                        )

                self.tfbx.sendTransform((self.L0, 0.0, 0.0), 
                                        tf.transformations.quaternion_from_euler(0.0, 0.0, angle2+self.q2CenterE),
                                        state.header.stamp,
                                        "link2",     # child
                                        "link0"      # parent
                                        )
                self.tfbx.sendTransform((self.L2, 0.0, 0.0), 
                                        tf.transformations.quaternion_from_euler(0.0, 0.0, angle4),
                                        state.header.stamp,
                                        "link4",     # child
                                        "link2"      # parent
                                        )
                
                
                # Frame EndEffector
                self.tfbx.sendTransform((state.pose.position.x, state.pose.position.y, state.pose.position.z), 
                                        qEE,
                                        state.header.stamp,
                                        "EndEffector",     # child
                                        "Stage"      # parent
                                        )
                # Frame Target
                if self.ptsToolRef is not None:
                    #rospy.logwarn('marker at [%0.2f,%0.2f]' % (self.ptsToolRef.point.x,self.ptsToolRef.point.y))
                    markerTarget = Marker(header=Header(stamp = state.header.stamp,
                                                        frame_id='Stage'),
                                          ns='target',
                                          id=0,
                                          type=2, #SPHERE,
                                          action=0,
                                          pose=Pose(position=Point(x=self.ptsToolRef.point.x, 
                                                                   y=self.ptsToolRef.point.y, 
                                                                   z=self.ptsToolRef.point.z)),
                                          scale=Vector3(x=3.0,
                                                        y=3.0,
                                                        z=3.0),
                                          color=ColorRGBA(a=0.5,
                                                          r=1.0,
                                                          g=0.1,
                                                          b=0.1),
                                          lifetime=rospy.Duration(1.0))
                    self.pubMarker.publish(markerTarget)

#                    markerToolOffset   = Marker(header=state.header,
#                                          ns='markers',
#                                          id=1,
#                                          type=2, #SPHERE,
#                                          action=0,
#                                          pose=Pose(position=Point(x=state.pose.position.x+self.ptOffsetSense.x, 
#                                                                   y=state.pose.position.y+self.ptOffsetSense.y, 
#                                                                   z=state.pose.position.z+self.ptOffsetSense.z)),
#                                          scale=Vector3(x=6.0,
#                                                        y=6.0,
#                                                        z=6.0),
#                                          color=ColorRGBA(a=0.5,
#                                                          r=0.5,
#                                                          g=0.5,
#                                                          b=0.5),
#                                          lifetime=rospy.Duration(0.1))
                    markerToolOffset   = Marker(header=state.header,
                                                ns='tooloffset',
                                                id=1,
                                                type=0, #ARROW,
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
                                                        Point(x=state.pose.position.x+self.ptOffsetSense.x, 
                                                              y=state.pose.position.y+self.ptOffsetSense.y, 
                                                              z=state.pose.position.z+self.ptOffsetSense.z)])
                    self.pubMarker.publish(markerToolOffset)



    # SendTargetCommand()
    #   Updates the motor command with the current target.
    #
    def SendTargetCommand(self):
        #rospy.loginfo ('5B ptToolRef=%s' % self.ptToolRef)
        if self.ptsToolRef is not None:
            self.speedStageMax = rospy.get_param('fivebar/speed_max', 200.0)
            self.radiusMovement = rospy.get_param('arena/radius_movement', 25.4)
            
            # Clip the target point to the arena bounds.
            #self.ptToolRefClipped = self.ClipPtToRadius(self.ptsToolRefExternal.point)

            # Compute various vectors.
            self.ptContourSense.x = self.ptEeSense.x + self.ptOffsetSense.x
            self.ptContourSense.y = self.ptEeSense.y + self.ptOffsetSense.y
            self.ptContourError.x = self.ptsToolRef.point.x - self.ptContourSense.x #- self.ptOffsetSense.x
            self.ptContourError.y = self.ptsToolRef.point.y - self.ptContourSense.y #- self.ptOffsetSense.y
            self.ptToolRefError.x = self.ptsToolRef.point.x - self.ptEeSense.x
            self.ptToolRefError.y = self.ptsToolRef.point.y - self.ptEeSense.y

            # Get the end-effector ref coordinates.
            # Option A:
            #ptRef = self.ScaleVecToMag(self.ptToolRefError, self.ptOffsetSense)
            #self.ptEeRef.x = self.ptsToolRef.point.x+ptRef.x
            #self.ptEeRef.y = self.ptsToolRef.point.y+ptRef.y

            # Option B: works ok.
            #kTest = rospy.get_param('fivebar/kTest', 0.0)
            #self.ptEeRef.x = self.ptsToolRef.point.x - self.ptOffsetSense.x + kTest*self.ptContourError.x
            #self.ptEeRef.y = self.ptsToolRef.point.y - self.ptOffsetSense.y + kTest*self.ptContourError.y

            # Option C: best so far.
            ptRef = self.ScaleVecToMag(self.ptContourError, self.ptOffsetSense)
            self.ptEeRef.x = self.ptsToolRef.point.x + ptRef.x
            self.ptEeRef.y = self.ptsToolRef.point.y + ptRef.y
            
            
            # PID Gains & Parameters.
            kP = rospy.get_param('fivebar/kP', 0.1)
            kI = rospy.get_param('fivebar/kI', 0.0)
            kD = rospy.get_param('fivebar/kD', 0.0)
            maxI = rospy.get_param('fivebar/maxI', 40.0)
            kWindup = rospy.get_param('fivebar/kWindup', 0.0)

            # PID control of the end-effector error.
            self.ptEeError.x = self.ptEeRef.x - self.ptEeSense.x
            self.ptEeError.y = self.ptEeRef.y - self.ptEeSense.y
            self.ptEeIerror.x = self.ptEeIerror.x + self.ptEeError.x
            self.ptEeIerror.y = self.ptEeIerror.y + self.ptEeError.y
            self.ptEeDerror.x = self.ptEeError.x - self.ptEeErrorPrev.x
            self.ptEeDerror.y = self.ptEeError.y - self.ptEeErrorPrev.y
            ptPID = Point(kP*self.ptEeError.x + kI*self.ptEeIerror.x + kD*self.ptEeDerror.x,
                          kP*self.ptEeError.y + kI*self.ptEeIerror.y + kD*self.ptEeDerror.y,
                          0.0)
            
            # Anti-windup
            self.ptEeIerrorClipped = self.ClipPtMag (self.ptEeIerror, maxI)
            self.ptAntiwindup = Point(kWindup * (self.ptEeIerror.x - self.ptEeIerrorClipped.x),
                                      kWindup * (self.ptEeIerror.y - self.ptEeIerrorClipped.y),
                                      kWindup * (self.ptEeIerror.z - self.ptEeIerrorClipped.z))
            magP = N.linalg.norm([self.ptEeError.x, self.ptEeError.y])
            magI = N.linalg.norm([self.ptEeIerror.x, self.ptEeIerror.y])
            magD = N.linalg.norm([self.ptEeDerror.x, self.ptEeDerror.y])
            magPID = N.linalg.norm([ptPID.x, ptPID.y])
            #magPIDRaw = N.linalg.norm([ptPIDRaw.x, ptPIDRaw.y])
            #rospy.logwarn('[P,I,D]=[%0.2f,%0.2f,%0.2f], PID=%0.4f' % (magP,magI,magD, magPID))
            self.ptEeIerror.x -= self.ptAntiwindup.x
            self.ptEeIerror.y -= self.ptAntiwindup.y

            # Get the command for the hardware.            
            self.ptEeCommand.x = ptPID.x + self.ptEeSense.x
            self.ptEeCommand.y = ptPID.y + self.ptEeSense.y
            
            self.ptEeErrorPrev.x = self.ptEeError.x 
            self.ptEeErrorPrev.y = self.ptEeError.y 
            
            #rospy.logwarn ('5B ptEeCommand=%s' % [self.ptEeCommand.x, self.ptEeCommand.y])

            # Display a vector in rviz.
            ptBase = self.ptEeSense
            ptEnd = self.ptEeCommand
            markerCommand= Marker(header=Header(stamp = self.time,
                                                frame_id='Stage'),
                                  ns='command',
                                  id=1,
                                  type=0, #ARROW,
                                  action=0,
                                  scale=Vector3(x=0.1, # Shaft diameter
                                                y=0.2, # Head diameter
                                                z=0.0),
                                  color=ColorRGBA(a=0.9,
                                                  r=0.5,
                                                  g=1.0,
                                                  b=0.5),
                                  lifetime=rospy.Duration(1.0),
#                                      points=[Point(x=self.ptEeSense.x +self.ptsToolRef.point.x-self.ptEeSense.x-self.ptEeError.x-self.ptOffsetSense.x, 
#                                                    y=self.ptEeSense.y +self.ptsToolRef.point.y-self.ptEeSense.y-self.ptEeError.y-self.ptOffsetSense.y, 
#                                                    z=self.ptEeSense.z +self.ptsToolRef.point.z-self.ptEeSense.z-self.ptEeError.z-self.ptOffsetSense.z),
#                                              Point(x=self.ptEeCommand.x +self.ptsToolRef.point.x-self.ptEeSense.x-self.ptEeError.x-self.ptOffsetSense.x, 
#                                                    y=self.ptEeCommand.y +self.ptsToolRef.point.y-self.ptEeSense.y-self.ptEeError.y-self.ptOffsetSense.y, 
#                                                    z=self.ptEeCommand.z +self.ptsToolRef.point.z-self.ptEeSense.z-self.ptEeError.z-self.ptOffsetSense.z)])
                                  points=[ptBase, ptEnd])
            self.pubMarker.publish(markerCommand)

            
            # Get the desired positions for each joint.
            (angle1,angle2,angle3,angle4) = self.Get1234FromXY(self.ptEeCommand.x, 
                                               self.ptEeCommand.y)
            
    
            if (self.jointstate1 is not None) and (self.jointstate2 is not None):
                # Cheap and wrong way to convert mm/sec to radians/sec.  Should use Jacobian.                    
                speedMax = self.speedCommandTool * 0.0120 

                # Distribute the velocity over the two joints.
                dangle1 = N.abs(angle1-self.jointstate1.position) # Delta theta
                dangle2 = N.abs(angle2-self.jointstate2.position)
                dist = N.linalg.norm([dangle1,dangle2])
                if dist != 0.0:
                    scale1 = dangle1/dist
                    scale2 = dangle2/dist
                else:
                    scale1 = 0.5
                    scale2 = 0.5
                    
                v1 = scale1 * speedMax
                v2 = scale2 * speedMax
                
                #rospy.logwarn('5B speedMax=%s, v1,v2=%s' % (speedMax,[v1,v2]))
        
                
                with self.lock:
                    try:
                        #rospy.logwarn('%0.4f: dt=%0.4f, x,y=[%0.2f,%0.2f]' % (time.to_sec(),self.dt.to_sec(),self.ptsToolRef.point.x,self.ptsToolRef.point.y))
                        self.setPositionAtVel_joint1(Header(frame_id=self.names[0]), angle1, v1)
                        self.setPositionAtVel_joint2(Header(frame_id=self.names[1]), angle2, v2)
                    except (rospy.ServiceException, IOError), e:
                        rospy.logwarn ("5B FAILED %s"%e)

        
    def OnShutdown_callback(self):
        rospy.loginfo("5B Closed Fivebar device.")


        
    def Main(self):
        self.LoadServices()
        self.Calibrate_callback(None)
        self.initialized = True

        rospy.loginfo ('5B QMIN,QMAX: %s, %s, %s, %s' % (self.q1MinE, self.q1MaxE, self.q2MinE, self.q2MaxE))
        rospy.loginfo ('5B self.q1CenterEiz: %s, %s, %s' % (self.q1CenterE, self.q1CenterI, self.q1CenterZ))
        rospy.loginfo ('5B self.q2CenterEiz: %s, %s, %s' % (self.q2CenterE, self.q2CenterI, self.q2CenterZ))
        # Process messages forever.
        rosrate = rospy.Rate(100)

        while not rospy.is_shutdown():
            self.time = rospy.Time.now()
            self.dt = self.time - self.timePrev
            self.timePrev = self.time

            self.SendTransforms()
            self.SendTargetCommand()
            rosrate.sleep()
                
    

if __name__ == '__main__':
    try:
        fivebar = RosFivebar()
    except rospy.ROSInterruptException: 
        pass
    
    fivebar.Main()
    