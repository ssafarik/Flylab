#!/usr/bin/env python
import roslib; roslib.load_manifest('patterngen')
import rospy
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from patterngen.msg import MsgPatternGen
from patterngen.srv import SrvSignal
import numpy as N



class NullClass:
    pass
    
class PatternGenXY:

    def __init__(self):
        rospy.init_node('PatternGenXY')
        rospy.loginfo ("PatternGenXY name=%s", __name__)

        self.pattern = NullClass()
        self.pattern.mode = 'byshape'
        self.pattern.shape = 'circle'
        self.pattern.radius = 25.4
        self.pattern.points = []
        self.pattern.hz = 1.0
        self.pattern.count = 0
        self.iPoint = 0

        self.pts = PointStamped()

        self.hzPoint = 50
        self.dtPoint = rospy.Duration(1/self.hzPoint)
        self.ratePoint = rospy.Rate(self.hzPoint)
        rospy.loginfo('Point Output Rate (hz): %0.2f' % self.hzPoint)
        
        self.subPatternGen = rospy.Subscriber('PatternGen', MsgPatternGen, self.PatternGen_callback)
        
        # Load stage services.
        self.stSignalInput = 'signal_input'
        rospy.loginfo('Waiting for: '+self.stSignalInput)
        rospy.wait_for_service(self.stSignalInput)
        try:
            self.SignalOutput = rospy.ServiceProxy(self.stSignalInput, SrvSignal)
        except rospy.ServiceException, e:
            rospy.loginfo ("FAILED %s: %s"%(self.stSignalInput,e))
        rospy.loginfo('Passed: '+self.stSignalInput)
        
        


    def GetPointsConstant(self):
        nPoints = int(self.hzPoint/self.pattern.hz)
        points = [Point(x=self.pattern.radius, 
                        y=self.pattern.radius)] * nPoints  # [(r,r),(r,r),(r,r), ...]
        
        return points

    
    def GetPointsCircle(self):
        nPoints = int(self.hzPoint/self.pattern.hz)
        q = 0.0 #N.pi/2.0  # Starting position
        dq = 2.0*N.pi/nPoints
        r = self.pattern.radius

        points = []
        for i in range(nPoints):
            points.append(Point(x=r*N.cos(q), 
                                y=r*N.sin(q)))
            q = q + dq    # Step around the circle

        return points
    
        
    def GetPointsSquare(self):
        nPoints = int(self.hzPoint/self.pattern.hz)
        nPointsSide = int(nPoints / 4.0) # Points per side
        xmin = -self.pattern.radius / N.sqrt(2)
        xmax =  self.pattern.radius / N.sqrt(2)
        ymin = -self.pattern.radius / N.sqrt(2)
        ymax =  self.pattern.radius / N.sqrt(2)
        step = (xmax-xmin)/((nPointsSide+1)-1)

        points = []        
        x = xmin
        y = ymin
        
        for iSide in [0,1,2,3]:
            for i in range(nPointsSide):
                points.append(Point(x=x, y=y))

                if iSide==0:
                    dx =  step
                    dy =  0.0
                elif iSide==1:
                    dx =  0.0
                    dy =  step
                elif iSide==2:
                    
                    dx = -step
                    dy =  0.0
                elif iSide==3:
                    dx =  0.0
                    dy = -step
                
                x += dx
                y += dy
    
        return points
    

    def GetPointsFlylogo (self):
        flylogo = [[-4, 14],[-3, 11],[-7, 7],[-6, 5],[-7, 4],
                   [-9, 4],[-12, 6],[-10, 8],[-12, 10],[-10, 8],[-12, 6],[-9, 4],
                   [-13, 3],[-14, 0],[-11, 1.5],[-9, 4],
                   [-11, 0],
                   [-9, -4],[-11, -1.5],[-14, 0],[-13, -3],
                   [-9, -4],[-12, -6],[-10, -8],[-12, -10],[-10, -8],[-12, -6],[-9, -4],
                   [-7, -4],[-6, -5],[-7, -7],[-3, -11],[-4, -14],[-3, -11],[-7, -7],[-6, -5],
                   [-3, -7],[3, -13],[13, -15],[16, -12],[12, -6],[4, -2],
                   [0, -1],[-5, -4],[-2, 0],[-5, 4],[0, 1],[4, 2],[4, -2],[7,0],[4, 2],[12, 6],[16, 12],[13, 15],[3, 13],[-3, 7],[-6, 5],[-7, 7],[-3, 11],[-4, 14]]
        points = []

        for k in range(len(flylogo)):
            pt = Point(x=float(flylogo[k][0]) * self.pattern.radius / 20.0, 
                       y=float(flylogo[k][1]) * self.pattern.radius / 20.0) # 20.0 is the max radius of the pointlist above.
            points.append(pt)
           
        return points
    
        
    def GetPointsSpiral (self):
        nPoints = int(self.hzPoint/self.pattern.hz)
        rospy.logwarn('nPoints=%d' % nPoints)
        pitchSpiral = 2
        nRevolutionsPerPattern = 2 * 2 * pitchSpiral  # nCworCCW * nInOrOut * pitch
        nPointsPerRevolution = nPoints / nRevolutionsPerPattern
        q = 2.0 * N.pi * N.random.random()
        dq = 2.0 * N.pi/nPointsPerRevolution
        
        rmax = self.pattern.radius
        rmin = 0.10
        r = rmin
        dr = (rmax-rmin) / (pitchSpiral * nPointsPerRevolution)

        points = []
        for iCWorCCW in range(2):
            for iINorOUT in range(2):
                for iRev in range(pitchSpiral):  # Number of revolutions.
                    for iPoint in range(nPointsPerRevolution):
                        points.append(Point(x=r*N.cos(q), 
                                            y=r*N.sin(q)))
                        q = q + dq # Step around the circle.
                        r = r + dr
            
                #if r>=rmax or r<=rmin: 
                dr = -dr
        
        #if r<rmin: 
            q = q + N.pi * (1.0 - 0.1*N.random.random()) # So we don't always retrace the same path.
            dq = -dq
            

        return points
        

    # GetPointsRamp() creates a set of points where pt.x goes from 0 to radius, and pt.y goes from radius to 0.
    def GetPointsRamp(self):
        nPoints = int(self.hzPoint/self.pattern.hz)
        xStart = 0.0
        xEnd = self.pattern.radius

        delta = (xEnd-xStart) / (nPoints-1)
        
        x = xStart
        points = []
        for i in range(nPoints):
            points.append(Point(x=x, 
                                y=xEnd-x))
            x = x+delta


        return points
    
        
    def UpdatePatternPoints (self):        
        if self.pattern.mode == 'byshape':  # Create the point list.
            if self.pattern.shape == 'constant':
                self.pattern.points = self.GetPointsConstant()
            elif self.pattern.shape == 'circle':
                self.pattern.points = self.GetPointsCircle()
            elif self.pattern.shape == 'square':
                self.pattern.points = self.GetPointsSquare()
            elif self.pattern.shape == 'flylogo':
                self.pattern.points = self.GetPointsFlylogo()
            elif self.pattern.shape == 'spiral':
                self.pattern.points = self.GetPointsSpiral()
            elif self.pattern.shape == 'ramp':
                self.pattern.points = self.GetPointsRamp()
            else:
                rospy.logerror('PatternGen: unknown shape')
                
                
        
    # PatternGen_callback() 
    #   Receive the message that sets up a pattern generation.
    #
    def PatternGen_callback (self, msgPatternGen):
        self.pattern.mode   = msgPatternGen.mode
        self.pattern.shape  = msgPatternGen.shape
        self.pattern.frame  = msgPatternGen.frame
        self.pattern.hz     = msgPatternGen.hz
        self.pattern.count  = msgPatternGen.count
        self.pattern.radius = msgPatternGen.radius

        if self.pattern.count==-1:
            self.pattern.count = 2147483640 # MAX_INT
        
        if self.pattern.mode=='bypoints':
            self.pattern.points = msgPatternGen.points
            
        self.UpdatePatternPoints()
        if msgPatternGen.preempt or self.iPoint >= len(self.pattern.points):
            self.iPoint = 0
            
            
    def SendSignalPoint(self): 
        if self.pattern.points is not None and len(self.pattern.points)>0:
            #rospy.logwarn('rate=%s' % self.hzPoint)
            if self.pattern.count>0:
                self.pts.header.frame_id = self.pattern.frame
                #self.pts.header.stamp = rospy.Time.now() + self.dtPoint
                self.pts.point = self.pattern.points[self.iPoint]
                try:
                    self.SignalOutput (self.pts)
                    #rospy.logwarn('points[%d]=[%0.2f,%0.2f]' % (self.iPoint,self.pattern.points[self.iPoint].x,self.pattern.points[self.iPoint].y))
                except rospy.ServiceException:
                    rospy.logwarn('SignalOutput() exception.  Reconnecting...')
                    rospy.wait_for_service(self.stSignalInput)
                    self.SignalOutput = rospy.ServiceProxy(self.stSignalInput, SrvSignal)
                    
                #rospy.logwarn ('PG pt=%s' % [pts.point.x, pts.point.y])
                self.iPoint += 1
                
                # When the pattern output is completed, go to the next pattern.
                if self.iPoint >= len(self.pattern.points):
                    #rospy.logwarn('EndOfPattern: self.iPoint=%d, count=%d' % (self.iPoint, self.pattern.count))
                    self.iPoint = 0
                    self.pattern.count -= 1
                    self.UpdatePatternPoints()

        
    def Main(self):
        #rospy.loginfo('pattern object=%s' % self.pattern)
        #try:
            while not rospy.is_shutdown():
                self.SendSignalPoint()
                self.ratePoint.sleep()
                
        #except:
        #    rospy.logwarn('PatternGen shutting down')


if __name__ == '__main__':
    try:
        patterngen = PatternGenXY()
        patterngen.Main()
    except rospy.ROSInterruptException: 
        pass

