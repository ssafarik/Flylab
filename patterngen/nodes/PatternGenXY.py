#!/usr/bin/env python
import roslib; roslib.load_manifest('patterngen')
import rospy
import copy
import numpy as N
import threading
from geometry_msgs.msg import PoseStamped, Point, PointStamped

from patterngen.msg import MsgPattern
from patterngen.srv import *



class NullClass:
    pass
    
class PatternGenXY:

    def __init__(self):
        rospy.init_node('PatternGenXY')
        rospy.loginfo ("PatternGenXY name=%s", __name__)

        # The signal to continuously output.
        self.signal = NullClass()
        self.signal.mode = 'byshape'
        self.signal.shape = 'none'
        self.signal.frame_id = 'Stage'
        self.signal.hzPattern = 1.0
        self.signal.hzPoint = 50
        self.signal.count = 0
        self.signal.points = []
        self.signal.size = Point(25.4, 25.4, 0.0)
        self.signal.preempt = True
        self.signal.param = 0.0

        # The pattern to use for point creation.
        self.pattern = NullClass()
        self.pattern.mode = 'byshape'
        self.pattern.shape = 'none'
        self.pattern.frame_id = 'Stage'
        self.pattern.hzPattern = 1.0
        self.pattern.hzPoint = 50
        self.pattern.count = 0
        self.pattern.points = []
        self.pattern.size = Point(25.4, 25.4, 0.0)
        self.pattern.preempt = True
        self.pattern.param = 0.0

        self.iPoint = 0

        self.pts = PointStamped()
        self.lock = threading.Lock()
        
        self.dtPoint = rospy.Duration(1/self.pattern.hzPoint)
        self.ratePoint = rospy.Rate(self.pattern.hzPoint)
        
        self.subSetSignalGen = rospy.Subscriber('SetSignalGen', MsgPattern, self.SetSignalGen_callback)
        self.srvGetPatternPoints = rospy.Service('GetPatternPoints', SrvGetPatternPoints, self.GetPatternPoints_callback)
        
        
        # Load stage services.
        self.stSignalInput = 'signal_input'
        rospy.loginfo('Waiting for: '+self.stSignalInput)
        rospy.wait_for_service(self.stSignalInput)
        try:
            self.SignalOutput = rospy.ServiceProxy(self.stSignalInput, SrvSignal)
        except rospy.ServiceException, e:
            rospy.loginfo ("FAILED %s: %s"%(self.stSignalInput,e))
        rospy.loginfo('Passed: '+self.stSignalInput)
        
        


    def GetPointsConstant(self, pattern):
        nPoints = int(pattern.hzPoint/pattern.hzPattern)
        points = [Point(x=pattern.size.x, 
                        y=pattern.size.y)] * nPoints  # [(x,y),(x,y),(x,y), ...]
        
        return points

    
    def GetPointsCircle(self, pattern):
        nPoints = int(pattern.hzPoint/pattern.hzPattern)
        q = 0.0 #N.pi/2.0  # Starting position
        dq = 2.0*N.pi/nPoints
        r = N.linalg.norm([pattern.size.x, pattern.size.y])

        points = []
        for i in range(nPoints):
            points.append(Point(x=r*N.cos(q), 
                                y=r*N.sin(q)))
            q = q + dq    # Step around the circle

	return points
    
        
    def GetPointsSquare(self, pattern):
        nPoints = int(pattern.hzPoint/pattern.hzPattern)
        nPointsSide = int(nPoints / 4.0) # Points per side
        xmin = -pattern.size.x / 2
        xmax =  pattern.size.x / 2
        ymin = -pattern.size.y / 2
        ymax =  pattern.size.y / 2
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
    

    def GetPointsFlylogo (self, pattern):
        flylogo = [[-4, 14],[-3, 11],[-7, 7],[-6, 5],[-7, 4],
                   [-9, 4],[-12, 6],[-10, 8],[-12, 10],[-10, 8],[-12, 6],[-9, 4],
                   [-13, 3],[-14, 0],[-11, 1.5],[-9, 4],
                   [-11, 0],
                   [-9, -4],[-11, -1.5],[-14, 0],[-13, -3],
                   [-9, -4],[-12, -6],[-10, -8],[-12, -10],[-10, -8],[-12, -6],[-9, -4],
                   [-7, -4],[-6, -5],[-7, -7],[-3, -11],[-4, -14],[-3, -11],[-7, -7],[-6, -5],
                   [-3, -7],[0,-10],[3, -13],[8,-14.5],[13, -15],[16, -12],[12, -6],[4, -2],
                   [0, -1],[-5, -4],[-2, 0],[-5, 4],[0, 1],[4, 2],[4, -2],[7,0],[4, 2],[12, 6],[16, 12],[13, 15],[8,14.5],[3, 13],[0,10],[-3, 7],[-6, 5],[-7, 7],[-3, 11],[-4, 14]]
        points = []

        xmax = 16.0
        ymax = 15.0
        for k in range(len(flylogo)):
            pt = Point(x=float(flylogo[k][0]) * pattern.size.x / xmax,
                       y=float(flylogo[k][1]) * pattern.size.y / ymax)
            points.append(pt)
           
        return points
    
        
    def GetPointsSpiral (self, pattern):
        nPoints = int(pattern.hzPoint/pattern.hzPattern)
        rospy.logwarn('nPoints=%d' % nPoints)
        pitchSpiral = 2
        nRevolutionsPerPattern = 2 * 2 * pitchSpiral  # nCworCCW * nInOrOut * pitch
        nPointsPerRevolution = nPoints / nRevolutionsPerPattern
        q = 2.0 * N.pi * N.random.random()
        dq = 2.0 * N.pi/nPointsPerRevolution
        radius = N.linalg.norm([pattern.size.x, pattern.size.y])
        
        rmax = radius
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
        

    # GetPointsRamp() creates a set of points where pt.x goes from 0 to size.x, and pt.y goes from size.y to 0.
    def GetPointsRamp(self, pattern):
        nPoints = int(pattern.hzPoint/pattern.hzPattern)
        xStart = 0.0
        xEnd = pattern.size.x
        yStart = pattern.size.y
        yEnd = 0.0

        xDelta = (xEnd-xStart) / (nPoints-1)
        yDelta = (yEnd-yStart) / (nPoints-1)
        
        x = xStart
        y = yStart
        points = []
        for i in range(nPoints):
            points.append(Point(x=x, 
                                y=y))
            x = x+xDelta
            y = y+yDelta

        # Alternatively do something like:  points = N.linspace([xStart,yStart],[xEnd,yEnd],nPoints)
        
        return points
    
        
    def GetPointsGridRaster(self, pattern):
        gridpitch = int(pattern.param)
        nPoints = int(pattern.hzPoint/pattern.hzPattern)
        nPointsSide = int(nPoints / gridpitch)
        xmin = -pattern.size.x / 2
        xmax =  pattern.size.x / 2
        ymin = -pattern.size.y / 2
        ymax =  pattern.size.y / 2

        points = []        
        for x in N.linspace(xmin, xmax, nPointsSide+1):
            for y in N.linspace(ymin, ymax, nPointsSide+1):
                points.append(Point(x,y,0))

    
        return points


    # PeanoCurve() generates a fractal Peano curve.
    class PeanoCurve:
        points = []
        
        def Left (self):
            self.x += -self.dx
            self.points.append(Point(self.x, self.y, 0))
            
        def Right (self):
            self.x += self.dx
            self.points.append(Point(self.x, self.y, 0))
            
        def Up (self):
            self.y += self.dy
            self.points.append(Point(self.x, self.y, 0))
            
        def Down (self):
            self.y += -self.dy
            self.points.append(Point(self.x, self.y, 0))
            
        
        def A(self, level):
            if level > 0:
                self.I(level-1)
                self.Up()
                self.J(level-1)
                self.Left()
                self.A(level-1)
                self.Down()
                self.B(level-1)
                #self.Down()
    
        def B(self, level):
            if level > 0:
                self.D(level-1)
                self.Right()
                self.C(level-1)
                self.Down()
                self.L(level-1)
                self.Left()
                self.A(level-1)
                #self.Down()
    
        def C(self, level):
            if level > 0:
                self.E(level-1)
                self.Right()
                self.C(level-1)
                self.Down()
                self.L(level-1)
                self.Left()
                self.A(level-1)
                #self.Down()
    
        def D(self, level):
            if level > 0:
                self.B(level-1)
                self.Down()
                self.D(level-1)
                self.Right()
                self.G(level-1)
                self.Up()
                self.F(level-1)
                #self.Right()
    
        def E(self, level):
            if level > 0:
                self.C(level-1)
                self.Down()
                self.D(level-1)
                self.Right()
                self.G(level-1)
                self.Up()
                self.F(level-1)
                #self.Right()
    
        def F(self, level):
            if level > 0:
                self.J(level-1)
                self.Left()
                self.I(level-1)
                self.Up()
                self.F(level-1)
                self.Right()
                self.E(level-1)
                #self.Right()
    
        def G(self, level):
            if level > 0:
                self.C(level-1)
                self.Down()
                self.D(level-1)
                self.Right()
                self.G(level-1)
                self.Up()
                self.H(level-1)
                #self.Up()
    
        def H(self, level):
            if level > 0:
                self.J(level-1)
                self.Left()
                self.I(level-1)
                self.Up()
                self.F(level-1)
                self.Right()
                self.G(level-1)
                #self.Up()
    
        def I(self, level):
            if level > 0:
                self.K(level-1)
                self.Left()
                self.I(level-1)
                self.Up()
                self.F(level-1)
                self.Right()
                self.G(level-1)
                #self.Up()
    
        def J(self, level):
            if level > 0:
                self.H(level-1)
                self.Up()
                self.J(level-1)
                self.Left()
                self.A(level-1)
                self.Down()
                self.L(level-1)
                #self.Left()
    
        def K(self, level):
            if level > 0:
                self.I(level-1)
                self.Up()
                self.J(level-1)
                self.Left()
                self.A(level-1)
                self.Down()
                self.L(level-1)
                #self.Left()
    
        def L(self, level):
            if level > 0:
                self.D(level-1)
                self.Right()
                self.C(level-1)
                self.Down()
                self.L(level-1)
                self.Left()
                self.K(level-1)
                #self.Left()
    
        def GetPoints(self, level, xSize, ySize):
            self.dx = xSize / (2**(level+1)-1)
            self.dy = ySize / (2**(level+1)-1)
            self.x = -self.dx/2
            self.y = self.dy/2
            
            self.points = []
            self.A(level)
            self.Down()
            self.D(level)
            self.Right()
            self.G(level)
            self.Up()
            self.J(level)
            self.Left()
            
            return self.points


    def GetPointsGridPeano(self, pattern):
        peano = self.PeanoCurve()
        level = int(pattern.param)
            
        return peano.GetPoints(level, pattern.size.x, pattern.size.y)
    

    # HilbertCurve() generates a fractal Hilbert curve.
    # This code needs to be tweaked to take xSize,ySize, etc, as w/ Peano. 
    class HilbertCurve:
        points = []
        
        def HilbertMove (self, dx, dy):
            self.x += dx
            self.y += dy
            self.points.append(Point(self.x, self.y, 0))
            
        
        def HilbertA(self, level, d):
            if level > 0:
                self.HilbertB(level-1,d)
                self.HilbertMove(0,d)
                self.HilbertA(level-1,d)
                self.HilbertMove(d,0)
                self.HilbertA(level-1,d)
                self.HilbertMove(0,-d)
                self.HilbertC(level-1,d)
    
        def HilbertB(self, level, d):
            if level > 0:
                self.HilbertA(level-1,d)
                self.HilbertMove(d,0)
                self.HilbertB(level-1,d)
                self.HilbertMove(0,d)
                self.HilbertB(level-1,d)
                self.HilbertMove(-d,0)
                self.HilbertD(level-1,d)
    
        def HilbertC(self, level, d):
            if level > 0:
                self.HilbertD(level-1,d)
                self.HilbertMove(-d,0)
                self.HilbertC(level-1,d)
                self.HilbertMove(0,-d)
                self.HilbertC(level-1,d)
                self.HilbertMove(d,0)
                self.HilbertA(level-1,d)
    
        def HilbertD(self, level, d):
            if level > 0:
                self.HilbertC(level-1,d)
                self.HilbertMove(0,-d)
                self.HilbertD(level-1,d)
                self.HilbertMove(-d,0)
                self.HilbertD(level-1,d)
                self.HilbertMove(0,d)
                self.HilbertB(level-1,d)
    
        def GetPoints(self, level, d):
            self.x = -(d * 2^level)
            self.y = -(d * 2^level)
            self.points = []
            self.HilbertA(level, d)
            
            return self.points


    def GetPointsGridHilbert(self, pattern):
        hilbert = self.HilbertCurve()
        level = int(pattern.param)
        width = 10
            
        return hilbert.GetPoints(level, width)
    

    def GetPointsGrid(self, pattern):
        return self.GetPointsGridPeano(pattern)


    def GetPointsCharacter(self, pattern):
        points_bychar = {
            '0': [[ 3.0, -4.0], [ 2.0, -5.0], [-2.0, -5.0], [-3.0, -4.0], [-3.0,  4.0], [-2.0,  5.0], [ 2.0,  5.0], [ 3.0,  4.0], [ 3.0, -4.0]],
            '1': [[ 0.0, -5.0], [ 0.0,  5.0]],
            '2': [[-3.0, -4.0], [-2.0, -5.0], [ 2.0, -5.0], [ 3.0, -4.0], [ 3.0, -1.0], [ 2.0,  0.0], [-2.0,  1.0], [-3.0,  2.0], [-3.0,  5.0], [ 3.0,  5.0]],
            '3': [[-3.0, -4.0], [-2.0, -5.0], [ 2.0, -5.0], [ 3.0, -4.0], [ 3.0, -1.0], [ 2.0,  0.0], [-2.0,  0.0], [ 2.0,  0.0], [ 3.0,  1.0], [ 3.0,  4.0], [ 2.0,  5.0], [-2.0,  5.0], [-3.0,  4.0]],
            '4': [[-2.0, -5.0], [-3.0,  0.0], [ 3.0,  0.0], [ 3.0, -5.0], [ 3.0,  5.0]],
            '5': [[ 3.0, -5.0], [-2.5, -5.0], [-2.8, -2.0], [ 0.0, -2.0], [ 2.0, -1.5], [ 3.0,  0.0], [ 3.0,  3.0], [ 2.0,  4.5], [ 0.0,  5.0], [-1.0,  5.0], [-3.0,  4.0]],
            '6': [[ 3.0, -5.0], [ 1.0, -5.0], [-1.0, -4.0], [-2.0, -3.0], [-3.0,  0.0], [-3.0,  3.0], [-2.0,  5.0], [ 2.0,  5.0], [ 3.0,  4.0], [ 3.0,  1.0], [ 2.0,  0.0], [-3.0,  0.0]],
            '7': [[-3.0, -5.0], [ 3.0, -5.0], [ 3.0, -3.0], [ 1.0, -1.0], [ 1.0,  5.0]],
            '8': [[ 3.5, -4.0], [-3.0,  2.0], [-3.0,  4.0], [-2.0,  5.0], [ 2.0,  5.0], [ 3.0,  4.0], [ 3.0,  2.0], [-3.0, -3.0], [-3.0, -4.0], [-2.0, -5.0], [ 1.0, -5.0], [ 3.0, -3.5]],
            '9': [[-3.0,  5.0], [-1.0,  5.0], [ 1.0,  4.0], [ 2.0,  3.0], [ 3.0,  0.0], [ 3.0, -3.0], [ 2.0, -5.0], [-2.0, -5.0], [-3.0, -4.0], [-3.0, -1.0], [-2.0,  0.0], [ 3.0,  0.0]],
            'A': [[-3.0,  5.0], [ 0.0, -5.0], [ 3.0,  5.0], [ 1.9,  1.0], [-2.0,  1.0]],
            'B': [[-3.0,  5.0], [-3.0, -5.0], [ 2.0, -5.0], [ 3.0, -4.0], [ 3.0, -1.0], [ 2.0,  0.0], [-3.0,  0.0], [ 2.0,  0.0], [ 3.0,  1.0], [ 3.0,  4.0], [ 2.0,  5.0], [-3.0,  5.0]],
            'C': [[ 3.0, -3.0], [ 3.0, -4.0], [ 2.0, -5.0], [-2.0, -5.0], [-3.0, -4.0], [-3.0,  4.0], [-2.0,  5.0], [ 2.0,  5.0], [ 3.0,  4.0], [ 3.0,  2.0]],
            'D': [[-3.0,  5.0], [-3.0, -5.0], [ 2.0, -5.0], [ 3.0, -4.0], [ 3.0,  4.0], [ 2.0,  5.0], [-3.0,  5.0]],
            'E': [[ 3.0, -5.0], [-3.0, -5.0], [-3.0,  0.0], [ 2.0,  0.0], [-3.0,  0.0], [-3.0,  5.0], [ 3.0,  5.0]],
            'F': [[ 3.0, -5.0], [-3.0, -5.0], [-3.0,  0.0], [ 2.0,  0.0], [-3.0,  0.0], [-3.0,  5.0]],
            'G': [[ 3.0, -3.0], [ 3.0, -4.0], [ 2.0, -5.0], [-2.0, -5.0], [-3.0, -4.0], [-3.0,  4.0], [-2.0,  5.0], [ 2.0,  5.0], [ 3.0,  4.0], [ 3.0,  1.0], [ 1.0,  1.0]],
            'H': [[-3.0, -5.0], [-3.0,  5.0], [-3.0,  0.0], [ 3.0,  0.0], [ 3.0, -5.0], [ 3.0,  5.0]],
            'I': [[-1.0, -5.0], [ 1.0, -5.0], [ 0.0, -5.0], [ 0.0,  5.0], [-1.0,  5.0], [ 1.0,  5.0]],
            'J': [[-3.0,  2.0], [-3.0,  4.0], [-2.0,  5.0], [ 2.0,  5.0], [ 3.0,  4.0], [ 3.0, -5.0]],
            'K': [[-3.0, -5.0], [-3.0,  5.0], [-3.0,  1.0], [ 3.0, -5.0], [-1.7,  0.2], [ 3.0,  5.0]],
            'L': [[-3.0, -5.0], [-3.0,  5.0], [ 3.0,  5.0]],
            'M': [[-3.0,  5.0], [-3.0, -5.0], [ 0.0, -2.0], [ 3.0, -5.0], [ 3.0,  5.0]],
            'N': [[-3.0,  5.0], [-3.0, -5.0], [ 3.0,  5.0], [ 3.0, -5.0]],
            'O': [[ 2.0, -5.0], [-2.0, -5.0], [-3.0, -4.0], [-3.0,  4.0], [-2.0,  5.0], [ 2.0,  5.0], [ 3.0,  4.0], [ 3.0, -4.0], [ 2.0, -5.0]],
            'P': [[-3.0,  5.0], [-3.0, -5.0], [ 2.0, -5.0], [ 3.0, -4.0], [ 3.0, -1.0], [ 2.0,  0.0], [-3.0,  0.0]],
            'Q': [[ 3.0,  4.0], [ 2.0,  5.0], [-2.0,  5.0], [-3.0,  4.0], [-3.0, -4.0], [-2.0, -5.0], [ 2.0, -5.0], [ 3.0, -4.0], [ 3.0,  4.0], [ 2.5,  4.5], [ 0.0,  2.0], [ 3.0,  5.0]],
            'R': [[-3.0,  5.0], [-3.0, -5.0], [ 2.0, -5.0], [ 3.0, -4.0], [ 3.0, -1.0], [ 2.0,  0.0], [-3.0,  0.0], [ 1.0,  0.0], [ 3.0,  5.0]],
            'S': [[ 3.0, -3.0], [ 3.0, -4.0], [ 2.0, -5.0], [-2.0, -5.0], [-3.0, -4.0], [-3.0, -2.0], [-2.0, -1.0], [ 2.0,  0.0], [ 3.0,  1.0], [ 3.0,  4.0], [ 2.0,  5.0], [-2.0,  5.0], [-3.0,  4.0], [-3.0,  3.0]],
            'T': [[ 3.0, -5.0], [-3.0, -5.0], [ 0.0, -5.0], [ 0.0,  5.0]],
            'U': [[-3.0, -5.0], [-3.0,  4.0], [-2.0,  5.0], [ 2.0,  5.0], [ 3.0,  4.0], [ 3.0, -5.0]],
            'V': [[-3.0, -5.0], [ 0.0,  5.0], [ 3.0, -5.0]],
            'W': [[-3.0, -5.0], [-2.0,  5.0], [ 0.0,  1.0], [ 2.0,  5.0], [ 3.0, -5.0]],
            'X': [[-3.0, -5.0], [ 0.0,  0.0], [-3.0,  5.0], [ 3.0, -5.0], [ 0.0,  0.0], [ 3.0,  5.0]],
            'Y': [[-3.0, -5.0], [ 0.0,  0.0], [ 3.0, -5.0], [ 0.0,  0.0], [ 0.0,  5.0]],
            'Z': [[-3.0, -5.0], [ 3.0, -5.0], [-3.0,  5.0], [ 3.0,  5.0]]
         }
        
        point_list = []
        if pattern.shape in points_bychar:
            xy_list = points_bychar[pattern.shape]
            for xy in xy_list:
                point_list.append(Point(x=xy[0] * pattern.size.x / 10.0, 
                                        y=xy[1] * pattern.size.y / 10.0))
            
        return point_list

    

    def UpdatePatternPoints (self, pattern):        
        if pattern.mode == 'byshape':  # Create the point list.
            if pattern.shape == 'constant':
                pattern.points = self.GetPointsConstant(pattern)
            elif pattern.shape == 'circle':
                pattern.points = self.GetPointsCircle(pattern)
            elif pattern.shape == 'square':
                pattern.points = self.GetPointsSquare(pattern)
            elif pattern.shape == 'flylogo':
                pattern.points = self.GetPointsFlylogo(pattern)
            elif pattern.shape == 'spiral':
                pattern.points = self.GetPointsSpiral(pattern)
            elif pattern.shape == 'ramp':
                pattern.points = self.GetPointsRamp(pattern)
            elif pattern.shape == 'grid':
                pattern.points = self.GetPointsGrid(pattern)
            elif pattern.shape == 'raster':
                pattern.points = self.GetPointsGridRaster(pattern)
            elif pattern.shape == 'hilbert':
                pattern.points = self.GetPointsGridHilbert(pattern)
            elif pattern.shape == 'peano':
                pattern.points = self.GetPointsGridPeano(pattern)
            elif (len(pattern.shape)==1) and (pattern.shape.isalnum()):
                pattern.points = self.GetPointsCharacter(pattern)
            elif pattern.shape == 'none':
                pattern.points = []
            else:
                pattern.points = []
                rospy.logerror('PatternGen: unknown shape')
        #else pattern.points = pattern.points
                
                
        
    # PatternGen_callback() 
    #   Receive the message that sets up a pattern generation.
    #
    def SetSignalGen_callback (self, msgPatternGen):
        with self.lock:
            self.signal.mode        = msgPatternGen.mode
            self.signal.shape       = msgPatternGen.shape
            self.signal.frame_id    = msgPatternGen.frame_id
            self.signal.hzPattern   = msgPatternGen.hzPattern
            self.signal.hzPoint     = msgPatternGen.hzPoint
            self.signal.count       = msgPatternGen.count
            self.signal.size        = msgPatternGen.size

	    self.dtPoint = rospy.Duration(1/self.signal.hzPoint)
	    self.ratePoint = rospy.Rate(self.signal.hzPoint)
    
            if self.signal.count==-1:
                self.signal.count = 2147483640 # MAX_INT
            
            if self.signal.mode=='bypoints':
                self.signal.points = msgPatternGen.points
                
            self.UpdatePatternPoints(self.signal)
            if msgPatternGen.preempt or self.iPoint >= len(self.signal.points):
                self.iPoint = 0

	    #rospy.logwarn ('SetSignalGen_callback() points=%s' % self.signal.points)
            
            
    def GetPatternPoints_callback(self, reqGetPatternPoints):
        with self.lock:
            
#            pattern.mode       = reqGetPatternPoints.pattern.mode
#            pattern.shape      = reqGetPatternPoints.pattern.shape
#            pattern.frame_id   = reqGetPatternPoints.pattern.frame_id
#            pattern.hzPattern  = reqGetPatternPoints.pattern.hzPattern
#            pattern.hzPoint    = reqGetPatternPoints.pattern.hzPoint
#            pattern.count      = reqGetPatternPoints.pattern.count
#            pattern.size       = reqGetPatternPoints.pattern.size
#    
#            if pattern.count==-1:
#                pattern.count = 2147483640 # MAX_INT
#            
#            if self.signal.mode=='bypoints':
#                self.pattern.points = reqGetPatternPoints.pattern.points
#                
            #rospy.logwarn ('reqGetPatternPoints=%s' % reqGetPatternPoints)
            #respGetPatternPoints = copy.copy(reqGetPatternPoints)
            #rospy.logwarn ('respGetPatternPoints=%s' % respGetPatternPoints)
            
            self.UpdatePatternPoints(reqGetPatternPoints.pattern)
            
        return SrvGetPatternPointsResponse(pattern=reqGetPatternPoints.pattern)
    
            
    def SendSignalPoint(self): 
        if self.signal.points is not None and len(self.signal.points)>0:
            #rospy.logwarn('rate=%s' % self.pattern.hzPoint)
            if self.signal.count>0:
                self.pts.header.frame_id = self.signal.frame_id
                #self.pts.header.stamp = rospy.Time.now() + self.dtPoint
                self.pts.point = self.signal.points[self.iPoint]
                try:
                    self.SignalOutput (self.pts)
                    #rospy.logwarn('points[%d]=[%0.2f,%0.2f]' % (self.iPoint,self.signal.points[self.iPoint].x,self.signal.points[self.iPoint].y))
                except rospy.ServiceException:
                    rospy.logwarn('SignalOutput() exception.  Reconnecting...')
                    rospy.wait_for_service(self.stSignalInput)
                    self.SignalOutput = rospy.ServiceProxy(self.stSignalInput, SrvSignal)
                    
                #rospy.logwarn ('PG pt=%s' % [pts.point.x, pts.point.y])
                self.iPoint += 1
                
                # When the pattern output is completed, go to the next pattern.
                if self.iPoint >= len(self.signal.points):
                    #rospy.logwarn('EndOfPattern: self.iPoint=%d, count=%d' % (self.iPoint, self.signal.count))
                    self.iPoint = 0
                    self.signal.count -= 1
                    self.UpdatePatternPoints(self.signal)

        
    def Main(self):
        while not rospy.is_shutdown():
            self.SendSignalPoint()
            self.ratePoint.sleep()


if __name__ == '__main__':
    try:
        patterngen = PatternGenXY()
        patterngen.Main()
    except rospy.exceptions.ROSInterruptException: 
        pass

