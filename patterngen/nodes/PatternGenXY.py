#!/usr/bin/env python
import roslib; roslib.load_manifest('patterngen')
import rospy
import copy
import numpy as np
import threading
from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped, Twist
from std_msgs.msg import String
import tf
from flycore.msg import MsgFrameState

from patterngen.msg import MsgPattern
from patterngen.srv import *

class Struct:
    pass


class PatternGenXY:

    def __init__(self):
        rospy.init_node('PatternGenXY')
        rospy.loginfo ("PatternGenXY name=%s", __name__)

        # The pattern for continuous output.
        self.output = Struct()
        self.output.pattern = MsgPattern()
        self.output.pattern.shape = 'none'
        self.output.pattern.hzPattern = 1.0
        self.output.pattern.hzPoint = 50
        self.output.pattern.count = 0
        self.output.pattern.points = []
        self.output.pattern.frameidPosition = 'Arena'
        self.output.pattern.frameidAngle = 'Arena'
        self.output.pattern.size = Point(25.4, 25.4, 0.0)
        self.output.pattern.restart = True
        self.output.pattern.param = 0.0
        self.output.pattern.isDirty = True
        self.output.pathlen = 0.0
        self.output.ratePoint = rospy.Rate(self.output.pattern.hzPoint)
        self.output.iPoint = None

        self.output.lock = threading.Lock()
        
        

        self.command = 'continue'
        self.command_list = ['continue','pause_now','pause_after_trial', 'exit_after_trial', 'exit_now']
        
        self.subCommand          = rospy.Subscriber('experiment/command', String, self.Command_callback)
        self.subSetPattern       = rospy.Subscriber('SetPattern', MsgPattern, self.SetPattern_callback)
        self.tfrx = tf.TransformListener()
        
        self.services = {}
        self.services['GetPatternPoints'] = rospy.Service('GetPatternPoints', SrvGetPatternPoints, self.GetPatternPoints_callback)
        
        
        # Load stage services.
        self.stSignalInput = 'signal_input'
        rospy.loginfo('Waiting for: '+self.stSignalInput)
        rospy.wait_for_service(self.stSignalInput)
        try:
            self.SignalOutput = rospy.ServiceProxy(self.stSignalInput, SrvSignal)
        except rospy.ServiceException, e:
            rospy.loginfo ("FAILED %s: %s"%(self.stSignalInput,e))
        
        


    def Command_callback(self, msgString):
        self.command = msgString.data


    def GetPointsConstant(self, pattern):
        nPoints = int(pattern.hzPoint/pattern.hzPattern)
        points = [Point(x=pattern.size.x, 
                        y=pattern.size.y)] * nPoints  # [(x,y),(x,y),(x,y), ...]
        
        return points

    
    def GetPointsCircle(self, pattern):
        nPoints = int(pattern.hzPoint/pattern.hzPattern)
        q = 0.0 #np.pi/2.0  # Starting position
        dq = 2.0*np.pi/nPoints
        r = np.linalg.norm([pattern.size.x, pattern.size.y])

        points = []
        for i in range(nPoints):
            points.append(Point(x=r*np.cos(q), 
                                y=r*np.sin(q)))
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

                if (iSide==0):
                    dx =  step
                    dy =  0.0
                elif (iSide==1):
                    dx =  0.0
                    dy =  step
                elif (iSide==2):
                    
                    dx = -step
                    dy =  0.0
                elif (iSide==3):
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
            pt = Point(x=-float(flylogo[k][0]) * pattern.size.x / xmax,
                       y=float(flylogo[k][1]) * pattern.size.y / ymax)
            points.append(pt)
           
        return self.InterpolatePoints(points, 0.2)
    
        
    def GetPointsSpiral (self, pattern):
        nPoints = int(pattern.hzPoint/pattern.hzPattern)
        pitchSpiral = 2
        nRevolutionsPerPattern = 2 * 2 * pitchSpiral  # nCworCCW * nInOrOut * pitch
        nPointsPerRevolution = nPoints / nRevolutionsPerPattern
        q = 2.0 * np.pi * np.random.random()
        dq = 2.0 * np.pi/nPointsPerRevolution
        radius = np.linalg.norm([pattern.size.x, pattern.size.y])
        
        rmax = radius
        rmin = 0.10
        r = rmin
        dr = (rmax-rmin) / (pitchSpiral * nPointsPerRevolution)

        points = []
        for iCWorCCW in range(2):
            for iINorOUT in range(2):
                for iRev in range(pitchSpiral):  # Number of revolutions.
                    for iPoint in range(nPointsPerRevolution):
                        points.append(Point(x=r*np.cos(q), 
                                            y=r*np.sin(q)))
                        q = q + dq # Step around the circle.
                        r = r + dr
            
                #if r>=rmax or r<=rmin: 
                dr = -dr
        
        #if r<rmin: 
            q = q + np.pi * (1.0 - 0.1*np.random.random()) # So we don't always retrace the same path.
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

        # Alternatively do something like:  points = np.linspace([xStart,yStart],[xEnd,yEnd],nPoints)
        
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
        for x in np.linspace(xmin, xmax, nPointsSide+1):
            for y in np.linspace(ymin, ymax, nPointsSide+1):
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
            if (0 < level):
                self.I(level-1)
                self.Up()
                self.J(level-1)
                self.Left()
                self.A(level-1)
                self.Down()
                self.B(level-1)
                #self.Down()
    
        def B(self, level):
            if (0 < level):
                self.D(level-1)
                self.Right()
                self.C(level-1)
                self.Down()
                self.L(level-1)
                self.Left()
                self.A(level-1)
                #self.Down()
    
        def C(self, level):
            if (0 < level):
                self.E(level-1)
                self.Right()
                self.C(level-1)
                self.Down()
                self.L(level-1)
                self.Left()
                self.A(level-1)
                #self.Down()
    
        def D(self, level):
            if (0 < level):
                self.B(level-1)
                self.Down()
                self.D(level-1)
                self.Right()
                self.G(level-1)
                self.Up()
                self.F(level-1)
                #self.Right()
    
        def E(self, level):
            if (0 < level):
                self.C(level-1)
                self.Down()
                self.D(level-1)
                self.Right()
                self.G(level-1)
                self.Up()
                self.F(level-1)
                #self.Right()
    
        def F(self, level):
            if (0 < level):
                self.J(level-1)
                self.Left()
                self.I(level-1)
                self.Up()
                self.F(level-1)
                self.Right()
                self.E(level-1)
                #self.Right()
    
        def G(self, level):
            if (0 < level):
                self.C(level-1)
                self.Down()
                self.D(level-1)
                self.Right()
                self.G(level-1)
                self.Up()
                self.H(level-1)
                #self.Up()
    
        def H(self, level):
            if (0 < level):
                self.J(level-1)
                self.Left()
                self.I(level-1)
                self.Up()
                self.F(level-1)
                self.Right()
                self.G(level-1)
                #self.Up()
    
        def I(self, level):
            if (0 < level):
                self.K(level-1)
                self.Left()
                self.I(level-1)
                self.Up()
                self.F(level-1)
                self.Right()
                self.G(level-1)
                #self.Up()
    
        def J(self, level):
            if (0 < level):
                self.H(level-1)
                self.Up()
                self.J(level-1)
                self.Left()
                self.A(level-1)
                self.Down()
                self.L(level-1)
                #self.Left()
    
        def K(self, level):
            if (0 < level):
                self.I(level-1)
                self.Up()
                self.J(level-1)
                self.Left()
                self.A(level-1)
                self.Down()
                self.L(level-1)
                #self.Left()
    
        def L(self, level):
            if (0 < level):
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


    def GetPointsPeano(self, pattern):
        peano = self.PeanoCurve()
        level = int(pattern.param)
            
        
        return peano.GetPoints(level, pattern.size.x, pattern.size.y)
    

    def GetPointsGrid(self, pattern):
        points = self.GetPointsPeano(pattern)
        return self.InterpolatePoints(points, 0.2)


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
        if (pattern.shape in points_bychar):
            xy_list = points_bychar[pattern.shape]
            for xy in xy_list:
                point_list.append(Point(x=xy[0] * pattern.size.x / 10.0, 
                                        y=-xy[1] * pattern.size.y / 10.0))
            
        return self.InterpolatePoints(point_list, 0.2)


    # Add intermediate points such that the point-point spacing is no greater than dr.
    def InterpolatePoints (self, points, dr):
        if (0.0 < dr):
            points_new = []
            for i in range(len(points)-1): # So we can interpolate from point i to point i+1.
                r = np.linalg.norm([points[i+1].x-points[i].x, points[i+1].y-points[i].y])
                n = np.ceil(r / dr)
                dx = (points[i+1].x-points[i].x) / n
                dy = (points[i+1].y-points[i].y) / n
                x = 0.0
                y = 0.0
                for ni in range(int(n)):
                    points_new.append(Point(x=points[i].x+x,y=points[i].y+y))
                    x = x+dx
                    y = y+dy
            points_new.append(Point(x=points[-1].x,y=points[-1].y))
            
        else:
            points_new = points
            
            
        return points_new
        

    def UpdatePatternPoints (self, pattern):      
        if (pattern.isDirty):  
            if (pattern.shape == 'constant'):
                pattern.points = self.GetPointsConstant(pattern)
            elif (pattern.shape == 'point'):
                pattern.points = self.GetPointsConstant(pattern)
            elif (pattern.shape == 'circle'):
                pattern.points = self.GetPointsCircle(pattern)
            elif (pattern.shape == 'square'):
                pattern.points = self.GetPointsSquare(pattern)
            elif (pattern.shape == 'flylogo'):
                pattern.points = self.GetPointsFlylogo(pattern)
            elif (pattern.shape == 'spiral'):
                pattern.points = self.GetPointsSpiral(pattern)
            elif (pattern.shape == 'ramp'):
                pattern.points = self.GetPointsRamp(pattern)
            elif (pattern.shape == 'peano'):
                pattern.points = self.GetPointsPeano(pattern)
            elif (pattern.shape == 'grid'):
                pattern.points = self.GetPointsGrid(pattern)
            elif (pattern.shape == 'raster'):
                pattern.points = self.GetPointsGridRaster(pattern)
            elif ((len(pattern.shape)==1) and (pattern.shape.isalnum())):
                pattern.points = self.GetPointsCharacter(pattern)
            elif (pattern.shape == 'bypoints'):
                pass
            elif (pattern.shape == 'none'):
                pattern.points = []
            else:
                pattern.points = []
                rospy.logerror('PatternGen: unknown pattern')
            
            pattern.isDirty = False
        
                
                
    def GetPathLength(self, points):
        pathlen = 0.0
        if (0 < len(points)):
            ptCur = points[0]
            for pt in points:
                pathlen += np.linalg.norm([pt.x-ptCur.x, 
                                           pt.y-ptCur.y])
                ptCur = pt
                
            pathlen += np.linalg.norm([points[-1].x-ptCur.x, 
                                       points[-1].y-ptCur.y])
            
        return pathlen
    
        
    # PatternGen_callback() 
    #   Receive the message that sets up a pattern generation.
    #
    def SetPattern_callback (self, msgPattern):
        iPointPre = self.output.iPoint

        # To provide a glitch-free transition, do the time-consuming setup of outputTmp first, then copy it into place at the end.
        outputTmp = Struct()
        outputTmp.pattern = MsgPattern()
        outputTmp.pattern.isDirty           = True
        outputTmp.pattern.frameidPosition   = msgPattern.frameidPosition
        outputTmp.pattern.frameidAngle      = msgPattern.frameidAngle
        outputTmp.pattern.shape             = msgPattern.shape
        outputTmp.pattern.hzPattern         = msgPattern.hzPattern
        outputTmp.pattern.hzPoint           = msgPattern.hzPoint
        outputTmp.pattern.count             = msgPattern.count
        outputTmp.pattern.size              = msgPattern.size
        outputTmp.pattern.restart           = msgPattern.restart
        outputTmp.pattern.param             = msgPattern.param
        outputTmp.pattern.direction         = msgPattern.direction # Forward (+1) or reverse (-1) through the pattern points.  0 means choose at random, +1 or -1.
        
        # If direction==0, then choose random +1 or -1.
        if (outputTmp.pattern.direction==0):
            outputTmp.pattern.direction = 2*np.random.randint(2)-1
             
        outputTmp.ratePoint = rospy.Rate(outputTmp.pattern.hzPoint)
        if (outputTmp.pattern.count==-1):
            outputTmp.pattern.count = 2147483640 # MAX_INT
        
        if (outputTmp.pattern.shape=='bypoints'):
            outputTmp.pattern.points = msgPattern.points
            
        self.UpdatePatternPoints(outputTmp.pattern)
        outputTmp.pathlen = self.GetPathLength(outputTmp.pattern.points)
        
        if (0 < len(outputTmp.pattern.points)):
            if (not outputTmp.pattern.restart) and (iPointPre is not None):
                # Find the point in the new pattern nearest the current point in the last pattern.
                # Convert Point()'s to lists.
                xyNew = []
                for pt in outputTmp.pattern.points:
                    xyNew.append([pt.x,pt.y])

                #rospy.logwarn('%s, %s' % (self.output.iPoint, len(self.output.pattern.points)))
                if (self.output.iPoint is not None):
                    pointPrev = self.output.pattern.points[self.output.iPoint]   # Save the prev cursor.
                     
                xyPrev = [pointPrev.x, pointPrev.y]
                    
                dxy = np.subtract(xyPrev, xyNew)
                dx = dxy[:,0]
                dy = dxy[:,1]
                d = np.hypot(dx, dy)
                iNearest = np.argmin(d) # Index into self.output.pattern.points of the new cursor.
                iPointTmp = iNearest
            else:
                if (0 < outputTmp.pattern.direction):
                    iPointTmp = 0
                elif (outputTmp.pattern.direction < 0):
                    iPointTmp = len(outputTmp.pattern.points)-1
                else:
                    iPointTmp = 0
                    rospy.logwarn ('pattern.direction==0.  Pattern will not advance.')
                    
                


            if (iPointPre is not None):
                iPointPost = self.output.iPoint
                outputTmp.iPoint = (iPointTmp + (iPointPost-iPointPre)) % len(outputTmp.pattern.points) # Install the new iNearest, plus a correction for processing time.
            else:
                outputTmp.iPoint = iPointTmp

                
        with self.output.lock:
            self.output.pattern   = outputTmp.pattern
            self.output.ratePoint = outputTmp.ratePoint
            self.output.pathlen   = outputTmp.pathlen
            self.output.iPoint    = outputTmp.iPoint

        #rospy.logwarn ('SetPattern_callback() points=%s' % self.output.pattern.points)
            
            
    def GetPatternPoints_callback(self, reqGetPatternPoints):
        #with self.output.lock:
        self.UpdatePatternPoints(reqGetPatternPoints.pattern)
            
        return SrvGetPatternPointsResponse(pattern=reqGetPatternPoints.pattern)
    
            
    def SendPointAndStep(self, output): 
        with output.lock:
            if (output.iPoint is not None) and (output.pattern.points is not None) and (0 < len(output.pattern.points)) and (0 < output.pattern.count):
                direction = copy.copy(output.pattern.direction)
                bSuccess = False

                pts = self.TransformPatternPoint(output)
                vecNext = self.StepNextPatternPoint(output)
                a = np.arctan2(vecNext.y, vecNext.x)

                speed = output.pathlen * output.pattern.hzPattern
                velocity = Twist(linear=Point(x = speed * np.cos(a),
                                              y = speed * np.sin(a))) # TODO: We're ignoring angular velocity.
                
                #rospy.logwarn('iPoint=%d/%d, speed=% 4.1f, x=% 4.1f' % (output.iPoint, len(output.pattern.points), speed, pts.point.x))
                state=MsgFrameState(header=pts.header,
                                    pose=Pose(position=pts.point),
                                    velocity=velocity)#
                resp = self.SignalOutput (state)
                bSuccess = resp.success
                if (not bSuccess):
                    output.pattern.direction = -direction # Back out until no longer clipped.  
                    #rospy.logwarn('clipped')

                #pt = output.pattern.points[output.iPoint]
                #rospy.logwarn('%s: (%f,%f,%f)' % (output.iPoint, pt.x, pt.y, pt.z))



    def TransformPatternPoint(self, output):
        pts = PointStamped()
                      
        try:  
            pts.header.stamp = self.tfrx.getLatestCommonTime('Arena', output.pattern.frameidPosition)
            (pos,q) = self.tfrx.lookupTransform('Arena', output.pattern.frameidPosition, pts.header.stamp)
        except tf.Exception, e:
            rospy.logwarn('Exception transforming %s->Arena in TransformPatternPoint():  %s' % (output.pattern.frameidPosition,e))
        else:
            pts.header.frame_id = 'Arena'
            pts.point = copy.copy(output.pattern.points[output.iPoint])
            pts.point.x += pos[0]
            pts.point.y += pos[1]
        
        return pts
                

    def StepNextPatternPoint(self, output):                    
        #rospy.logwarn ('PG pt=%s' % [pts.point.x, pts.point.y])
        ptCur = output.pattern.points[output.iPoint]
        output.iPoint += output.pattern.direction
        
        # When the pattern output is completed, go to the next pattern.
        if (0 < len(output.pattern.points)):
            if (output.iPoint >= len(output.pattern.points)):
                #rospy.logwarn('EndOfPattern: output.iPoint=%d, count=%d' % (output.iPoint, output.pattern.count))
                output.iPoint = 0
                if (0 < output.pattern.count):
                    output.pattern.count -= 1
                self.UpdatePatternPoints(output.pattern)

            # When the pattern output is completed, go to the next pattern.
            if (output.iPoint < 0):
                #rospy.logwarn('EndOfPattern: output.iPoint=%d, count=size %d' % (output.iPoint, output.pattern.count))
                output.iPoint = len(output.pattern.points)-1
                if (0 < output.pattern.count):
                    output.pattern.count -= 1
                self.UpdatePatternPoints(output.pattern)
            
            ptNext = output.pattern.points[output.iPoint]
            
            # Vector to the next point.
            dPoint = Point(x=ptNext.x-ptCur.x,
                           y=ptNext.y-ptCur.y)
        else:
            dPoint = Point(x=0,y=0)
            
        return dPoint

        
    def Main(self):
        while (not rospy.is_shutdown()) and (self.command != 'exit_now'):
            if (self.command != 'pause_now'):
                self.SendPointAndStep(self.output)
                
            self.output.ratePoint.sleep()

        # Shutdown all the services we offered.
        for key in self.services:
            self.services[key].shutdown()
            
            


if (__name__ == '__main__'):
    try:
        patterngen = PatternGenXY()
        patterngen.Main()
    except rospy.exceptions.ROSInterruptException: 
        pass

