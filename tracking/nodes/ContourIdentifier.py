#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('tracking')
import sys
import copy
import rospy
import tf
import numpy as N
import threading
from tracking.msg import ArenaState, Contourinfo, ContourinfoLists
from arena_tf.srv import ArenaCameraConversion
from geometry_msgs.msg import Point, PointStamped, PoseArray, Pose, PoseStamped, Quaternion, Transform, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from flycore.msg import MsgFrameState, TrackingCommand
from pythonmodules import filters, CircleFunctions, SetDict
import Fly
from munkres import Munkres
import cProfile


INDEX_X = 0
INDEX_Y = 1
INDEX_AMIN = 2
INDEX_EMIN = 3
INDEX_AMEAN = 4
INDEX_EMEAN = 5
INDEX_AMAX = 6
INDEX_EMAX = 7
INDEX_XKINEMATIC = 8
INDEX_YKINEMATIC = 9

INDEX_AREA = 2
INDEX_ECC = 3
        

###############################################################################
###############################################################################
###############################################################################
# The class ContourIdentifier subscribes to ContourinfoLists and EndEffector state, 
# and determines which of the visual contours correspond to the robot, and which 
# to the flies.  Retains identities of the individual flies, determines their
# orientation, and computes the error offset between the kinematic end effector
# position and the visual robot position.
#
# Publishes an ArenaState message, and the EndEffectorOffset.
#


                
    
###############################################################################
###############################################################################
###############################################################################
class ContourIdentifier:

    def __init__(self):
        self.initialized = False
        self.stateKinematic = None  # If no robot exists, this will remain as None.  Set in ContourinfoLists_callback.
        self.nRobots = 0
        self.nFlies = 0
        self.bUseVisualServoing = True
        
        self.contourinfo_list = []
        self.mapContourinfoFromObject = []      # A mapping from the (kalman) object number to the contourinfo number.
        self.iContours = []
        self.objects = []
        self.munkres = Munkres() # Hungarian assignment algorithm.
        self.lockParams = threading.Lock()
        self.lockThreads = threading.Lock()
        
        # Set up the input cache.
        self.cache = [None, None]
        self.iWorking = 0
        
        self.tfrx = tf.TransformListener()
        self.tfbx = tf.TransformBroadcaster()
        rospy.sleep(1)
        
        self.ResetFlyObjects()
        
        
        # Get the parameters.    
        with self.lockParams:    
            self.params = rospy.get_param('/', {})
            
        defaults = {'camera':{'mask':{'x':0, 
                                      'y':0,
                                      'radius':25}},
                    'arena':{'radius_inner':25,
                             'radius_outer':30},
                    'tracking':{'offsetEndEffectorMax': 15.0,
                                'robot':{'width':1.0,
                                         'length':1.0,
                                         'height':1.0},
                                },
                    'queue_size_contours':1
                    }
        SetDict.SetWithPreserve(self.params, defaults)

        
        # Messages
        self.subTrackingCommand     = rospy.Subscriber('tracking/command', TrackingCommand, self.TrackingCommand_callback)
        self.subContourinfoLists    = rospy.Subscriber('ContourinfoLists', ContourinfoLists, self.ContourinfoLists_callback, queue_size=self.params['tracking']['queue_size_contours'])
        self.pubArenaState          = rospy.Publisher('ArenaState', ArenaState)
        self.pubVisualState         = rospy.Publisher('visual_state', MsgFrameState)
        self.subStateEE             = rospy.Subscriber('end_effector', MsgFrameState, self.EndEffector_callback)
        self.pubStateEE             = rospy.Publisher('end_effector', MsgFrameState)
        
        self.transformEE = None
        
        # Poses
        self.poseRobot = Pose()
        self.posearrayFly = PoseArray()
        
        # Points
        self.endeffector_endeffectorframe = PointStamped()
        self.endeffector_endeffectorframe.header.frame_id = "EndEffector"
        self.endeffector_endeffectorframe.point.x = 0
        self.endeffector_endeffectorframe.point.y = 0
        self.endeffector_endeffectorframe.point.z = 0


        self.enabledExclusionzone = False
        self.pointExclusionzone_list = [Point(x=0.0, y=0.0)]
        self.radiusExclusionzone_list = [0.0]
        self.markerExclusionzone_list = []
        
        self.xSave = []
        self.ySave = []
        self.iSave = 0
        self.contouranglePrev = 0.0
        
        self.pubMarker = rospy.Publisher('visualization_marker', Marker)
        self.markerArenaOuter = Marker(header=Header(stamp = rospy.Time.now(),
                                                frame_id='Arena'),
                                  ns='arenaOuter',
                                  id=0,
                                  type=Marker.CYLINDER,
                                  action=0,
                                  pose=Pose(position=Point(x=0, 
                                                           y=0, 
                                                           z=0)),
                                  scale=Vector3(x=self.params['arena']['radius_outer']*2.0*1.05,
                                                y=self.params['arena']['radius_outer']*2.0*1.05,
                                                z=0.01),
                                  color=ColorRGBA(a=0.05,
                                                  r=1.0,
                                                  g=1.0,
                                                  b=1.0),
                                  lifetime=rospy.Duration(1.0))
        self.markerArenaInner = Marker(header=Header(stamp = rospy.Time.now(),
                                                frame_id='Arena'),
                                  ns='arenaInner',
                                  id=1,
                                  type=Marker.CYLINDER,
                                  action=0,
                                  pose=Pose(position=Point(x=0, 
                                                           y=0, 
                                                           z=0)),
                                  scale=Vector3(x=self.params['arena']['radius_inner']*2.0*1.05,
                                                y=self.params['arena']['radius_inner']*2.0*1.05,
                                                z=0.01),
                                  color=ColorRGBA(a=0.05,
                                                  r=1.0,
                                                  g=1.0,
                                                  b=1.0),
                                  lifetime=rospy.Duration(1.0))


        stSrv = 'arena_from_image'
        try:
            rospy.wait_for_service(stSrv)
            self.ArenaFromCamera = rospy.ServiceProxy(stSrv, ArenaCameraConversion, persistent=True)
        except rospy.ServiceException, e:
            rospy.logwarn ('CI FAILED to connect service %s(): %s' % (stSrv, e))

        # Open files for saving raw data.
        #self.fidRobot = open("/tmp/robot.csv", 'w')
        #self.fidRobot.write("xendeffector, yendeffector, xfiltered, yfiltered, xcontour, ycontour\n")
        #self.fidFly = open("/tmp/fly.csv", 'w')
        #self.fidFly.write("xraw, yraw, xfiltered, yfiltered\n")

        rospy.on_shutdown(self.OnShutdown_callback)

        self.timePrev = rospy.Time.now().to_sec()
        self.initialized = True


    def OnShutdown_callback(self):
        #self.fidRobot.close()
        #self.fidFly.close()
        pass
        

    # TrackingCommand_callback()
    # Receives various commands to change the tracking behavior.
    # See TrackingCommand.msg for details.
    #
    def TrackingCommand_callback(self, trackingcommand):
        with self.lockThreads:
            if trackingcommand.command=='initialize':
                self.enabledExclusionzone = trackingcommand.exclusionzones.enabled
                self.pointExclusionzone_list = trackingcommand.exclusionzones.point_list
                self.radiusExclusionzone_list = trackingcommand.exclusionzones.radius_list
                for i in range(len(self.pointExclusionzone_list)):
                    rospy.loginfo ('Tracking exclusion zone %s at (%0.2f,%0.2f) radius=%0.2f' % (['disabled','enabled'][self.enabledExclusionzone],
                                                                                                 self.pointExclusionzone_list[i].x,
                                                                                                 self.pointExclusionzone_list[i].y,
                                                                                                 self.radiusExclusionzone_list[i]))
                if self.enabledExclusionzone:
                    self.markerExclusionzone_list = []
                    for i in range(len(self.pointExclusionzone_list)):
                        self.markerExclusionzone_list.append(Marker(header=Header(stamp = rospy.Time.now(),
                                                                                  frame_id='Arena'),
                                                                    ns='exclusionzone_%d' % i,
                                                                    id=100+i,
                                                                    type=Marker.CYLINDER,
                                                                    action=0,
                                                                    pose=Pose(position=Point(x=self.pointExclusionzone_list[i].x, 
                                                                                             y=self.pointExclusionzone_list[i].y, 
                                                                                             z=0)),
                                                                    scale=Vector3(x=self.radiusExclusionzone_list[i]*2.0,
                                                                                  y=self.radiusExclusionzone_list[i]*2.0,
                                                                                  z=0.01),
                                                                    color=ColorRGBA(a=0.1,
                                                                                    r=1.0,
                                                                                    g=1.0,
                                                                                    b=1.0),
                                                                    lifetime=rospy.Duration(1.0))
                                                             )
                self.nFlies = trackingcommand.nFlies
                
                self.bUseVisualServoing = trackingcommand.bUseVisualServoing
                self.nRobots            = trackingcommand.nRobots
                    

                rospy.loginfo('CI nRobots=%d, nFlies=%d' % (self.nRobots, self.nFlies))
                self.ResetFlyObjects()
            

    def ResetFlyObjects (self):
        # Save status.
        initializedSav = self.initialized
        self.initialized = False # Unset this so we don't handle messages on the objects while we're working on them.
        
        for iObject in range(len(self.objects)):
            del self.objects[0]
        self.objects = []

        
        self.iRobot_list        = range(self.nRobots)
        self.iFly_list          = range(self.nRobots, self.nRobots+self.nFlies)
        self.iAll_list          = range(self.nRobots+self.nFlies)

        iName = 0 # Counter for the object names.
        
        # Add the robot(s), if any.
        for iRobot in self.iRobot_list:
            try:
                self.objects.append(Fly.Fly(tfrx=self.tfrx, name="Robot", lock=self.lockParams))
            except rospy.ServiceException, e:
                rospy.logwarn ('Exception adding Fly() object: %s' % e)
                
        iName += 1
            

        # Add the flies, if any.
        for iFly in self.iFly_list:
            try:
                self.objects.append(Fly.Fly(tfrx=self.tfrx, name=("Fly%02d" % iName), lock=self.lockParams))
            except rospy.ServiceException:
                rospy.logwarn ('Exception adding Fly() object: %s' % e)
            iName += 1
        

        # Restore status.
        self.initialized = initializedSav
    
    
    
    # FilterContourinfoWithinRadius()
    # Filter the contours by radius.  Return a contourinfolists containing only those within radius.
    #  
    def FilterContourinfolistsWithinMask(self, contourinfolistsIn):
        contourinfolistsOut = ContourinfoLists()
        if self.initialized:
            contourinfolistsOut.header = contourinfolistsIn.header
            contourinfolistsOut.x = []
            contourinfolistsOut.y = []
            contourinfolistsOut.angle = []
            contourinfolistsOut.area = []
            contourinfolistsOut.ecc = []
            contourinfolistsOut.imgRoi = []
            
            for iContour in range(len(contourinfolistsIn.x)):
                if N.linalg.norm(N.array([contourinfolistsIn.x[iContour]-self.params['camera']['mask']['x'], 
                                          contourinfolistsIn.y[iContour]-self.params['camera']['mask']['y']])) <= self.params['camera']['mask']['radius']:
                    contourinfolistsOut.x.append(contourinfolistsIn.x[iContour])
                    contourinfolistsOut.y.append(contourinfolistsIn.y[iContour])
                    contourinfolistsOut.angle.append(contourinfolistsIn.angle[iContour])
                    contourinfolistsOut.area.append(contourinfolistsIn.area[iContour])
                    contourinfolistsOut.ecc.append(contourinfolistsIn.ecc[iContour])
                    contourinfolistsOut.imgRoi.append(contourinfolistsIn.imgRoi[iContour])
                    
        return contourinfolistsOut
        
        
    # TransformContourinfolistsArenaFromCamera()
    # Transform the points in contourinfolistsIn to be in the Arena frame.
    #
    def TransformContourinfolistsArenaFromCamera(self, contourinfolistsIn):
        if self.initialized:
            
            try:
                response = self.ArenaFromCamera(contourinfolistsIn.x, contourinfolistsIn.y)
            except rospy.ServiceException, e:
                stSrv = 'arena_from_image'
                try:
                    rospy.wait_for_service(stSrv)
                    self.ArenaFromCamera = rospy.ServiceProxy(stSrv, ArenaCameraConversion, persistent=True)
                except rospy.ServiceException, e:
                    rospy.logwarn ('CI FAILED to connect service %s(): %s' % (stSrv, e))
                else:
                    rospy.logwarn ('CI Reconnected service %s()' % stSrv)

            
            contourinfolistsOut = copy.deepcopy(contourinfolistsIn)
            contourinfolistsOut.header.frame_id = 'Arena'
            contourinfolistsOut.x = response.xDst
            contourinfolistsOut.y = response.yDst
        else:
            contourinfolistsOut = copy.deepcopy(contourinfolistsIn)
            contourinfolistsOut.header.frame_id = 'Arena'
            contourinfolistsOut.x = []
            contourinfolistsOut.y = []
            
        return contourinfolistsOut
      
      
    # GetDistanceMatrix()
    # Get the matrix of distances between each pair of points in the two lists.  
    # xy1 and xy2 are lists of points, i.e. xy1 is M x 2, xy2 is N x 2
    #
    def GetDistanceMatrix(self, xyObjects, xyContours):
        # The basic distance matrix.
        d0 = N.subtract.outer(xyObjects[:,INDEX_X], xyContours[:,INDEX_X]) # nObjects-by-nContours matrix of x-distances between objects and contours.
        d1 = N.subtract.outer(xyObjects[:,INDEX_Y], xyContours[:,INDEX_Y]) # nObjects-by-nContours matrix of y-distances between objects and contours.
        d = N.hypot(d0, d1)                                                # nObjects-by-nContours matrix of 2-norm distances between objects and contours.
        
        # Additional distance penalties.
        
        # Make sure the robot gets first choice of the contours.
        # Set to 0 the robot's nearest contour.
        for iRobot in self.iRobot_list:
            k = d[iRobot,:].argmin()
            d[iRobot,k] = 0.0
        

        # Penalty for distance from kinematic position
        bPenalizeKinematic = False    # True only seems to make it worse.
        if (bPenalizeKinematic):
            for m in range(len(xyObjects)):
                if (N.isnan(xyObjects[m,INDEX_XKINEMATIC])):
                    xyObjects[m,INDEX_XKINEMATIC] = xyContours[m,X]
                    xyObjects[m,INDEX_YKINEMATIC] = xyContours[m,Y]
    
            d0 = N.subtract.outer(xyObjects[:,INDEX_XKINEMATIC], xyContours[:,INDEX_X])
            d1 = N.subtract.outer(xyObjects[:,INDEX_YKINEMATIC], xyContours[:,INDEX_Y])
            dPenalty = N.hypot(d0, d1)
    
            d += dPenalty
            
        
        # Penalty for deviation of visual characteristics: area and ecc.
        gainArea = 0.1 
        aPenalty = 0#gainArea * N.abs(N.subtract.outer(xyObjects[:,INDEX_AMEAN], xyContours[:,INDEX_AREA]))

        gainEcc = 10.0
        ePenalty = gainEcc * N.abs(N.subtract.outer(xyObjects[:,INDEX_EMEAN], xyContours[:,INDEX_ECC]))
        d += (aPenalty + ePenalty)
            
#             for m in range(len(xyObjects)):
#                 for n in range(len(xyContours)):
#                     try:
#                         if (bMatchArea and not N.isnan(xyObjects[m,INDEX_AMEAN])):
#                             areametric = gainArea * N.abs(xyContours[n,INDEX_AREA] - xyObjects[m,INDEX_AMEAN])
#                             d[m,n] += areametric
#                          
#                         if (bMatchEcc and not N.isnan(xyObjects[m,INDEX_EMEAN])):
#                             eccmetric = gainEcc * N.abs(xyContours[n,INDEX_ECC] - xyObjects[m,INDEX_EMEAN])# / N.max([xyContours[n,INDEX_ECC], xyObjects[m,INDEX_EMEAN]]) # Ranges 0 to 1.
#                             d[m,n] += eccmetric
#                          
#                          
#                     except TypeError:
#                         d[m,n] = None
                    
        return d
    
    
    # GetMatchGaleShapely()
    #   Solve the "Stable Marriage Problem" for two sets of points in a distance matrix.
    #   Returns the mapping.
    #
    def GetMatchGaleShapely(self, d):
        #rospy.logwarn ('CI GetMatchGaleShapely(d=%s)' % d)
        len0 = d.shape[0]
        len1 = d.shape[1]
        #Initialize all m in M and w in W to free
        M = [None for i in range(len0)]
        W = [None for i in range(len1)]
        if None not in d.flatten(): #sum(d,[]):
            # Find a stable marriage solution.
            proposed = [[0 for w in range(len1)] for m in range(len0)] 
            #while exists a free man m who still has a woman w to propose to
            while (None in W) and (None in M):
                #rospy.logwarn('M=%s'%M)
                #rospy.logwarn('W=%s'%W)
                #rospy.logwarn('P=%s'%proposed)
                if None in M:
                    m = M.index(None)
                else:
                    m = None
                if m is not None:
                    #       w = m's highest ranked such woman who he has not proposed to yet
                    d2 = list(N.array(d[m])+N.array(proposed[m]))
                    w = d2.index(min(d2))
                    #rospy.logwarn('m,w=%s'%[m,w])
                    #       if w is free
                    if W[w] is None:
                        #           (m, w) become engaged
                        M[m] = w
                        W[w] = m
                        proposed[m][w] = 999999
                        #       else some pair (m', w) already exists
                    else:
                        mp = W[w]
                        #           if w prefers m to m'
                        if d[m][w] < d[mp][w]:
                            #               (m, w) become engaged
                            M[m] = w
                            W[w] = m
                            proposed[m][w] = 999999
                            #               m' becomes free
                            M[mp] = None
                            #           else
                        else:
                            #               (m', w) remain engaged
                            M[mp] = w
                            W[w] = mp
                            proposed[mp][w] = 999999
                            proposed[m][w] = 999999
        return (M,W)        
                   
                   
    def GetMatchMunkres(self, d):
        map = [None for i in range(len(d))]
        indexes = self.munkres.compute(d)
        for k in range(len(map)):
            map[k] = indexes[k][1]
        return map
    

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


    # MapContoursFromObjects()
    #   Compute a mapping between the visual contours, on one hand, and on the other hand the tracked & kinematic positions.
    #   Uses self.contourinfo_list, self.stateKinematic, & self.objects,
    #   Returns a list of indices such that self.objects[k] = contour[map[k]], where k==0 is the robot.
    #
    def MapContoursFromObjects(self):
        # Make the list of objects (i.e. robots & flies).
        xyObjects = N.zeros([self.nRobots+self.nFlies, 10])
        
        # At least as many contour slots as objects, with missing contours placed in a "pool" located far away (e.g. 55555,55555).
        if (self.bUseVisualServoing):
            nContours = max(self.nRobots+self.nFlies, len(self.contourinfo_list))
        else:
            nContours = max(self.nRobots+self.nFlies, len(self.contourinfo_list)+self.nRobots) # Space for a fake contour to match with the robot.
            
        xyContours = N.tile([55555.0, 55555.0, 1.0, 1.0], 
                            (nContours,1)
                            )


        iContour = 0    # This keeps track of the position in the list xyContours[].
        
        # Put the contours into the contour list.
        for i in range(len(self.contourinfo_list)):
            if (not N.isnan(self.contourinfo_list[i].x)):
                xyContours[iContour,:] = N.array([self.contourinfo_list[i].x,
                                                  self.contourinfo_list[i].y,
                                                  self.contourinfo_list[i].area,
                                                  self.contourinfo_list[i].ecc])
            iContour += 1
        
        
        # Put robots into the objects list.
        for iRobot in self.iRobot_list:
            if (iRobot<len(self.objects)):
                areaMin = self.objects[iRobot].areaMin
                areaMean = self.objects[iRobot].areaMean
                areaMax = self.objects[iRobot].areaMax
                eccMin = self.objects[iRobot].eccMin
                eccMean = self.objects[iRobot].eccMean
                eccMax = self.objects[iRobot].eccMax
            else:
                areaMin = 0.0
                areaMean = 1.0
                areaMax = N.inf
                eccMin = 0.0
                eccMean = 1.0
                eccMax = N.inf
                
                
            # Get the kinematic position.
            if (self.stateKinematic is not None):
                xKinematic = self.stateKinematic.pose.position.x
                yKinematic = self.stateKinematic.pose.position.y
            else:
                xKinematic = N.nan
                yKinematic = N.nan
                
                
            # Get the robot Kalman xy.
            if (iRobot<len(self.objects)) and (self.objects[iRobot].isVisible):
                xyKalman = N.array([self.objects[iRobot].state.pose.position.x, # Position penalty    
                                   self.objects[iRobot].state.pose.position.y,  # Position penalty
                                   areaMin,  eccMin,    # Additional penalties
                                   areaMean, eccMean,   # Additional penalties
                                   areaMax,  eccMax,    # Additional penalties
                                   xKinematic,          # Additional penalties                         
                                   yKinematic])         # Additional penalties
            else:
                xyKalman = None
                
                
            # Get the robot kinematic xy.
            if (self.stateKinematic is not None):
                xyKinematic = N.array([self.stateKinematic.pose.position.x,     # Position penalty
                                         self.stateKinematic.pose.position.y,   # Position penalty
                                         areaMin,  eccMin,  # Additional penalties
                                         areaMean, eccMean, # Additional penalties
                                         areaMax,  eccMax,  # Additional penalties
                                         xKinematic,        # Additional penalties
                                         yKinematic])       # Additional penalties
            else:
                xyKinematic = None


            # Decide which position to use for robot matching.
            if (xyKalman is not None) and (xyKinematic is not None):
                # Don't let the fly walk away with the robot.
                if (N.linalg.norm(xyKalman[0:2]-xyKinematic[0:2]) < self.params['tracking']['offsetEndEffectorMax']):
                    xyRobot = xyKalman
                else:
                    xyRobot = xyKinematic
                    #rospy.logwarn('Too far away: % 0.1f' % N.linalg.norm(xyKalman[0:2]-xyKinematic[0:2]))
            else:
                if (xyKalman is not None):
                    xyRobot = xyKalman
                elif (xyKinematic is not None):
                    xyRobot = xyKinematic
                else:
                    xyRobot = None
                    
            # Put the robot into the objects list.
            if (xyRobot is not None):                    
                xyObjects[iRobot,:] = xyRobot
                
                
            # If not visual servoing, then make a fake contour exactly matching the robot.
            if (not self.bUseVisualServoing) and (xyKinematic is not None):
                xyContours[iContour,:] = N.array([xyKinematic[INDEX_X],  # x
                                                  xyKinematic[INDEX_Y],  # y
                                                  xyKinematic[INDEX_AMEAN],  # area
                                                  xyKinematic[INDEX_EMEAN]]) # ecc
                iContour += 1

        
        # Put flies into the objects list.    
        for iFly in self.iFly_list:
            xyObjects[iFly,:] = N.array([self.objects[iFly].state.pose.position.x,
                                         self.objects[iFly].state.pose.position.y,
                                         self.objects[iFly].areaMin,  self.objects[iFly].eccMin,
                                         self.objects[iFly].areaMean, self.objects[iFly].eccMean,
                                         self.objects[iFly].areaMax,  self.objects[iFly].eccMax,
                                         N.nan, N.nan]) # Kinematic positions.

        
            
        # Match objects with contourinfo_list.
        d = self.GetDistanceMatrix(xyObjects, xyContours)
        if (d is not []):
            # Choose the algorithm.
            #alg = 'galeshapely'
            alg = 'munkres'
            if (alg=='galeshapely'):
                (mapObjectsGaleShapely, mapContours) = self.GetMatchGaleShapely(d)
                mapContoursFromObjects = mapObjectsGaleShapely
                
            if (alg=='munkres'):
                mapObjectsMunkres = self.GetMatchMunkres(d)
                mapContoursFromObjects = mapObjectsMunkres
            
            # Set the augmented entries to None.
            for m in range(len(mapContoursFromObjects)):
                if (mapContoursFromObjects[m])>=len(self.contourinfo_list):
                    mapContoursFromObjects[m] = None
            
            #for i in range(len(d)):
            #    rospy.logwarn(d[i])
            #rospy.logwarn ('CI mapObjectsGaleShapely =%s' % mapObjectsGaleShapely)
            #rospy.logwarn ('CI mapObjectsMunkres     =%s' % mapObjectsMunkres)
            #rospy.logwarn ('CI mapContoursFromObjects=%s' % mapContoursFromObjects)
            #rospy.logwarn('-----')
                
            # Append a None for missing flies.
            while len(mapContoursFromObjects) < self.nRobots+self.nFlies:
                mapContoursFromObjects.append(None)
                
        else:
            mapContoursFromObjects = [None for k in self.iAll_list]
            

        # Mark the missing contours.
        for i in range(len(mapContoursFromObjects)):            
            if (len(self.contourinfo_list) <= mapContoursFromObjects[i]):
                mapContoursFromObjects[i] = None
            
#        rospy.logwarn ('----------------------------------')
#        if (self.nRobots==1):
#            rospy.logwarn ('CI xyKalman=%s' % xyKalman)

#        rospy.logwarn ('CI xyObjects=%s' % xyObjects)
#        rospy.logwarn ('CI xyContours=%s' % xyContours)
#        rospy.logwarn ('CI mapObjects=%s' % (mapContoursFromObjects))
        
        return mapContoursFromObjects
    
    
    def PublishArenaStateFromObjects(self):                    
        # Construct the ArenaState message.
        arenastate = ArenaState()
        #if self.objects[0].state.pose.position.x is not None:
        for iRobot in self.iRobot_list:
            if (iRobot < len(self.objects)):
                arenastate.robot.header.stamp    = self.objects[iRobot].state.header.stamp
                arenastate.robot.header.frame_id = self.objects[iRobot].state.header.frame_id
                arenastate.robot.name            = self.objects[iRobot].name
                arenastate.robot.pose            = self.objects[iRobot].state.pose
                arenastate.robot.velocity        = self.objects[iRobot].state.velocity
                arenastate.robot.wings           = self.objects[iRobot].state.wings
                arenastate.robot.speed           = self.objects[iRobot].speed
        
        for iFly in self.iFly_list:
            if (iFly < len(self.objects)):
                arenastate.flies.append(MsgFrameState(header   = self.objects[iFly].state.header, 
                                                      name     = self.objects[iFly].name,
                                                      pose     = self.objects[iFly].state.pose,
                                                      velocity = self.objects[iFly].state.velocity,
                                                      wings    = self.objects[iFly].state.wings,
                                                      speed    = min(50.0, self.objects[iFly].speed)))

        
        # Publish the ArenaState.
        self.pubArenaState.publish(arenastate)
        

    # Bring in the end effector state via messages.        
    def EndEffector_callback(self, stateKinematic):
        self.stateKinematic = stateKinematic
        

    def ContourinfoLists_callback(self, contourinfolistsPixels):
        # Point to the cache non-working entry.
        iLoading = (self.iWorking+1) % 2
        
        self.cache[iLoading] = contourinfolistsPixels 
    
    
    def ProcessContourinfoLists(self):
        if (self.cache[self.iWorking] is not None):
            contourinfolistsPixels = self.cache[self.iWorking]
            with self.lockThreads:
                try:
                    if (self.initialized):
    
                        # Publish state of the EndEffector for ourselves (so we get the EE via bag-recordable message rather than via tf. 
                        if (0 < self.nRobots):
                            try:
                                stamp = self.tfrx.getLatestCommonTime('Arena', 'EndEffector')
                                (translationEE,rotationEE) = self.tfrx.lookupTransform('Arena', 'EndEffector', stamp)
                            except tf.Exception, e:
                                pass    # No transform - Either the EE is still initializing, or because we're replaying a bag file.
                                #rospy.logwarn ('CI EndEffector not yet initialized: %s' % e)
                            else:
                                self.pubStateEE.publish(MsgFrameState(header=Header(stamp=contourinfolistsPixels.header.stamp,
                                                                                    frame_id='Arena'),
                                                                      pose=Pose(position=Point(translationEE[0],translationEE[1],translationEE[2]),
                                                                                orientation=Quaternion(rotationEE[0],rotationEE[1],rotationEE[2],rotationEE[3]))
                                                                      )
                                                        )
            
                        # Apply the arena mask, and transform to arena frame.
                        contourinfolistsPixels = self.FilterContourinfolistsWithinMask(contourinfolistsPixels)
                        contourinfolists = self.TransformContourinfolistsArenaFromCamera(contourinfolistsPixels)
                        
            
                        # Repackage the contourinfolists as a list of contourinfos, ignoring any that are in an exclusion zone.
                        self.contourinfo_list = []            
                        for i in range(len(contourinfolists.x)):
                            inExclusionzone = False
                            if (self.enabledExclusionzone):
                                # See if the contourinfo is in any of the exclusionzones.
                                for k in range(len(self.pointExclusionzone_list)):
                                    inExclusionzone = inExclusionzone or (N.linalg.norm([contourinfolists.x[i]-self.pointExclusionzone_list[k].x, 
                                                                                         contourinfolists.y[i]-self.pointExclusionzone_list[k].y]) < self.radiusExclusionzone_list[k])
                                
                            if (not inExclusionzone): 
                                contourinfo = Contourinfo()
                                contourinfo.header = contourinfolists.header
                                contourinfo.x      = contourinfolists.x[i]
                                contourinfo.y      = contourinfolists.y[i]
                                if (not N.isnan(contourinfolists.angle[i])):
                                    contourinfo.angle = contourinfolists.angle[i]
                                else:
                                    contourinfo.angle = self.contouranglePrev
                                self.contouranglePrev = contourinfo.angle
                                
                                contourinfo.area   = contourinfolists.area[i]
                                contourinfo.ecc    = contourinfolists.ecc[i]
                                contourinfo.imgRoi = contourinfolists.imgRoi[i]
                                self.contourinfo_list.append(contourinfo)
                
                        
                        # Figure out who is who in the camera image.
                        try:
                            self.mapContourinfoFromObject = self.MapContoursFromObjects()
                        except IndexError:
                            self.mapContourinfoFromObject = None
            
                        if (self.mapContourinfoFromObject is not None):
                            # Update the robot state w/ the contourinfo and end-effector positions.
                            for iRobot in self.iRobot_list:
                                if (self.mapContourinfoFromObject[iRobot] is not None):
                                    
                                    # If the map points to a contourinfo, then use it.
                                    # For the robot, use the end-effector angle instead of the contourinfo angle.
                                    if (self.stateKinematic is not None):
                                        q = self.stateKinematic.pose.orientation
                                        rpy = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
                                        self.contourinfo_list[self.mapContourinfoFromObject[iRobot]].angle = rpy[2]
                                    
                                    # Use this contourinfo.    
                                    contourinfo = self.contourinfo_list[self.mapContourinfoFromObject[iRobot]]
                                    
                                # Else use the kinematic state, if available.
                                elif (self.stateKinematic is not None):
                                    q = self.stateKinematic.pose.orientation
                                    rpy = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
                                    area = N.pi * (self.params['tracking']['robot']['width']/2)**2
    
                                    # Make a 'fake' contourinfo from kinematics.    
                                    contourinfo = Contourinfo(header=self.stateKinematic.header,
                                                              x=self.stateKinematic.pose.position.x,
                                                              y=self.stateKinematic.pose.position.y,
                                                              angle=rpy[2],
                                                              area=area,
                                                              ecc=1.0,
                                                              imgRoi=None
                                                              )
                                else:
                                    contourinfo = None
                                     
                                    
                                # Update the object.
                                if (iRobot < len(self.objects)):
                                    self.objects[iRobot].Update(contourinfo)
            
                                # Write a file (for getting Kalman covariances, etc).
                                #data = '%s, %s, %s, %s, %s, %s\n' % (self.stateKinematic.pose.position.x,
                                #                                     self.stateKinematic.pose.position.y,
                                #                                     self.objects[iRobot].state.pose.position.x, 
                                #                                     self.objects[iRobot].state.pose.position.y,
                                #                                     self.contourinfo_list[self.mapContourinfoFromObject[iRobot]].x,
                                #                                     self.contourinfo_list[self.mapContourinfoFromObject[iRobot]].y)
                                #self.fidRobot.write(data)
                                    
                            # Update the flies' states.
                            for iFly in self.iFly_list:
                                if (self.mapContourinfoFromObject[iFly] is not None):
                                    contourinfo = self.contourinfo_list[self.mapContourinfoFromObject[iFly]]
                                else:
                                    contourinfo = None
                                
                                if (iFly < len(self.objects)):
                                    self.objects[iFly].Update(contourinfo)
                                    
                                # Write a file.
                                #if self.mapContourinfoFromObject[1] is not None:
                                #    data = '%s, %s, %s, %s\n' % (self.contourinfo_list[self.mapContourinfoFromObject[1]].x, 
                                #                                 self.contourinfo_list[self.mapContourinfoFromObject[1]].y, 
                                #                                 self.objects[1].state.pose.position.x, 
                                #                                 self.objects[1].state.pose.position.y)
                                #    self.fidFly.write(data)
                            
            
                            self.PublishArenaStateFromObjects()
            
                            # Publish the visual_state.
                            if (self.bUseVisualServoing) and (0 in self.iRobot_list) and (0 < len(self.objects)):
                                self.pubVisualState.publish(self.objects[0].state)
                            
                            
                            # Publish a marker to indicate the size of the arena.
                            self.markerArenaOuter.header.stamp = contourinfolists.header.stamp
                            self.markerArenaInner.header.stamp = contourinfolists.header.stamp
                            self.pubMarker.publish(self.markerArenaOuter)
                            self.pubMarker.publish(self.markerArenaInner)
            
                            # Publish markers for all the exclusionzones.
                            if self.enabledExclusionzone:
                                for marker in self.markerExclusionzone_list:
                                    marker.header.stamp = contourinfolists.header.stamp
                                    self.pubMarker.publish(marker)
            
                except rospy.exceptions.ROSException, e:
                    rospy.loginfo('CI ROSException: %s' % e)
                      
            # Mark this entry as done.
            self.cache[self.iWorking] = None
            
        # Go to the other image.
        self.iWorking = (self.iWorking+1) % 2


    def main(self):
        while (not rospy.is_shutdown()):
            ci.ProcessContourinfoLists()

    
if __name__ == '__main__':
    rospy.init_node('ContourIdentifier')
    ci = ContourIdentifier()
    ci.main()
    #cProfile.run('ci.main()', '/home/rancher/profile.pstats')    
    