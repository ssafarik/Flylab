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
from geometry_msgs.msg import Point, PointStamped, PoseArray, Pose, PoseStamped, Quaternion, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from flycore.msg import MsgFrameState, TrackingCommand
from pythonmodules import filters
from pythonmodules import CircleFunctions
import Fly
from munkres import Munkres

INDEX_X = 0
INDEX_Y = 1
INDEX_AMIN = 2
INDEX_EMIN = 3
INDEX_AMEAN = 4
INDEX_EMEAN = 5
INDEX_AMAX = 6
INDEX_EMAX = 7
INDEX_XCOMPUTED = 8
INDEX_YCOMPUTED = 9

INDEX_AREA = 2
INDEX_ECC = 3
        

###############################################################################
###############################################################################
###############################################################################
# The class ContourIdentifier subscribes to ContourinfoLists and EndEffector state, 
# and determines which of the visual contours correspond to the robot, and which 
# to the flies.  Retains identities of the individual flies, determines their
# orientation, and computes the error offset between the computed end effector
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
        self.stateEndEffector = None  # If no robot exists, this will remain as None.  Set in ContourinfoLists_callback.
        self.nFlies = rospy.get_param('nFlies', 0)
        self.nRobots = rospy.get_param('nRobots', 0)
        
        self.contourinfo_list = []
        self.mapContourinfoFromObject = []      # A mapping from the (kalman) object number to the contourinfo number.
        self.iContours = []
        self.objects = []
        self.munkres = Munkres() # Hungarian assignment algorithm.
        self.lock = threading.Lock()
        
        self.tfrx = tf.TransformListener()
        self.tfbx = tf.TransformBroadcaster()
        
        self.ResetFlyObjects()
        
        
        # Messages
        queue_size_contours         = rospy.get_param('tracking/queue_size_contours', 1)
        self.subTrackingCommand     = rospy.Subscriber('tracking/command', TrackingCommand, self.TrackingCommand_callback)
        self.subContourinfoLists    = rospy.Subscriber('ContourinfoLists', ContourinfoLists, self.ContourinfoLists_callback, queue_size=queue_size_contours)
        self.pubArenaState          = rospy.Publisher('ArenaState', ArenaState)
        self.pubVisualState         = rospy.Publisher('VisualState', MsgFrameState)
        
        # Poses
        self.poseRobot = Pose()
        self.posearrayFly = PoseArray()
        
        # Points
        self.endeffector_endeffectorframe = PointStamped()
        self.endeffector_endeffectorframe.header.frame_id = "EndEffector"
        self.endeffector_endeffectorframe.point.x = 0
        self.endeffector_endeffectorframe.point.y = 0
        self.endeffector_endeffectorframe.point.z = 0
        

        self.xMask = rospy.get_param("camera/mask/x", 0) # Pixels
        self.yMask = rospy.get_param("camera/mask/y", 0) # Pixels
        self.radiusMask = rospy.get_param("camera/mask/radius", 25) # Pixels
        self.radiusArenaInner = rospy.get_param("arena/radius_inner", 25) # Millimeters
        self.radiusArenaOuter = rospy.get_param("arena/radius_outer", 30) # Millimeters
        self.offsetEndEffectorMax = rospy.get_param('tracking/offsetEndEffectorMax', 15.0)
        
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
                                                frame_id='/Arena'),
                                  ns='arenaOuter',
                                  id=0,
                                  type=Marker.CYLINDER,
                                  action=0,
                                  pose=Pose(position=Point(x=0, 
                                                           y=0, 
                                                           z=0)),
                                  scale=Vector3(x=self.radiusArenaOuter*2.0*1.05,
                                                y=self.radiusArenaOuter*2.0*1.05,
                                                z=0.01),
                                  color=ColorRGBA(a=0.05,
                                                  r=1.0,
                                                  g=1.0,
                                                  b=1.0),
                                  lifetime=rospy.Duration(1.0))
        self.markerArenaInner = Marker(header=Header(stamp = rospy.Time.now(),
                                                frame_id='/Arena'),
                                  ns='arenaInner',
                                  id=1,
                                  type=Marker.CYLINDER,
                                  action=0,
                                  pose=Pose(position=Point(x=0, 
                                                           y=0, 
                                                           z=0)),
                                  scale=Vector3(x=self.radiusArenaInner*2.0*1.05,
                                                y=self.radiusArenaInner*2.0*1.05,
                                                z=0.01),
                                  color=ColorRGBA(a=0.05,
                                                  r=1.0,
                                                  g=1.0,
                                                  b=1.0),
                                  lifetime=rospy.Duration(1.0))


        try:
            rospy.wait_for_service('arena_from_camera') #, timeout=10.0)
            self.ArenaFromCamera = rospy.ServiceProxy('arena_from_camera', ArenaCameraConversion)
        except (rospy.ServiceException, IOError), e:
            print "Service call failed: %s"%e

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
        if trackingcommand.command=='setexclusionzones':
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
            self.ResetFlyObjects()
        

    def ResetFlyObjects (self):
        with self.lock:
            # Save status.
            #rospy.logwarn ('ResetFlyObjects()A: initialized=%s' % self.initialized)
            initializedSav = self.initialized
            self.initialized = False
            
            for iObject in range(len(self.objects)):
                del self.objects[0]
            self.objects = []
    
            
            self.iRobot_list = range(self.nRobots)
            self.iFly_list = range(self.nRobots, self.nRobots+self.nFlies)
            self.iAll_list = range(self.nRobots+self.nFlies)
    
            iName = 0 # Counter for the object names.
            
            # Add the robot(s), if any.
            for iRobot in self.iRobot_list:
                try:
                    self.objects.append(Fly.Fly(tfrx=self.tfrx, name="Robot"))
                except rospy.ServiceException, e:
                    rospy.logwarn ('Exception adding Fly() object: %s' % e)
                    
            iName += 1
                
    
            # Add the flies, if any.
            for iFly in self.iFly_list:
                try:
                    self.objects.append(Fly.Fly(tfrx=self.tfrx, name=("Fly%s" % iName)))
                except rospy.ServiceException:
                    rospy.logwarn ('Exception adding Fly() object: %s' % e)
                iName += 1
            
    
            # Restore status.
            self.initialized = initializedSav
            #rospy.logwarn ('ResetFlyObjects()B: initialized=%s' % self.initialized)
        
        
        
    # FilterContourinfoWithinRadius()
    # Filter the contours by radius.  Return a contourinfolists containing only those within radius.
    #  
    def FilterContourinfoWithinMask(self, contourinfolistsIn):
        contourinfolistsOut = ContourinfoLists()
        if self.initialized:
            contourinfolistsOut.header = contourinfolistsIn.header
            contourinfolistsOut.x = []
            contourinfolistsOut.y = []
            contourinfolistsOut.angle = []
            contourinfolistsOut.area = []
            contourinfolistsOut.ecc = []
            contourinfolistsOut.imgRoi = []
            
            #contourinfolistsArena = TransformContourinfoArenaFromCamera(contourinfolistsIn)
            for iContour in range(len(contourinfolistsIn.x)):
                if N.linalg.norm(N.array([contourinfolistsIn.x[iContour]-self.xMask, 
                                          contourinfolistsIn.y[iContour]-self.yMask])) <= self.radiusMask:
                    contourinfolistsOut.x.append(contourinfolistsIn.x[iContour])
                    contourinfolistsOut.y.append(contourinfolistsIn.y[iContour])
                    contourinfolistsOut.angle.append(contourinfolistsIn.angle[iContour])
                    contourinfolistsOut.area.append(contourinfolistsIn.area[iContour])
                    contourinfolistsOut.ecc.append(contourinfolistsIn.ecc[iContour])
                    contourinfolistsOut.imgRoi.append(contourinfolistsIn.imgRoi[iContour])
                    
        return contourinfolistsOut
        
        
    # TransformContourinfoArenaFromCamera()
    # Transform the points in contourinfolistsIn to be in the Arena frame.
    #
    def TransformContourinfoArenaFromCamera(self, contourinfolistsIn):
        #contourinfolistsOut = ContourinfoLists()
        if self.initialized:
            response = self.ArenaFromCamera(contourinfolistsIn.x, contourinfolistsIn.y)
            contourinfolistsOut = copy.copy(contourinfolistsIn)
            contourinfolistsOut.header.frame_id = 'Arena'
            contourinfolistsOut.x = response.xDst
            contourinfolistsOut.y = response.yDst
        else:
            contourinfolistsOut = copy.copy(contourinfolistsIn)
            contourinfolistsOut.header.frame_id = 'Arena'
            contourinfolistsOut.x = []
            contourinfolistsOut.y = []
            
        return contourinfolistsOut
      
      
    # GetDistanceMatrix()
    # Get the matrix of distances between each pair of points in the two lists.  
    # xy1 and xy2 are lists of points, i.e. xy1 is M x 2, xy2 is N x 2
    #
    def GetDistanceMatrixFromContours(self, xyObjects, xyContours):
        #rospy.logwarn('xyObjects=%s' % xyObjects)
        #rospy.logwarn('xyContours=%s' % xyContours)
        
        # The basic distance matrix.
        d0 = N.subtract.outer(xyObjects[:,INDEX_X], xyContours[:,INDEX_X])
        d1 = N.subtract.outer(xyObjects[:,INDEX_Y], xyContours[:,INDEX_Y])
        d = N.hypot(d0, d1)
        
        # Additional distance penalties.
        
        # Make sure the robot gets first choice of the contours.
        for iRobot in range(self.nRobots):
            k = d[iRobot,:].argmin()
            d[iRobot,k] = 0.0
        
        # Add penalty to non-robot distances.
#         dPenalty = N.ones(d.shape)
#         dPenalty *= 1000.0
#         for iRobot in range(self.nRobots):
#             dPenalty[iRobot,:] = 0
#             
#         d += dPenalty


        # Penalty for distance from computed position
        bPenalizeComputed = False
        if (bPenalizeComputed):
            for m in range(len(xyObjects)):
                if (N.isnan(xyObjects[m,INDEX_XCOMPUTED])):
                    xyObjects[m,INDEX_XCOMPUTED] = xyContours[m,X]
                    xyObjects[m,INDEX_YCOMPUTED] = xyContours[m,Y]
    
            d0 = N.subtract.outer(xyObjects[:,INDEX_XCOMPUTED], xyContours[:,INDEX_X])
            d1 = N.subtract.outer(xyObjects[:,INDEX_YCOMPUTED], xyContours[:,INDEX_Y])
            dPenalty = N.hypot(d0, d1)
    
            d += dPenalty
        
        #rospy.logwarn('area=%s,%s' % (xyContours[:,INDEX_AREA], xyObjects[:,INDEX_AMEAN]))#, xyContours[:,INDEX_ECC], xyObjects[:,INDEX_EMEAN]))
        
        # Penalty for deviation from visual characteristics.
        bMatchVisual = False    # True only seems to make it worse.
        if (bMatchVisual):    
            for m in range(len(xyObjects)):
                for n in range(len(xyContours)):
                    try:
                        if (not N.isnan(xyObjects[m,INDEX_AMEAN])):
                            gainArea = 0.1 
                            areametric = gainArea * N.abs(xyContours[n,INDEX_AREA] - xyObjects[m,INDEX_AMEAN])
                            d[m,n] += areametric
                         
                        if (not N.isnan(xyObjects[m,INDEX_EMEAN])):
                            gainEcc = 10.0
                            eccmetric = gainEcc * N.abs(xyContours[n,INDEX_ECC] - xyObjects[m,INDEX_EMEAN]) / N.max([xyContours[n,INDEX_ECC], xyObjects[m,INDEX_EMEAN]]) # Ranges 0 to 1.
                            d[m,n] += eccmetric
                         
                         
                    except TypeError:
                        d[m,n] = None
                    
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
    #   Uses self.contourinfo_list & self.objects,
    #   Returns a list of indices such that self.objects[k] = contour[map[k]], and self.objects[0]=therobot
    #
    def MapContoursFromObjects(self):
        # Time.
        if (self.stateEndEffector is not None):
            stamp = self.stateEndEffector.header.stamp
        elif (len(self.objects)>0) and (self.objects[0].isVisible):
            stamp = self.objects[0].state.header.stamp
        else:
            stamp = rospy.Time.now()


        # Make the list of objects (i.e. robots & flies).
        xyObjects = N.zeros([self.nRobots+self.nFlies, 10])
        
        # Robots.
        for iRobot in self.iRobot_list:
#            if (self.stateEndEffector is not None):
#                xyRobot = [self.stateEndEffector.pose.position.x,
#                                 self.stateEndEffector.pose.position.y]
#                #rospy.logwarn ('CI Robot image at %s' % ([self.objects[0].state.pose.position.x,
#                #                                          self.objects[0].state.pose.position.y]))
#            elif (iRobot<len(self.objects)) and (self.objects[iRobot].isVisible):
#                xyRobot = [self.objects[iRobot].state.pose.position.x,
#                                 self.objects[iRobot].state.pose.position.y]
#            else:
#                xyRobot = [0.0, 0.0]

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
                
                
            if (iRobot<len(self.objects)) and (self.objects[iRobot].isVisible) and (self.stateEndEffector is not None):
                xyRobot = N.array([self.objects[iRobot].state.pose.position.x,
                                   self.objects[iRobot].state.pose.position.y,
                                   areaMin,  eccMin,
                                   areaMean, eccMean,
                                   areaMax,  eccMax,
                                   self.stateEndEffector.pose.position.x,                         
                                   self.stateEndEffector.pose.position.y])
            else:
                xyRobot = None
                
            if (self.stateEndEffector is not None):
                xyEndEffector = N.array([self.stateEndEffector.pose.position.x,
                                         self.stateEndEffector.pose.position.y,
                                         areaMin,  eccMin,
                                         areaMean, eccMean,
                                         areaMax,  eccMax,
                                         self.stateEndEffector.pose.position.x, 
                                         self.stateEndEffector.pose.position.y])
                #rospy.logwarn ('CI Robot image at %s' % ([self.objects[0].state.pose.position.x,
                #                                          self.objects[0].state.pose.position.y]))
            else:
                xyEndEffector = None


            # Decide which position to use for robot matching.
            if (xyRobot is not None) and (xyEndEffector is not None):
                # Don't let the fly walk away with the robot.
                if (N.linalg.norm(xyRobot[0:2]-xyEndEffector[0:2]) < self.offsetEndEffectorMax):
                    xy = xyRobot
                else:
                    xy = xyEndEffector
                    #rospy.logwarn('Too far away: % 0.1f' % N.linalg.norm(xyRobot[0:2]-xyEndEffector[0:2]))
            else:
                if (xyRobot is not None):
                    xy = xyRobot
                elif (xyEndEffector is not None):
                    xy = xyEndEffector
                else:
                    xy = None
                    
            if (xy is not None):                    
                xyObjects[iRobot,:] = xy
                self.tfbx.sendTransform((xy[0], xy[1], 0.0),
                                        tf.transformations.quaternion_about_axis(0, (0,0,1)),
                                        stamp,
                                        "RobotComputed",
                                        "Arena")


        # Flies.    
        for iFly in self.iFly_list:
            #if self.objects[iFly].isVisible:
                xyObjects[iFly,:] = N.array([self.objects[iFly].state.pose.position.x,
                                             self.objects[iFly].state.pose.position.y,
                                             self.objects[iFly].areaMin,  self.objects[iFly].eccMin,
                                             self.objects[iFly].areaMean, self.objects[iFly].eccMean,
                                             self.objects[iFly].areaMax,  self.objects[iFly].eccMax,
                                             N.nan, N.nan]) # Computed positions.
                #rospy.loginfo ('CI Object %s at x,y=%s' % (iFly,[self.objects[iFly].state.pose.position.x,
                #                                                 self.objects[iFly].state.pose.position.y]))
            
        # Make the list of contours.
        # At least as many contour slots as objects, with missing contours placed in a "pool" located far away (e.g. 55555,55555).
        xyContours = N.tile([55555.0, 55555.0, 1.0, 1.0], 
                            (max(len(xyObjects),len(self.contourinfo_list)),1)) 
        for iContour in range(len(self.contourinfo_list)):
            if (not N.isnan(self.contourinfo_list[iContour].x)):
                xyContours[iContour,:] = N.array([self.contourinfo_list[iContour].x,
                                                  self.contourinfo_list[iContour].y,
                                                  self.contourinfo_list[iContour].area,  self.contourinfo_list[iContour].ecc])
                #rospy.loginfo ('CI contourinfo %s at x,y=%s' % (iContour,[self.contourinfo_list[iContour].x,
                #                                               self.contourinfo_list[iContour].y]))
        
        # Print ecc/area stats.
        #rospy.logwarn ('robot,fly ecc=[%0.2f, %0.2f], area=[%0.2f, %0.2f]' % (self.objects[0].eccMean,
        #                                                                      self.objects[1].eccMean,
        #                                                                      self.objects[0].areaMean,
        #                                                                      self.objects[1].areaMean))
        
        
        # Match objects with contourinfo_list.
#        d = self.GetDistanceMatrixFromContours(xyObjects, contourinfo_listAug, contourinfoMin_list, contourinfoMax_list, contourinfoMean_list, ptComputed)
        d = self.GetDistanceMatrixFromContours(xyObjects, xyContours)
        if d is not []:
            # Choose the algorithm.
            #alg = 'galeshapely'
            alg = 'munkres'
            if alg=='galeshapely':
                (mapObjectsGaleShapely, mapContours) = self.GetMatchGaleShapely(d)
                mapContoursFromObjects = mapObjectsGaleShapely
                
            if alg=='munkres':
                mapObjectsMunkres = self.GetMatchMunkres(d)
                mapContoursFromObjects = mapObjectsMunkres
                
            #rospy.logwarn('mapContoursFromObjects=%s'%mapContoursFromObjects)
            #rospy.logwarn('len(mapContoursFromObjects)=%d'%len(mapContoursFromObjects))
            #rospy.logwarn('len(xyContours)=%d'%len(xyContours))
            # Set the augmented entries to None.
            for m in range(len(mapContoursFromObjects)):
                if (mapContoursFromObjects[m])>=len(self.contourinfo_list):
                    mapContoursFromObjects[m] = None

            #rospy.logwarn ('CI mapObjectsGaleShapely =%s' % (mapObjectsGaleShapely))
            #rospy.logwarn ('CI mapObjectsMunkres  =%s' % mapObjectsMunkres)
    
            #rospy.logwarn ('CI mapContoursFromObjects=%s' % mapContoursFromObjects)
                
            # Prepend a None for a missing robot.
            #if self.stateEndEffector is None:
            #    mapContoursFromObjects.insert(0,None)
                
            # Append a None for missing flies.
            while len(mapContoursFromObjects) < self.nRobots+self.nFlies:
                mapContoursFromObjects.append(None)
                
        else:
            mapContoursFromObjects = [None for k in self.iAll_list]
            
#        rospy.logwarn ('----------------------------------')
#        if (self.nRobots==1):
#            rospy.logwarn ('CI xyRobot=%s' % xyRobot)

#        rospy.logwarn ('CI xyObjects=%s' % xyObjects)
#        rospy.logwarn ('CI xyContours=%s' % xyContours)
#        if self.stateEndEffector is not None:
#            rospy.logwarn ('CI ptComputed=(%0.2f+%0.2f, %0.2f+%0.2f)' % (self.stateEndEffector.pose.position.x, 
#                                                                         self.objects[0].ptOffset.x,
#                                                                         self.stateEndEffector.pose.position.y, 
#                                                                         self.objects[0].ptOffset.y))
#        rospy.logwarn ('CI mapObjects=%s' % (mapContoursFromObjects))
#        rospy.logwarn('d=\n%s'%d)
        
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
        

    def ContourinfoLists_callback(self, contourinfolistsPixels):
        #rospy.logwarn('ContourinfoLists_callback(now-prev=%s)' % (rospy.Time.now().to_sec()-self.timePrev))
#        self.timePrev = rospy.Time.now().to_sec()
        if self.initialized:
            # Get state of the EndEffector.
            if self.nRobots>0:
                try:
                    stamp = self.tfrx.getLatestCommonTime('Arena', 'EndEffector')
                    (transEndEffector, quatEndEffector) = self.tfrx.lookupTransform('Arena', 'EndEffector', stamp)
                except tf.Exception, e:
                    rospy.logwarn ('CI EndEffector not yet initialized: %s' % e)
                else:
                    if self.stateEndEffector is None:
                        self.ResetFlyObjects() # When first data, need to reset all the fly objects due to tracking of robot contour as a fly, hence having an extra object.
                        self.stateEndEffector = MsgFrameState()
                        
                    self.stateEndEffector.header.stamp = stamp
                    self.stateEndEffector.header.frame_id = 'Arena'
                    self.stateEndEffector.pose.position.x = transEndEffector[0]
                    self.stateEndEffector.pose.position.y = transEndEffector[1]
                    self.stateEndEffector.pose.position.z = transEndEffector[2]
                    self.stateEndEffector.pose.orientation.x = quatEndEffector[0]
                    self.stateEndEffector.pose.orientation.y = quatEndEffector[1]
                    self.stateEndEffector.pose.orientation.z = quatEndEffector[2]
                    self.stateEndEffector.pose.orientation.w = quatEndEffector[3]

            contourinfolistsPixels = self.FilterContourinfoWithinMask(contourinfolistsPixels)
            contourinfolists = self.TransformContourinfoArenaFromCamera(contourinfolistsPixels)

            # Create a null contourinfo.
            contourinfoNone = Contourinfo()
            contourinfoNone.header = contourinfolists.header
            contourinfoNone.x = None
            contourinfoNone.y = None
            contourinfoNone.angle = None
            contourinfoNone.area = None
            contourinfoNone.ecc = None
            contourinfoNone.imgRoi = None

            # Repackage the contourinfolists into a list of contourinfos, ignoring any that are in the exclusion zone.
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
                    if (contourinfolists.angle[i] != 99.9) and (not N.isnan(contourinfolists.angle[i])):
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

                
            #rospy.logwarn ('CI map=%s' % self.mapContourinfoFromObject)
            #for i in range(len(self.contourinfo_list)):
            #    rospy.logwarn ('CI contour[%d].x,y=%s' % (i,[self.contourinfo_list[i].x,self.contourinfo_list[i].y]))
            
            # Update the robot state w/ the contourinfo and end-effector positions.
            if self.mapContourinfoFromObject is not None:
                for iRobot in self.iRobot_list:
                    if (self.stateEndEffector is not None):
                        if self.mapContourinfoFromObject[iRobot] is not None:
                            # For the robot, use the end-effector angle instead of the contourinfo angle.
                            if self.stateEndEffector is not None:
                                q = self.stateEndEffector.pose.orientation
                                rpy = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
                                self.contourinfo_list[self.mapContourinfoFromObject[iRobot]].angle = rpy[2]

                            contourinfo = self.contourinfo_list[self.mapContourinfoFromObject[iRobot]]
                        else:
                            contourinfo = contourinfoNone
                             
                        if (iRobot < len(self.objects)):
                            self.objects[iRobot].Update(contourinfo, 
                                                        PoseStamped(header=self.stateEndEffector.header, 
                                                                    pose=self.stateEndEffector.pose))

                        # Write a file (for getting Kalman covariances, etc).
                        #data = '%s, %s, %s, %s, %s, %s\n' % (self.stateEndEffector.pose.position.x,
                        #                                     self.stateEndEffector.pose.position.y,
                        #                                     self.objects[iRobot].state.pose.position.x, 
                        #                                     self.objects[iRobot].state.pose.position.y,
                        #                                     self.contourinfo_list[self.mapContourinfoFromObject[iRobot]].x,
                        #                                     self.contourinfo_list[self.mapContourinfoFromObject[iRobot]].y)
                        #self.fidRobot.write(data)
                        
                        #rospy.loginfo ('CI update robot    contour=%s' % contourinfo)
                    
#                    rospy.logwarn('contourinfolists.angle[]=%s' % contourinfolists.angle)
#                    rospy.logwarn('map=%s' % self.mapContourinfoFromObject)
                
                # Update the flies' states.
                for iFly in self.iFly_list:
                    if self.mapContourinfoFromObject[iFly] is not None:
                        contourinfo = self.contourinfo_list[self.mapContourinfoFromObject[iFly]]
                    else:
                        contourinfo = contourinfoNone
                        #rospy.logwarn ('No contourinfo for fly %d' % iFly)
                    
                    if (iFly < len(self.objects)):
                        self.objects[iFly].Update(contourinfo, None)
                        
                    #self.stateEndEffector.header.stamp,#rospy.Time.now()
                    #rospy.loginfo ('CI update state %s contour=%s' % (iFly,contourinfo))
    
                    # Write a file.
                    #if self.mapContourinfoFromObject[1] is not None:
                    #    data = '%s, %s, %s, %s\n' % (self.contourinfo_list[self.mapContourinfoFromObject[1]].x, 
                    #                                 self.contourinfo_list[self.mapContourinfoFromObject[1]].y, 
                    #                                 self.objects[1].state.pose.position.x, 
                    #                                 self.objects[1].state.pose.position.y)
                    #    self.fidFly.write(data)
                

    
                self.PublishArenaStateFromObjects()

                
                # Publish the VisualState.
                if (0 in self.iRobot_list) and (0 < len(self.objects)):
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

                        

    
if __name__ == '__main__':
    rospy.init_node('ContourIdentifier')
    ci = ContourIdentifier()
    rospy.spin()
    
    