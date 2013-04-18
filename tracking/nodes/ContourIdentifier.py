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
    # Get the matrix of distances between each pair of points in the two lists.  Adjust distances with
    # priorities, where smaller=better.
    # xy1 and xy2 are lists of points, i.e. xy1 is M x 2, xy2 is N x 2
    #
    def GetDistanceMatrix(self, xy1, xy2, iPriorities=None):
        d = N.array([[N.inf for n in range(len(xy2))] for m in range(len(xy1))])
        for m in range(len(xy1)):
            for n in range(len(xy2)):
                try:
                    d[m,n] = N.linalg.norm([xy1[m][0]-xy2[n][0],
                                             xy1[m][1]-xy2[n][1]])
                    # Quick & dirty priority calc.
                    if iPriorities is not None:
                        d[m,n] += iPriorities[m]
                except TypeError:
                    d[m,n] = None
        #rospy.loginfo('CI DM xy1=%s' % xy1)
        #rospy.loginfo('CI DM xy2=%s' % xy2)
        #rospy.loginfo('CI DM d=%s' % d)
        return d
    
    
    def GetDistanceMatrixFromContours(self, xyObjects, contourinfo_list, contourinfoMin_list, contourinfoMax_list, contourinfoMean_list, ptComputed):
        d = N.array([[N.inf for n in range(len(contourinfo_list))] for m in range(len(xyObjects))])
        for m in range(len(xyObjects)):
            for n in range(len(contourinfo_list)):
                try:
                    d[m,n] = N.linalg.norm([xyObjects[m][0]-contourinfo_list[n].x,
                                             xyObjects[m][1]-contourinfo_list[n].y])

                    # Penalize deviations from any known computed positions.
                    #if (ptComputed[m] is None) and (ptComputed[0] is not None):
                    if (ptComputed[m] is not None):
                        dComputed = N.linalg.norm([ptComputed[m].x-contourinfo_list[n].x,
                                                   ptComputed[m].y-contourinfo_list[n].y])
                        #dComputed = N.linalg.norm([ptComputed[m].x-xyObjects[m][0],
                        #                         ptComputed[m].y-xyObjects[m][1]])
                    else: # Use object position as the computed position.
                        dComputed = 0 #d[m,n]
                        
                    d[m,n] += dComputed #* dComputed
                    
                    # Penalize deviations from ideal visual characteristics.
                    #if (contourinfo_list[n].ecc <= contourinfoMin_list[m].ecc) or (contourinfoMax_list[m].ecc <= contourinfo_list[n].ecc): 
                    #    d[m,n] += 1000 
                    #if (contourinfo_list[n].area <= contourinfoMin_list[m].area) or (contourinfoMax_list[m].area <= contourinfo_list[n].area): 
                    #    d[m,n] += 0
                    gainEcc = 10.0
                    eccmetric = gainEcc * N.abs(contourinfo_list[n].ecc - contourinfoMean_list[m].ecc) / N.max([contourinfo_list[n].ecc, contourinfoMean_list[m].ecc]) # Ranges 0 to 1.
                    d[m,n] += 0#eccmetric
                    
                    gainArea = 0.1 
                    areametric = gainArea * N.abs(contourinfo_list[n].area - contourinfoMean_list[m].area)
                    d[m,n] += 0#areametric
                    #if m<2:
                    #    rospy.logwarn('Object %s, eccmetric=%0.2f, areametric=%0.2f' % (m, eccmetric, areametric))
                    
                    
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
    

    # MapContoursFromObjects()
    #   Uses self.contourinfo_list & self.objects,
    #   Returns a list of indices such that self.objects[k] = contour[map[k]], and self.objects[0]=therobot
    #
    def MapContoursFromObjects(self):
        # Make a list of object positions (i.e. robots & flies).
        xyObjects = []

        # Time.
        if (self.stateEndEffector is not None):
            stamp = self.stateEndEffector.header.stamp
        elif (len(self.objects)>0) and (self.objects[0].isVisible):
            stamp = self.objects[0].state.header.stamp
        else:
            stamp = rospy.Time.now()


        # Robots.
        for iRobot in self.iRobot_list:
            if (self.stateEndEffector is not None):
                xyRobotComputed = [self.stateEndEffector.pose.position.x,
                                   self.stateEndEffector.pose.position.y]
                #rospy.logwarn ('CI Robot image at %s' % ([self.objects[0].state.pose.position.x,
                #                                          self.objects[0].state.pose.position.y]))
            elif (iRobot<len(self.objects)) and (self.objects[iRobot].isVisible):
                xyRobotComputed = [self.objects[iRobot].state.pose.position.x,
                                   self.objects[iRobot].state.pose.position.y]
            else:
                xyRobotComputed = [0.0, 0.0]


            xyObjects.append(xyRobotComputed)
            self.tfbx.sendTransform((xyRobotComputed[0], xyRobotComputed[1], 0.0),
                                    tf.transformations.quaternion_about_axis(0, (0,0,1)),
                                    stamp,
                                    "RobotComputed",
                                    "Arena")


        # Flies.    
        for iFly in self.iFly_list:
            #if self.objects[iFly].isVisible:
                xyObjects.append([self.objects[iFly].state.pose.position.x,
                                  self.objects[iFly].state.pose.position.y])
                #rospy.loginfo ('CI Object %s at x,y=%s' % (iFly,[self.objects[iFly].state.pose.position.x,
                #                                                 self.objects[iFly].state.pose.position.y]))
            
        # Make a list of contourinfo positions.
        xyContours = []
        for iContour in range(len(self.contourinfo_list)):
            if self.contourinfo_list[iContour].x is not None:
                xyContours.append([self.contourinfo_list[iContour].x,
                                   self.contourinfo_list[iContour].y])
                #rospy.loginfo ('CI contourinfo %s at x,y=%s' % (iContour,[self.contourinfo_list[iContour].x,
                #                                               self.contourinfo_list[iContour].y]))
        
                
        #rospy.loginfo ('CI flies[0,1].isVisible=%s' % [self.objects[0].isVisible, self.objects[1].isVisible])

        # Construct two lists of contourinfo stats, to describe the range of good robot/fly properties.
        contourinfoMin_list = []
        contourinfoMax_list = []
        contourinfoMean_list = []
        
        # Append the robot contourinfo stats.
        contourinfo = Contourinfo()
        for iRobot in self.iRobot_list:
            contourinfo.area   = self.objects[iRobot].areaMin #self.areaMinRobot
            contourinfo.ecc    = self.objects[iRobot].eccMin #self.eccMinRobot
            contourinfoMin_list.append(contourinfo)
            
            contourinfo.area   = self.objects[iRobot].areaMax #self.areaMaxRobot
            contourinfo.ecc    = self.objects[iRobot].eccMax #self.eccMaxRobot
            contourinfoMax_list.append(contourinfo)
            
            contourinfo.area   = self.objects[iRobot].areaSum / self.objects[iRobot].areaCount 
            contourinfo.ecc    = self.objects[iRobot].eccSum / self.objects[iRobot].eccCount
            contourinfoMean_list.append(contourinfo)
        
        
        # Append the fly contourinfo stats.
        for iFly in self.iFly_list:
            contourinfo.area   = self.objects[iFly].areaMin #self.areaMinFly
            contourinfo.ecc    = self.objects[iFly].eccMin #self.eccMinFly
            contourinfoMin_list.append(contourinfo)
            
            contourinfo.area   = self.objects[iFly].areaMax #self.areaMaxFly
            contourinfo.ecc    = self.objects[iFly].eccMax #self.eccMaxFly
            contourinfoMax_list.append(contourinfo)
            
            contourinfo.area   = self.objects[iFly].areaSum / self.objects[iFly].areaCount 
            contourinfo.ecc    = self.objects[iFly].eccSum / self.objects[iFly].eccCount
            contourinfoMean_list.append(contourinfo)


        # Print ecc/area stats.
        #rospy.logwarn ('robot,fly ecc=[%0.2f, %0.2f], area=[%0.2f, %0.2f]' % (self.objects[0].eccSum/self.objects[0].eccCount,
        #                                                                      self.objects[1].eccSum/self.objects[1].eccCount,
        #                                                                      self.objects[0].areaSum/self.objects[0].areaCount,
        #                                                                      self.objects[1].areaSum/self.objects[1].areaCount))
        
        
        # Create a list of computed object positions, if any.        
        ptComputed = [None for k in self.iAll_list] #[None for k in range(self.nRobots+len(self.objects))]
        for iRobot in self.iRobot_list:
            ptComputed[iRobot] = Point(x=xyRobotComputed[0], y=xyRobotComputed[1])

        # Augment the contourinfo_list list, if necessary, so there are as many contourinfo_list as objects.
        contourinfo_listAug = copy.copy(self.contourinfo_list)
        while len(contourinfo_listAug)<len(xyObjects):
            contourinfo_listAug.append(ContourinfoLists(x=55555, y=55555, ecc=1.0, area=1.0)) # norm(x,y) must be less than 999999!
            #rospy.logwarn ('len(contourinfo_listAug),len(xyObjects)=%s' % [len(contourinfo_listAug),len(xyObjects)])
            
# Do we really need these?  They take time to send.
#        for m in range(len(xyObjects)):
#            if (xyObjects[m][0] is not None) and (xyObjects[m][1] is not None):
#                self.tfbx.sendTransform((xyObjects[m][0], xyObjects[m][1], 0.0),
#                                        (0,0,0,1),
#                                        stamp,
#                                        "xyObjects"+str(m),
#                                        "Arena")
#            
#        for n in range(len(contourinfo_listAug)):
#            self.tfbx.sendTransform((contourinfo_listAug[n].x, contourinfo_listAug[n].y, 0.0),
#                                    (0,0,0,1),
#                                    stamp,
#                                    "contours"+str(n),
#                                    "Arena")
        

        # Match objects with contourinfo_list.
        #rospy.logwarn ('GetDistanceMatrixFromContours()')
        d = self.GetDistanceMatrixFromContours(xyObjects, contourinfo_listAug, contourinfoMin_list, contourinfoMax_list, contourinfoMean_list, ptComputed)
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
                #rospy.logwarn('mapContoursFromObjects[m], len(xyContours)=%s'%[mapContoursFromObjects[m],len(xyContours)])
                if (mapContoursFromObjects[m])>=len(xyContours):
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
#            rospy.logwarn ('CI xyRobotComputed=%s' % xyRobotComputed)

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
                        #rospy.logwarn ('CI robot.position=%s, ptOffset=%s' % ([self.objects[iRobot].state.pose.position.x,
                        #                                                            self.objects[iRobot].state.pose.position.y],
                        #                                                           [self.objects[iRobot].ptOffset.x,
                        #                                                            self.objects[iRobot].ptOffset.y]))
                
                #rospy.logwarn('iFly_list=%s, len(mapContourinfoFromObject)=%d' % (self.iFly_list,len(self.mapContourinfoFromObject)))
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
                
                
                # Publish the EndEffectorOffset.
                if (0 in self.iRobot_list) and (0 < len(self.objects)):
#                     self.pubVisualPosition.publish(PoseStamped(header=self.objects[0].state.header,
#                                                                pose=self.objects[0].state.pose))
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
    
    