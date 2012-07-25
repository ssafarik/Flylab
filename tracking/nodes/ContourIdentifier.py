#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('tracking')
import sys
import copy
import rospy
import tf
import numpy as N
from tracking.msg import ArenaState, Contour, ContourInfo
from plate_tf.srv import PlateCameraConversion
from geometry_msgs.msg import Point, PointStamped, PoseArray, Pose, PoseStamped, Quaternion, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from flycore.msg import MsgFrameState
import filters
from pythonmodules import CircleFunctions
import Fly
from munkres import Munkres


###############################################################################
###############################################################################
###############################################################################
# The class ContourIdentifier subscribes to ContourInfo and EndEffector state, 
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
        self.stateEndEffector = None  # If no robot exists, this will remain as None.  Set in EndEffector callback.
        self.nFlies = rospy.get_param('nFlies', 1)
        self.nRobots = rospy.get_param('nRobots', 1)
        
        self.contours = []
        self.mapContourFromObject = []      # A mapping from the (kalman) object number to the contour number.
        self.iContours = []
        self.objects = []
        self.munkres = Munkres() # Hungarian assignment algorithm.
        
        self.tfrx = tf.TransformListener()
        self.tfbx = tf.TransformBroadcaster()
        
        self.ResetFlyObjects()
        
        
        # Messages
        self.subContourInfo = rospy.Subscriber("ContourInfo", ContourInfo, self.ContourInfo_callback, queue_size=2)
        self.subEndEffector = rospy.Subscriber('EndEffector', MsgFrameState, self.EndEffector_callback, queue_size=2)
        
        self.pubArenaState = rospy.Publisher('ArenaState', ArenaState)
        self.pubEndEffectorOffset = rospy.Publisher('EndEffectorOffset', Point)

        # Poses
        self.poseRobot = Pose()
        self.posearrayFly = PoseArray()
        
        # Points
        self.endeffector_endeffectorframe = PointStamped()
        self.endeffector_endeffectorframe.header.frame_id = "EndEffector"
        self.endeffector_endeffectorframe.point.x = 0
        self.endeffector_endeffectorframe.point.y = 0
        self.endeffector_endeffectorframe.point.z = 0
        

        self.radiusMask = rospy.get_param("camera/mask/radius", 25) # Pixels
        self.radiusArenaInner = rospy.get_param("arena/radius_inner", 25) # Millimeters
        self.radiusArenaOuter = rospy.get_param("arena/radius_outer", 30) # Millimeters

        self.xSave = []
        self.ySave = []
        self.iSave = 0
        self.contouranglePrev = 0.0
        
        self.pubMarker = rospy.Publisher('visualization_marker', Marker)
        self.markerArena = Marker(header=Header(stamp = rospy.Time.now(),
                                                frame_id='/Plate'),
                                  ns='arena',
                                  id=0,
                                  type=3, #CYLINDER,
                                  action=0,
                                  pose=Pose(position=Point(x=0, 
                                                           y=0, 
                                                           z=0)),
                                  scale=Vector3(x=self.radiusArenaOuter*2.0,
                                                y=self.radiusArenaOuter*2.0,
                                                z=0.01),
                                  color=ColorRGBA(a=0.05,
                                                  r=1.0,
                                                  g=1.0,
                                                  b=1.0),
                                  lifetime=rospy.Duration(1.0))


        try:
            rospy.wait_for_service('plate_from_camera') #, timeout=10.0)
            self.plate_from_camera = rospy.ServiceProxy('plate_from_camera', PlateCameraConversion)
        except (rospy.ServiceException, IOError), e:
            print "Service call failed: %s"%e

        # Open a file for saving raw data.
        self.fidRobot = open("/tmp/robot.csv", 'w')
        self.fidRobot.write("xendeffector, yendeffector, xfiltered, yfiltered, xcontour, ycontour\n")

        self.fidFly = open("/tmp/fly.csv", 'w')
        self.fidFly.write("xraw, yraw, xfiltered, yfiltered\n")

        rospy.on_shutdown(self.OnShutdown_callback)

        self.timePrev = rospy.Time.now().to_sec()
        self.initialized = True


    def OnShutdown_callback(self):
        self.fidRobot.close()
        self.fidFly.close()
        

    def EndEffector_callback(self, state):
        #rospy.logwarn('EndEffector_callback:')
        #rospy.logwarn('%s' % state)
        # When first called, need to reset all the fly objects, due to tracking of robot contour as a fly, hence having an extra object.
        if self.stateEndEffector is None:
            self.ResetFlyObjects()
            
            
        #self.stateEndEffector.header.frame_id = "Plate" # Interpret it as the Plate frame.
        
        if True: # Use EndEffector position.
            posesStage = PoseStamped(header=state.header,
                                     pose=state.pose)
            try:
                self.tfrx.waitForTransform('Plate', posesStage.header.frame_id, posesStage.header.stamp, rospy.Duration(1.0))
                posesPlate = self.tfrx.transformPose('Plate', posesStage)
            except tf.Exception, e:
                rospy.logwarn ('Exception in EndEffector_callbackA: %s' % e)
            else:
                self.stateEndEffector = state
                self.stateEndEffector.header = posesPlate.header
                self.stateEndEffector.pose = posesPlate.pose

        else: # Use link5 position
            posesStage = PoseStamped(header=Header(frame_id='Plate'),
                                     pose=Pose(position=Point(x=0, y=0, z=0)))
            try:
                self.tfrx.waitForTransform('link5', posesStage.header.frame_id, posesStage.header.stamp, rospy.Duration(1.0))
                posesPlate = self.tfrx.transformPose('link5',posesStage)
                self.stateEndEffector = state
                self.stateEndEffector.header = posesPlate.header
                self.stateEndEffector.header.frame_id = 'Plate'
                self.stateEndEffector.pose = posesPlate.pose
            except tf.Exception, e:
                rospy.logwarn ('Exception in EndEffector_callbackB: %s' % e)
                
        
        #rospy.loginfo ('CI received state=%s' % state)
        

    def ResetFlyObjects (self):
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
    # Filter the contours by radius.  Return a contourinfo containing only those within radius.
    #  
    def FilterContourinfoWithinRadius(self, contourinfoIn, radius):
        contourinfoOut = ContourInfo()
        if self.initialized:
            contourinfoOut.header = contourinfoIn.header
            contourinfoOut.x = []
            contourinfoOut.y = []
            contourinfoOut.angle = []
            contourinfoOut.area = []
            contourinfoOut.ecc = []
            
            #contourinfoPlate = TransformContourinfoPlateFromCamera(contourinfoIn)
            for iContour in range(len(contourinfoIn.x)):
                if N.linalg.norm(N.array([contourinfoIn.x[iContour],contourinfoIn.y[iContour]])) <= radius:
                    contourinfoOut.x.append(contourinfoIn.x[iContour])
                    contourinfoOut.y.append(contourinfoIn.y[iContour])
                    contourinfoOut.angle.append(contourinfoIn.angle[iContour])
                    contourinfoOut.area.append(contourinfoIn.area[iContour])
                    contourinfoOut.ecc.append(contourinfoIn.ecc[iContour])
                    
        return contourinfoOut
        
        
    # TransformContourinfoPlateFromCamera()
    # Transform the points in contourinfoIn to be in the Plate frame.
    #
    def TransformContourinfoPlateFromCamera(self, contourinfoIn):
        #contourinfoOut = ContourInfo()
        if self.initialized:
            response = self.plate_from_camera(contourinfoIn.x, contourinfoIn.y)
            contourinfoOut = copy.copy(contourinfoIn)
            contourinfoOut.header.frame_id = "Plate"
            contourinfoOut.x = response.Xdst
            contourinfoOut.y = response.Ydst
    
        return contourinfoOut
      

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
    
    
    def GetDistanceMatrixFromContours(self, xyObjects, contours, contoursMin, contoursMax, contoursMean, ptComputed):
        d = N.array([[N.inf for n in range(len(contours))] for m in range(len(xyObjects))])
        for m in range(len(xyObjects)):
            for n in range(len(contours)):
                try:
                    d[m,n] = N.linalg.norm([xyObjects[m][0]-contours[n].x,
                                             xyObjects[m][1]-contours[n].y])

                    # Penalize deviations from any known computed positions.
                    #if (ptComputed[m] is None) and (ptComputed[0] is not None):
                    if (ptComputed[m] is not None):
                        dComputed = N.linalg.norm([ptComputed[m].x-contours[n].x,
                                                   ptComputed[m].y-contours[n].y])
                        #dComputed = N.linalg.norm([ptComputed[m].x-xyObjects[m][0],
                        #                         ptComputed[m].y-xyObjects[m][1]])
                    else: # Use object position as the computed position.
                        dComputed = 0 #d[m,n]
                        
                    d[m,n] += dComputed #* dComputed
                    
                    # Penalize deviations from ideal visual characteristics.
                    #if (contours[n].ecc <= contoursMin[m].ecc) or (contoursMax[m].ecc <= contours[n].ecc): 
                    #    d[m,n] += 1000 
                    #if (contours[n].area <= contoursMin[m].area) or (contoursMax[m].area <= contours[n].area): 
                    #    d[m,n] += 0
                    gainEcc = 10.0
                    eccmetric = gainEcc * N.abs(contours[n].ecc - contoursMean[m].ecc) / N.max([contours[n].ecc, contoursMean[m].ecc]) # Ranges 0 to 1.
                    d[m,n] += eccmetric
                    
                    gainArea = 0.1 
                    areametric = gainArea * N.abs(contours[n].area - contoursMean[m].area)
                    d[m,n] += areametric
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
    #   Uses self.contours & self.objects,
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
                xyRobotComputed = [self.stateEndEffector.pose.position.x + self.objects[iRobot].ptOffset.x,
                                   self.stateEndEffector.pose.position.y + self.objects[iRobot].ptOffset.y]
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
                                    "Plate")


        # Flies.    
        for iFly in self.iFly_list:
            #if self.objects[iFly].isVisible:
                xyObjects.append([self.objects[iFly].state.pose.position.x,
                                  self.objects[iFly].state.pose.position.y])
                #rospy.loginfo ('CI Object %s at x,y=%s' % (iFly,[self.objects[iFly].state.pose.position.x,
                #                                                 self.objects[iFly].state.pose.position.y]))
            
        # Make a list of contour positions.
        xyContours = []
        for iContour in range(len(self.contours)):
            if self.contours[iContour].x is not None:
                xyContours.append([self.contours[iContour].x,
                                   self.contours[iContour].y])
                #rospy.loginfo ('CI contour %s at x,y=%s' % (iContour,[self.contours[iContour].x,
                #                                               self.contours[iContour].y]))
        
                
        #rospy.loginfo ('CI flies[0,1].isVisible=%s' % [self.objects[0].isVisible, self.objects[1].isVisible])

        # Construct two lists of contour stats, to describe the range of good robot/fly properties.
        contoursMin = []
        contoursMax = []
        contoursMean = []
        
        # Append the robot contour stats.
        contour = Contour()
        for iRobot in self.iRobot_list:
            contour.area   = self.objects[iRobot].areaMin #self.areaMinRobot
            contour.ecc    = self.objects[iRobot].eccMin #self.eccMinRobot
            contoursMin.append(contour)
            
            contour.area   = self.objects[iRobot].areaMax #self.areaMaxRobot
            contour.ecc    = self.objects[iRobot].eccMax #self.eccMaxRobot
            contoursMax.append(contour)
            
            contour.area   = self.objects[iRobot].areaSum / self.objects[iRobot].areaCount 
            contour.ecc    = self.objects[iRobot].eccSum / self.objects[iRobot].eccCount
            contoursMean.append(contour)
        
        
        # Append the fly contour stats.
        for iFly in self.iFly_list:
            contour.area   = self.objects[iFly].areaMin #self.areaMinFly
            contour.ecc    = self.objects[iFly].eccMin #self.eccMinFly
            contoursMin.append(contour)
            
            contour.area   = self.objects[iFly].areaMax #self.areaMaxFly
            contour.ecc    = self.objects[iFly].eccMax #self.eccMaxFly
            contoursMax.append(contour)
            
            contour.area   = self.objects[iFly].areaSum / self.objects[iFly].areaCount 
            contour.ecc    = self.objects[iFly].eccSum / self.objects[iFly].eccCount
            contoursMean.append(contour)


        # Print ecc/area stats.
        #rospy.logwarn ('robot,fly ecc=[%0.2f, %0.2f], area=[%0.2f, %0.2f]' % (self.objects[0].eccSum/self.objects[0].eccCount,
        #                                                                      self.objects[1].eccSum/self.objects[1].eccCount,
        #                                                                      self.objects[0].areaSum/self.objects[0].areaCount,
        #                                                                      self.objects[1].areaSum/self.objects[1].areaCount))
        
        
        # Create a list of computed object positions, if any.        
        ptComputed = [None for k in self.iAll_list] #[None for k in range(self.nRobots+len(self.objects))]
        for iRobot in self.iRobot_list:
            ptComputed[iRobot] = Point(x=xyRobotComputed[0], y=xyRobotComputed[1])

        # Augment the contours list, if necessary, so there are as many contours as objects.
        contoursAug = copy.copy(self.contours)
        while len(contoursAug)<len(xyObjects):
            contoursAug.append(ContourInfo(x=55555, y=55555, ecc=1.0, area=1.0)) # norm(x,y) must be less than 999999!
            #rospy.logwarn ('len(contoursAug),len(xyObjects)=%s' % [len(contoursAug),len(xyObjects)])
            
        # Match objects with contours.
        #rospy.logwarn ('GetDistanceMatrixFromContours()')
        for m in range(len(xyObjects)):
            self.tfbx.sendTransform((xyObjects[m][0], xyObjects[m][1], 0.0),
                                    (0,0,0,1),
                                    stamp,
                                    "xyObjects"+str(m),
                                    "Plate")
            

        for n in range(len(contoursAug)):
            self.tfbx.sendTransform((contoursAug[n].x, contoursAug[n].y, 0.0),
                                    (0,0,0,1),
                                    stamp,
                                    "contours"+str(n),
                                    "Plate")
        
        d = self.GetDistanceMatrixFromContours(xyObjects, contoursAug, contoursMin, contoursMax, contoursMean, ptComputed)
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
    

    def ContourInfo_callback(self, contourinfo):
#        rospy.logwarn('ContourInfo_callback(now-prev=%s)' % (rospy.Time.now().to_sec()-self.timePrev))
#        self.timePrev = rospy.Time.now().to_sec()

        if self.initialized:
            try:
                #rospy.logwarn ('CI contourinfo0 %s' % contourinfo)
                contourinfo = self.TransformContourinfoPlateFromCamera(contourinfo)
                #rospy.logwarn ('CI contourinfo1 %s' % contourinfo)
                contourinfo = self.FilterContourinfoWithinRadius(contourinfo, self.radiusMask)
                #rospy.logwarn ('CI contourinfo2 %s' % contourinfo)
    
                # Create a null contour.
                contourNone = Contour()
                contourNone.header = contourinfo.header
                contourNone.x = None
                contourNone.y = None
                contourNone.angle = None
                contourNone.area = None
                contourNone.ecc = None

                # Repackage the contourinfo into a list of contours
                self.contours = []            
                for i in range(len(contourinfo.x)):
                    contour = Contour()
                    contour.header = contourinfo.header
                    contour.x      = contourinfo.x[i]
                    contour.y      = contourinfo.y[i]
                    if not N.isnan(contourinfo.angle[i]):
                        contour.angle = contourinfo.angle[i]
                    else:
                        contour.angle = self.contouranglePrev
                    self.contouranglePrev = contour.angle
                    
                    contour.area   = contourinfo.area[i]
                    contour.ecc    = contourinfo.ecc[i]
                    self.contours.append(contour)
                    #rospy.logwarn('contour.angle=%0.2f' % (contour.angle))
    
                    # Send the contour transforms.
#                    try:
#                        self.tfbx.sendTransform((contour.x, contour.y, 0.0),
#                                                tf.transformations.quaternion_about_axis(contour.angle, (0,0,1)),
#                                                contour.header.stamp,
#                                                "contour"+str(i),
#                                                "ImageRect")
#                    except tf.Exception, e:
#                        rospy.logwarn ('Exception in sendTransform(%s->%s): %s' % ("contour"+str(i),"ImageRect",e))
                    
    
                # Figure out who is who in the camera image.
                try:
                    self.mapContourFromObject = self.MapContoursFromObjects()
                except IndexError:
                    self.mapContourFromObject = None
                    
                #rospy.logwarn ('CI map=%s' % self.mapContourFromObject)
                #for i in range(len(self.contours)):
                #    rospy.logwarn ('CI contour[%d].x,y=%s' % (i,[self.contours[i].x,self.contours[i].y]))
                
                # Update the robot state w/ the contour and end-effector positions.
                if self.mapContourFromObject is not None:
                    for iRobot in self.iRobot_list:
                        if (self.stateEndEffector is not None):
                            if self.mapContourFromObject[iRobot] is not None:
                                # For the robot, use the end-effector angle instead of the contour angle.
                                if self.stateEndEffector is not None:
                                    q = self.stateEndEffector.pose.orientation
                                    rpy = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
                                    self.contours[self.mapContourFromObject[iRobot]].angle = rpy[2]
    
                                contour = self.contours[self.mapContourFromObject[iRobot]]
                            else:
                                contour = contourNone
                                 
                            self.objects[iRobot].Update(contour, self.stateEndEffector.pose.position)
                            
                            # Write a file (for getting Kalman covariances, etc).
                            #data = '%s, %s, %s, %s, %s, %s\n' % (self.stateEndEffector.pose.position.x,
                            #                                     self.stateEndEffector.pose.position.y,
                            #                                     self.objects[iRobot].state.pose.position.x, 
                            #                                     self.objects[iRobot].state.pose.position.y,
                            #                                     self.contours[self.mapContourFromObject[iRobot]].x,
                            #                                     self.contours[self.mapContourFromObject[iRobot]].y)
                            #self.fidRobot.write(data)
                            
                            #rospy.loginfo ('CI update robot    contour=%s' % contour)
                        
#                    rospy.logwarn('contourinfo.angle[]=%s' % contourinfo.angle)
#                    rospy.logwarn('map=%s' % self.mapContourFromObject)
                    
                    # Update the flies' states.
                    for iFly in self.iFly_list:
                        if self.mapContourFromObject[iFly] is not None:
                            contour = self.contours[self.mapContourFromObject[iFly]]
                        else:
                            contour = contourNone
                        
                        self.objects[iFly].Update(contour, None)
        
                        #self.stateEndEffector.header.stamp,#rospy.Time.now()
                        #rospy.loginfo ('CI update state %s contour=%s' % (iFly,contour))
        
                        # Write a file.
                        #if self.mapContourFromObject[1] is not None:
                        #    data = '%s, %s, %s, %s\n' % (self.contours[self.mapContourFromObject[1]].x, 
                        #                                 self.contours[self.mapContourFromObject[1]].y, 
                        #                                 self.objects[1].state.pose.position.x, 
                        #                                 self.objects[1].state.pose.position.y)
                        #    self.fidFly.write(data)
                    
        
        
                    # Construct the ArenaState message.
                    arenastate = ArenaState()
                    #if self.objects[0].state.pose.position.x is not None:
                    for iRobot in self.iRobot_list:
                        arenastate.robot.header.stamp    = self.objects[iRobot].state.header.stamp
                        arenastate.robot.header.frame_id = self.objects[iRobot].state.header.frame_id
                        arenastate.robot.pose            = self.objects[iRobot].state.pose
                        arenastate.robot.velocity        = self.objects[iRobot].state.velocity
                        #rospy.logwarn ('CI robot.position=%s, ptOffset=%s' % ([self.objects[iRobot].state.pose.position.x,
                        #                                                            self.objects[iRobot].state.pose.position.y],
                        #                                                           [self.objects[iRobot].ptOffset.x,
                        #                                                            self.objects[iRobot].ptOffset.y]))
                    
		    #rospy.logwarn('iFly_list=%s, len(mapContourFromObject)=%d' % (self.iFly_list,len(self.mapContourFromObject)))
                    for iFly in self.iFly_list:
                        #rospy.logwarn ('iFly=%d, self.mapContourFromObject=%s, len(self.objects)=%d' % (iFly, self.mapContourFromObject, len(self.objects)))
                        if (self.mapContourFromObject[iFly] is not None) and (self.objects[iFly].state.pose.position.x is not None):
                            arenastate.flies.append(MsgFrameState(header = self.objects[iFly].state.header, 
                                                                  pose = self.objects[iFly].state.pose,
                                                                  velocity = self.objects[iFly].state.velocity))
                            #rospy.logwarn('arenastate.flies.append(%s)' % self.objects[iFly].name)

                    
                    # Publish the ArenaState.
                    self.pubArenaState.publish(arenastate)
                    
                    
                    # Publish the EndEffectorOffset.
                    if 0 in self.iRobot_list:
                        self.pubEndEffectorOffset.publish(self.objects[0].ptOffset)
                    
                    
                    # Publish a disc to indicate the arena extent.
                    self.markerArena.header.stamp = contourinfo.header.stamp
                    self.pubMarker.publish(self.markerArena)
            except rospy.ServiceException, e:
                rospy.logwarn ('Exception in contourinfo_callback(): %s' % e)


if __name__ == '__main__':
    rospy.init_node('ContourIdentifier')
    ci = ContourIdentifier()

    #try:
    rospy.spin()
    #except:
    #    rospy.loginfo("Shutting down")

    #cv.DestroyAllWindows()

