#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('tracking')

import sys
import rospy
import tf
import numpy as N
from tracking.msg import *
from plate_tf.srv import *
from geometry_msgs.msg import Point, PointStamped, PoseArray, Pose, PoseStamped, Quaternion, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from flycore.msg import *
import copy
import filters
from pythonmodules import CircleFunctions
#import MatchHungarian

#class Contour:
#    #None
#    x = None
#    y = None
#    angle = None
#    area = None
#    ecc = None

    #def __init(self):
    #    self.x = None
    #    self.y = None
    #    self.angle = None
    #    self.area = None
    #    self.ecc = None
        
        
###############################################################################
###############################################################################
###############################################################################
def PolarFromXy(x, y):
    mag = N.sqrt(x**2 + y**2)
    ang = N.arctan2(y, x)
    return (mag, ang)


def YawFromQuaternion(q):
    rpy = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
    return rpy[2]


# FlipQuaternion()
# Cheesy way to flip quaternion, knowing it is a vector at origin in xy-plane
#
def FlipQuaternion(q):

    angle = YawFromQuaternion(q)
    # Add pi to angle and compute new quaternion
    qz = tf.transformations.quaternion_about_axis((angle + N.pi), (0,0,1))
    return qz


# Unwrap()
#   Put the angle into the range of -pi to +pi.
#
def Unwrap(self, angle):
    angleOut = ((angle+N.pi) % (2.0*N.pi)) - N.pi
    
    #while N.pi < angleOut:
    #    angleOut -= (2.0*N.pi)
    #    
    #while angleOut < -N.pi:
    #    angleOut += (2.0*N.pi)
    
    return angleOut
    
    

###############################################################################
###############################################################################
###############################################################################
# class Fly()
# Implements a fly (or robot) object in a 2D arena. 
# Updates its state based on an image "contour" from a single camera.
#
class Fly:
    def __init__(self, name=None):
        self.initialized = False
        self.name = name

        self.tfbx = tf.TransformBroadcaster()
        self.pubMarker = rospy.Publisher('visualization_marker', Marker)
        

        self.kfState = filters.KalmanFilter()
        self.lpAngleF = filters.LowPassAngleFilter(RC=0.001)
        self.lpOffsetX = filters.LowPassFilter(RC=0.1)
        self.lpOffsetY = filters.LowPassFilter(RC=0.1)
        self.ptOffset = Point(x=0, y=0, z=0)  # Difference between contour position and (for robots) computed position.
        self.ptPPrev = Point(x=0, y=0, z=0)
        
        # Orientation detection stuff.
        self.angleOfTravelRecent = None
        self.flip = False
        self.lpFlip = filters.LowPassFilter(RC=3.0)
        self.contour = None
        
        self.isVisible = False
        self.isDead = False # To do: dead fly detection.

        self.timePrevious = None
        
        self.state = MsgFrameState()
        self.state.header.frame_id = 'Plate'
        self.state.pose.position.x = 0.0
        self.state.pose.position.y = 0.0
        self.state.pose.position.z = 0.0
        q = tf.transformations.quaternion_about_axis(0.0, (0,0,1))
        self.state.pose.orientation.x = q[0]    # Note: The orientation is ambiguous as to head/tail.  
        self.state.pose.orientation.y = q[1]    #       Check the self.flip status to resolve it.
        self.state.pose.orientation.z = q[2]    #
        self.state.pose.orientation.w = q[3]    #
        self.state.velocity.linear.x = 0.0
        self.state.velocity.linear.y = 0.0
        self.state.velocity.linear.z = 0.0
        self.state.velocity.angular.x = 0.0
        self.state.velocity.angular.y = 0.0
        self.state.velocity.angular.z = 0.0
        
        self.eccMin = 999
        self.eccMax = 0
        self.eccSum = 0
        self.eccCount = 1
        self.areaMin = 999
        self.areaMax = 0
        self.areaSum = 0
        self.areaCount = 1
        self.robot_width = rospy.get_param ('robot_width', 1.0)
        self.robot_height = rospy.get_param ('robot_height', 1.0)
        
        self.initialized = True


    # GetResolvedQuaternion()
    #   Using self.flip and self.contour.angle, return a flipped & filtered quaternion.
    #
    def GetResolvedQuaternion(self):
        if self.flip:
            angle = self.contour.angle + N.pi
        else:
            angle = self.contour.angle
            
        angleResolved = self.lpAngleF.Update(angle, rospy.Time.now().to_sec())
        qResolved = Quaternion()
        (qResolved.x, qResolved.y, qResolved.z, qResolved.w) = tf.transformations.quaternion_about_axis(angleResolved, (0,0,1))

        #if "Fly" in self.name:
        #    rospy.logwarn ('CI contour.angle=%+0.3f, self.flip=%s, angle=%+0.3f, angleResolved=%+0.3f' % (self.contour.angle, self.flip, angle, angleResolved))

        return qResolved


        
    # ResolveOrientation()
    #   Using self.contour, self.state, and self.angleOfTravelRecent,
    #   Choose among the various possibilities:
    #   angleResolved = (angle of contour)
    #     OR
    #   angleResolved = (angle of contour + pi)
    #
    # Updates the flip status in self,
    # and updates self.state.pose.orientation
    #   
    def ResolveOrientation(self):
        # angleContour is ambiguous mod pi radians
        eccmetric = (self.contour.ecc + 1/self.contour.ecc) - 1
        if eccmetric > 1.5:
            angleContour         = self.contour.angle #YawFromQuaternion(self.state.pose.orientation)
            angleContour_flipped = angleContour + N.pi
            
            # Most recent angle of travel.
            if self.angleOfTravelRecent is not None:
                angleOfTravel = self.angleOfTravelRecent
            else:
                angleOfTravel = angleContour
                
                
            flipValueLast = self.lpFlip.GetValue()
            if flipValueLast is None:
                flipValueLast = 0.0
            
            # Compare distances between angles of (travel - orientation) and (travel - flippedorientation)        
            dist         = CircleFunctions.circle_dist(angleContour,         angleOfTravel)
            dist_flipped = CircleFunctions.circle_dist(angleContour_flipped, angleOfTravel)
            speed = N.linalg.norm([self.state.velocity.linear.x, self.state.velocity.linear.y])
            
            # Vote for flipped or non-flipped.
            if dist < dist_flipped:
                flipValueNew = -1.0 # Not flipped
            else:
                flipValueNew =  1.0 # Flipped

            # Weight the prev/new values based on speed.                
            flipValuePre = (10/(10+speed) * flipValueLast) + ((1-10/(10+speed)) * flipValueNew) 

            # Update the filter, and get the flip state.
            flipValuePost = self.lpFlip.Update(flipValuePre, rospy.Time.now().to_sec())
            if flipValuePost > 0.0:
                flipNew = True
            else:
                flipNew = False
                    
            if speed > 3.0: # Deadband.
                self.flip = flipNew    
                
            self.state.pose.orientation = self.GetResolvedQuaternion()

                            

    # Update()
    # Update the current state using the visual position and the computed position (if applicable)
    def Update(self, contour, ptComputed):
        #rospy.loginfo ('CI contour=%s' % (contour))
        if self.initialized:
            t = rospy.Time.now().to_sec()
            contourPrev = self.contour
            self.contour = contour
            angleContourF = None
            angleContour = None
            
            # Update the position & orientation filters
            self.isVisible = False            
            if (self.contour is not None):
                # Update min/max ecc & area.
                if contour.ecc < self.eccMin:
                    self.eccMin = contour.ecc
                if self.eccMax < contour.ecc:
                    self.eccMax = contour.ecc
                if contour.area < self.areaMin:
                    self.areaMin = contour.area
                if self.areaMax < contour.area:
                    self.areaMax = contour.area
                self.eccSum += contour.ecc
                self.eccCount += 1
                self.areaSum += contour.area
                self.areaCount += 1
                
                if (self.contour.x is not None) and (self.contour.y is not None) and (self.contour.angle is not None):
                    self.isVisible = True
                    #rospy.loginfo ('CI kfState.state_post=%s' % self.kfState.state_post) 
                    (x,y,vx,vy) = self.kfState.Update((self.contour.x, self.contour.y), t)
                    #(x,y) = (self.contour.x,self.contour.y) # Unfiltered.
                    if N.abs(self.contour.x) > 9999 or N.abs(x)>9999:
                        rospy.logwarn ('CI LARGE CONTOUR.x=%s' % self.contour.x)
                        rospy.logwarn ('CI LARGE         x=%s' % x)
                        rospy.logwarn ('CI LARGE         t=%s' % t)

                    # If there's a big jump in contour angle, then change the flip state.
                    anglePrev = self.lpAngleF.GetValue()
                    if contourPrev is not None:
                        d = CircleFunctions.circle_dist(contourPrev.angle, self.contour.angle)
                        if (d > (N.pi/2)):
                            flipPrev = self.lpFlip.GetValue()
                            if flipPrev is not None:
                                self.lpFlip.SetValue(-flipPrev)
                                self.flip = not self.flip
                        
                    #angleContourF = self.lpAngleF.Update(self.contour.angle, t)
                    angleContour = self.contour.angle
                    #rospy.loginfo ('CI kfState.Update name=%s pre=%s, post=%s, t=%s' % (self.name, [contour.x,contour.y], [x,y], t))
    
                # Use the unfiltered data for the case where the filters return None results.
                if self.isVisible and ((x is None) or (y is None) or (angleContour is None)):
                    x = self.contour.x
                    y = self.contour.y
                    vx = 0.0
                    vy = 0.0
                    #angleContourF = self.contour.angle
                    angleContour = self.contour.angle
                    rospy.logwarn('CI KFILTER RETURNED NONE on %s, %s' % ((self.contour.x, self.contour.y), t))

            else: # self.contour is None
                (x,y,vx,vy) = self.kfState.Update(None, t)
                #self.lpAngleF.Update(None, t)
                #self.lpOffsetX.Update(None,t)
                #self.lpOffsetY.Update(None,t)
                #self.lpFlip.Update(0.0, t)
                angleContour = 0.0
                #rospy.logwarn('CI FILTER READ for %s w/ contour==None' % self.name)
                
                
            if self.isVisible:
                # Store the latest state.
                #if "Fly" in self.name:
                #    rospy.logwarn ('CI angleContour=%s' % angleContour)
                self.state.header.stamp = contour.header.stamp
                self.state.pose.position.x = x
                self.state.pose.position.y = y
                self.state.pose.position.z = 0.0
                #q = tf.transformations.quaternion_about_axis(angleContour, (0,0,1))
                #self.state.pose.orientation.x = q[0]
                #self.state.pose.orientation.y = q[1]
                #self.state.pose.orientation.z = q[2]
                #self.state.pose.orientation.w = q[3]
                self.state.velocity.linear.x = vx
                self.state.velocity.linear.y = vy
                self.state.velocity.linear.z = 0.0
                self.state.velocity.angular.x = 0.0
                self.state.velocity.angular.y = 0.0
                self.state.velocity.angular.z = 0.0

                # Update the most recent angle of travel.
                angleOfTravel = N.arctan2(self.state.velocity.linear.y, self.state.velocity.linear.x)
                speed = N.linalg.norm([self.state.velocity.linear.x, self.state.velocity.linear.y])
                if speed>5.0:
                    self.angleOfTravelRecent = angleOfTravel
                
                self.ResolveOrientation()
                
                #rospy.logwarn ('CI %s ecc=%s,%s, area=%s,%s, speed=%0.3f' % (self.name, contour.ecc,[self.eccMin,self.eccMax], contour.area,[self.areaMin,self.areaMax], speed))
                
                # Update the tool offset.
                if ptComputed is not None:

                    # PID control of the offset.
                    ptP = Point()
                    ptI = Point()
                    ptD = Point()
                    ptP.x = x-ptComputed.x
                    ptP.y = y-ptComputed.y
                    ptI.x = ptI.x + ptP.x
                    ptI.y = ptI.y + ptP.y
                    ptD.x = ptP.x - self.ptPPrev.x
                    ptD.y = ptP.y - self.ptPPrev.y
                    kP = rospy.get_param('tooloffset/kP', 1.0)
                    kI = rospy.get_param('tooloffset/kI', 0.0)
                    kD = rospy.get_param('tooloffset/kD', 0.0)
                    xPID = kP*ptP.x + kI*ptI.x + kD*ptD.x
                    yPID = kP*ptP.y + kI*ptI.y + kD*ptD.y
                    
                    # Filtered offset.
                    self.ptOffset = Point(x = self.lpOffsetX.Update(xPID, t),
                                          y = self.lpOffsetY.Update(yPID, t),
                                          z = 0)
                    self.ptPPrev.x = ptP.x
                    self.ptPPrev.y = ptP.y
                else:
                    self.ptOffset = Point(x=0, y=0, z=0)
            

                #rospy.logwarn ('x=%s,y=%s,orientation=%s,name=%s,frame_id=%s' % (self.state.pose.position.x, 
                #                         self.state.pose.position.y,
                #                        self.state.pose.orientation,
                #                        self.name,
                #                        self.state.header.frame_id))
                
                # Send the Raw transform.
                self.tfbx.sendTransform((self.contour.x, 
                                         self.contour.y, 
                                         0.0),
                                        tf.transformations.quaternion_about_axis(self.contour.angle, (0,0,1)),
                                        self.contour.header.stamp,
                                        self.name+"Contour",
                                        "Plate")
                
                # Send the Filtered transform.
                q = self.state.pose.orientation
                self.tfbx.sendTransform((self.state.pose.position.x, 
                                         self.state.pose.position.y, 
                                         self.state.pose.position.z),
                                        (q.x, q.y, q.z, q.w),
                                        rospy.Time.now(),
                                        self.name,
                                        self.state.header.frame_id)

                if 'Robot' in self.name:
                    markerTarget = Marker(header=Header(stamp = rospy.Time.now(),
                                                        frame_id='Plate'),
                                          ns='robot',
                                          id=0,
                                          type=3, #cylinder,
                                          action=0,
                                          pose=Pose(position=Point(x=self.state.pose.position.x, 
                                                                   y=self.state.pose.position.y, 
                                                                   z=self.state.pose.position.z)),
                                          scale=Vector3(x=self.robot_width,
                                                        y=self.robot_width,
                                                        z=self.robot_height),
                                          color=ColorRGBA(a=0.7,
                                                          r=0.5,
                                                          g=0.5,
                                                          b=0.5),
                                          lifetime=rospy.Duration(0.1))
                    self.pubMarker.publish(markerTarget)

        #rospy.loginfo ('CI %s contour=%s, self.isVisible=%s' % (self.name, contour, self.isVisible))
            
        

###############################################################################
###############################################################################
###############################################################################
class ContourIdentifier:

    def __init__(self):
        self.initialized = False
        self.maxFlies = rospy.get_param('maxFlies', 1)
        
        self.contours = []
        self.map = []
        self.iContours = []
        
        self.CreateFlyObjects()
        
        #if self.maxFlies>0:
        #    self.objects[1].name = "Fly"
            
        
        # Messages
        self.subContourInfo = rospy.Subscriber("ContourInfo", ContourInfo, self.ContourInfo_callback)
        self.subEndEffector = rospy.Subscriber('EndEffector', MsgFrameState, self.EndEffector_callback)
        
        self.pubArenaState = rospy.Publisher('ArenaState', ArenaState)
        self.pubEndEffectorOffset = rospy.Publisher('EndEffectorOffset', Point)

        self.tfrx = tf.TransformListener()
        self.tfbx = tf.TransformBroadcaster()
        
        # Poses
        self.stateEndEffector = None  # If no robot exists, this will remain as None.  Set in EndEffector callback.
        self.poseRobot = Pose()
        self.posearrayFly = PoseArray()
        
        # Points
        self.endeffector_endeffectorframe = PointStamped()
        self.endeffector_endeffectorframe.header.frame_id = "EndEffector"
        self.endeffector_endeffectorframe.point.x = 0
        self.endeffector_endeffectorframe.point.y = 0
        self.endeffector_endeffectorframe.point.z = 0
        

        self.radiusArena = rospy.get_param("arena/radius_camera",100) # Pixels

        self.xSave = []
        self.ySave = []
        self.iSave = 0

        self.pubMarker = rospy.Publisher('visualization_marker', Marker)
        self.markerArena = Marker(header=Header(stamp = rospy.Time.now(),
                                                frame_id='/Plate'),
                                  ns='arena',
                                  id=2,
                                  type=3, #CYLINDER,
                                  action=0,
                                  pose=Pose(position=Point(x=0, 
                                                           y=0, 
                                                           z=0)),
                                  scale=Vector3(x=self.radiusArena*2.0 * 85/470, #BUG: Need to properly convert from pixels to mm.
                                                y=self.radiusArena*2.0 * 85/470,
                                                z=0.01),
                                  color=ColorRGBA(a=0.05,
                                                  r=1.0,
                                                  g=1.0,
                                                  b=1.0),
                                  lifetime=rospy.Duration(0.1))


        try:
            rospy.wait_for_service('camera_to_plate') #, timeout=10.0)
            self.camera_to_plate = rospy.ServiceProxy('camera_to_plate', PlateCameraConversion)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # Open a file for saving raw data.
        self.fidRobot = open("/home/ssafarik/robot.csv", 'w')
        self.fidRobot.write("xendeffector, yendeffector, xfiltered, yfiltered, xcontour, ycontour\n")

        self.fidFly = open("/home/ssafarik/fly.csv", 'w')
        self.fidFly.write("xraw, yraw, xfiltered, yfiltered\n")

        rospy.on_shutdown(self.OnShutdown_callback)


        self.initialized = True


    def OnShutdown_callback(self):
        self.fidRobot.close()
        self.fidFly.close()
        

    def EndEffector_callback(self, state):
        # When first called, need to reset all the fly objects, due to tracking of robot contour as a fly, hence having an extra object.
        if self.stateEndEffector is None:
            self.CreateFlyObjects()
            
            
        #self.stateEndEffector.header.frame_id = "Plate" # Interpret it as the Plate frame.
        
        if True: # Use EndEffector position from Fivebar
            posesStage = PoseStamped(header=Header(frame_id=state.header.frame_id),
                                     pose=state.pose)
            try:
                posesPlate = self.tfrx.transformPose('Plate',posesStage)
                self.stateEndEffector = state
                self.stateEndEffector.header = posesPlate.header
                self.stateEndEffector.pose = posesPlate.pose
            except tf.Exception:
                pass
        else: # Use link5 position
            posesStage = PoseStamped(header=Header(frame_id='Plate'),
                                     pose=Pose(position=Point(x=0, y=0, z=0)))
            try:
                posesPlate = self.tfrx.transformPose('link5',posesStage)
                self.stateEndEffector = state
                self.stateEndEffector.header = posesPlate.header
                self.stateEndEffector.header.frame_id = 'Plate'
                self.stateEndEffector.pose = posesPlate.pose
            except tf.Exception:
                pass
                
        
        #rospy.loginfo ('CI received state=%s' % state)
        

    def CreateFlyObjects (self):
        #for i in range(len(self.objects)):
        #    del self.objects[i]
            
        self.objects = []
        for i in range(1+self.maxFlies):
            self.objects.append(Fly())
            self.objects[i].name = "Fly%s" % i
        
        self.objects[0].name = "Robot"
        
        
    def QuaternionPlateFromCamera(self, quat):
        # Must be cleverer way to calculate this using quaternion math...
        R = tf.transformations.quaternion_matrix(quat)
        scale_factor = 100
        points_camera = N.array(\
            [[0,  1, -1,  0,  0,  1, -1],
             [0,  0,  0,  1, -1,  1, -1],
             [0,  0,  0,  0,  0,  0,  0],
             [1,  1,  1,  1,  1,  1,  1]])
        points_camera = points_camera*scale_factor
        # rospy.logwarn("points_camera = \n%s", str(points_camera))
        points_camera_rotated = N.dot(R,points_camera)
        # rospy.logwarn("points_camera_rotated = \n%s", str(points_camera_rotated))
        try:
            xSrc = list(points_camera[0,:])
            ySrc = list(points_camera[1,:])
            # rospy.logwarn("xSrc = %s", str(xSrc))
            # rospy.logwarn("ySrc = %s", str(ySrc))
            response = self.camera_to_plate(xSrc,ySrc)
            points_plate_x = list(response.Xdst)
            points_plate_y = list(response.Ydst)
            z = [0]*len(points_plate_x)
            w = [1]*len(points_plate_y)
            points_plate = N.array([points_plate_x,points_plate_y,z,w])
            points_plate = N.append(points_plate,[[0,0],[0,0],[scale_factor,-scale_factor],[1,1]],axis=1)
            # rospy.logwarn("points_plate = \n%s", str(points_plate))

            xSrc = list(points_camera_rotated[0,:])
            ySrc = list(points_camera_rotated[1,:])
            response = self.camera_to_plate(xSrc,ySrc)
            points_plate_rotated_x = list(response.Xdst)
            points_plate_rotated_y = list(response.Ydst)
            # rospy.logwarn("points_plate_rotated_x = %s",str(points_plate_rotated_x))
            # rospy.logwarn("points_plate_rotated_y = %s",str(points_plate_rotated_y))
            # rospy.loginfo("z = %s",str(z))
            # rospy.loginfo("w = %s",str(w))
            points_plate_rotated = N.array([points_plate_rotated_x,points_plate_rotated_y,z,w])
            points_plate_rotated = N.append(points_plate_rotated,[[0,0],[0,0],[scale_factor,-scale_factor],[1,1]],axis=1)
            # rospy.loginfo("points_plate_rotated = \n%s", str(points_plate_rotated))
            T = tf.transformations.superimposition_matrix(points_plate_rotated,points_plate)
            # rospy.loginfo("T = \n%s", str(T))
            # al, be, ga = tf.transformations.euler_from_matrix(T, 'rxyz')
            # rospy.loginfo("ga = %s" % str(ga*180/N.pi))
            quat_plate = tf.transformations.quaternion_from_matrix(T)
            return quat_plate
        # except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException):
        except (tf.LookupException, tf.ConnectivityException, tf.Exception, rospy.ServiceException, AttributeError, ValueError):
            return None



    # FilterContourinfoWithinRadius()
    # Filter the contours by radius.  Return a contourinfo containing only those within radius.
    #  
    def FilterContourinfoWithinRadius(self, contourinfoIn, radius):
        contourinfoOut = ContourInfo()
        if self.initialized:
            contourinfoOut.header = contourinfoIn.header
            contourinfoOut.x = []
            contourinfoOut.y = []
            contourinfoOut.theta = []
            contourinfoOut.area = []
            contourinfoOut.ecc = []
            
            #contourinfoPlate = TransformContourinfoPlateFromCamera(contourinfoIn)
            for iContour in range(len(contourinfoIn.x)):
                if N.linalg.norm(N.array([contourinfoIn.x[iContour],contourinfoIn.y[iContour]])) <= radius:
                    contourinfoOut.x.append(contourinfoIn.x[iContour])
                    contourinfoOut.y.append(contourinfoIn.y[iContour])
                    contourinfoOut.theta.append(contourinfoIn.theta[iContour])
                    contourinfoOut.area.append(contourinfoIn.area[iContour])
                    contourinfoOut.ecc.append(contourinfoIn.ecc[iContour])
                    
        return contourinfoOut
        
        
    # TransformContourinfoPlateFromCamera()
    # Transform the points in contourinfoIn to be in the Plate frame.
    #
    def TransformContourinfoPlateFromCamera(self, contourinfoIn):
        contourinfoOut = ContourInfo()
        if self.initialized:
            response = self.camera_to_plate(contourinfoIn.x, contourinfoIn.y)
            contourinfoOut = contourinfoIn
            contourinfoOut.header.frame_id = "Plate"
            contourinfoOut.x = response.Xdst
            contourinfoOut.y = response.Ydst
    
        return contourinfoOut
      

    # GetDistanceMatrix()
    # Get the matrix of distances between each pair of points in the two lists.  Adjust distances with
    # priorities, where smaller=better.
    #
    def GetDistanceMatrix(self, xy1, xy2, iPriorities):
        d = N.array([[N.inf for n in range(len(xy2))] for m in range(len(xy1))])
        for m in range(len(xy1)):
            for n in range(len(xy2)):
                try:
                    d[m,n] = N.linalg.norm([xy1[m][0]-xy2[n][0],
                                             xy1[m][1]-xy2[n][1]])
                    
                    # Quick & dirty priority calc.
                    d[m,n] += iPriorities[m]

                except TypeError:
                    d[m,n] = None
        
        #rospy.loginfo('CI DM xy1=%s' % xy1)
        #rospy.loginfo('CI DM xy2=%s' % xy2)
        #rospy.loginfo('CI DM d=%s' % d)

        return d
    
    
    def GetDistanceMatrixFromContours(self, xyObjects, contours, contoursMin, contoursMax, contoursMean):
        d = N.array([[N.inf for n in range(len(contours))] for m in range(len(xyObjects))])
        for m in range(len(xyObjects)):
            for n in range(len(contours)):
                try:
                    d[m,n] = N.linalg.norm([xyObjects[m][0]-contours[n].x,
                                             xyObjects[m][1]-contours[n].y])
                    
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
    
    
    # GetStableMatching()
    #   Solve the "Stable Marriage Problem" for two sets of points in a distance matrix.
    #   Returns the mapping.
    #
    def GetStableMatching(self, d):
        #rospy.loginfo ('CI d=%s' % d)
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
                if None in M:
                    m = M.index(None)
                else:
                    m = None
                if m is not None:
            #       w = m's highest ranked such woman who he has not proposed to yet
                    d2 = list(N.array(d[m])+N.array(proposed[m]))
                    w = d2.index(min(d2))
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
                        
        

    # MapObjectsToContours()
    #   Uses self.contours & self.objects, and...
    #   Returns a list of indices such that self.objects[k] = contour[map[k]], and self.objects[0]=therobot
    def MapObjectsToContours(self):
        if (self.stateEndEffector is not None):
            ptEndEffector = Point(x = self.stateEndEffector.pose.position.x,
                                  y = self.stateEndEffector.pose.position.y)
        else:
            ptEndEffector = self.objects[0].state.pose.position

        
        # Make a list of object positions (i.e. robots & flies).
        xyObjects = []

        # Robots.
        if self.stateEndEffector is not None:
            xyObjects.append([ptEndEffector.x + self.objects[0].ptOffset.x,
                              ptEndEffector.y + self.objects[0].ptOffset.y])
            #rospy.logwarn ('CI Robot image at %s' % ([self.objects[0].state.pose.position.x,
            #                                          self.objects[0].state.pose.position.y]))
            
        # Flies.    
        for iFly in range(1, len(self.objects)): #1+self.maxFlies): 
            xyObjects.append([self.objects[iFly].state.pose.position.x,
                              self.objects[iFly].state.pose.position.y])
            #rospy.loginfo ('CI Object %s at x,y=%s' % (iFly,[self.objects[iFly].state.pose.position.x,
            #                                                 self.objects[iFly].state.pose.position.y]))
            
        # Make a list of contour positions.
        xyContours = []
        for iContour in range(len(self.contours)):
            if self.contours[iContour] is not None:
                xyContours.append([self.contours[iContour].x,
                                   self.contours[iContour].y])
                #rospy.loginfo ('CI contour %s at x,y=%s' % (iContour,[self.contours[iContour].x,
                #                                               self.contours[iContour].y]))
        
                
        #rospy.loginfo ('CI flies[0,1].isVisible=%s' % [self.objects[0].isVisible, self.objects[1].isVisible])

        # Construct two lists of contours, to describe the range of good robot/fly properties.
        contoursMin = []
        contoursMax = []
        contoursMean = []
        
        # Append the robot contour stats.
        if self.stateEndEffector is not None:
            contour = Contour()
            
            contour.area   = self.objects[0].areaMin #self.areaMinRobot
            contour.ecc    = self.objects[0].eccMin #self.eccMinRobot
            contoursMin.append(contour)
            
            contour.area   = self.objects[0].areaMax #self.areaMaxRobot
            contour.ecc    = self.objects[0].eccMax #self.eccMaxRobot
            contoursMax.append(contour)
            
            contour.area   = self.objects[0].areaSum / self.objects[0].areaCount 
            contour.ecc    = self.objects[0].eccSum / self.objects[0].eccCount
            contoursMean.append(contour)
        
        
        # Append the fly contours.
        for i in range(1, len(self.objects)): #1+self.maxFlies): 
            contour = Contour()
            
            contour.area   = self.objects[i].areaMin #self.areaMinFly
            contour.ecc    = self.objects[i].eccMin #self.eccMinFly
            contoursMin.append(contour)
            
            contour.area   = self.objects[i].areaMax #self.areaMaxFly
            contour.ecc    = self.objects[i].eccMax #self.eccMaxFly
            contoursMax.append(contour)
            
            contour.area   = self.objects[i].areaSum / self.objects[i].areaCount 
            contour.ecc    = self.objects[i].eccSum / self.objects[i].eccCount
            contoursMean.append(contour)


        # Print ecc/area stats.
        #rospy.logwarn ('robot,fly ecc=[%0.2f, %0.2f], area=[%0.2f, %0.2f]' % (self.objects[0].eccSum/self.objects[0].eccCount,
        #                                                                      self.objects[1].eccSum/self.objects[1].eccCount,
        #                                                                      self.objects[0].areaSum/self.objects[0].areaCount,
        #                                                                      self.objects[1].areaSum/self.objects[1].areaCount))
        
                
        # Match flies with contours.
        #d = self.GetDistanceMatrix(xyObjects, xyContours, iPriorities)
        d = self.GetDistanceMatrixFromContours(xyObjects, self.contours, contoursMin, contoursMax, contoursMean)
        if d is not []:
            (mapFliesStableMarriage, mapContours) = self.GetStableMatching(d)
            #(mapFliesHungarian, unmapped) = MatchHungarian.MatchIdentities(d.transpose())
            #rospy.logwarn ('CI mapFliesStableMarriage=%s' % (mapFliesStableMarriage))
            #rospy.logwarn ('CI mapFliesHungarian=%s, unmapped=%s' % (mapFliesHungarian,unmapped))
    
            if True:
                mapFlies = mapFliesStableMarriage
            #else:  
            #    mapFlies = [None for i in range(len(xyObjects))] #list(N.zeros(len(xyObjects)))
            #    for i in range(len(xyObjects)):
            #        if mapFliesHungarian[i] != -1:
            #            mapFlies[i] = int(mapFliesHungarian[i])
            #            #if not unmapped[i]:
            #            #    mapFlies.append(mapFliesHungarian[i])
            #            #else:
            #            #    mapFlies.append(None)
            #        else:
            #            mapFlies[i] = None
            #    #mapFlies = list(mapFlies[i] for i in range(len(mapFlies)))
                
            #map = [None for i in range(self.maxRobots + self.maxFlies)]
            
            # If there's no robot, then prepend a None to the map.
            if self.stateEndEffector is None:
                mapFlies.insert(0,None)
        else:
            mapFlies = [None for k in range(1+len(self.objects))]
            
        #rospy.logwarn ('CI xyObjects=%s' % xyObjects)
        #rospy.logwarn ('CI xyContours=%s' % xyContours)
        #rospy.logwarn ('CI mapFlies=%s' % (mapFlies))
        
        return mapFlies
    

    def ContourInfo_callback(self, contourinfo):
        if self.initialized:
            #rospy.logwarn ('contourinfo callback, stamp=%s' % contourinfo.header.stamp)
            
            #rospy.loginfo ('CI contourinfo0 %s' % contourinfo)
            contourinfo = self.TransformContourinfoPlateFromCamera(contourinfo)
            #rospy.loginfo ('CI contourinfo1 %s' % contourinfo)
            contourinfo = self.FilterContourinfoWithinRadius(contourinfo, self.radiusArena)
            #rospy.loginfo ('CI contourinfo2 %s' % contourinfo)

            # Repackage the contourinfo into a list of contours
            self.contours = []            
            for i in range(len(contourinfo.x)):
                contour = Contour()
                contour.header = contourinfo.header
                contour.x      = contourinfo.x[i]
                contour.y      = contourinfo.y[i]
                contour.angle  = contourinfo.theta[i]
                contour.area   = contourinfo.area[i]
                contour.ecc    = contourinfo.ecc[i]
                self.contours.append(contour)

                # Send the contour transforms.
                self.tfbx.sendTransform((contour.x, 
                                         contour.y, 
                                         0.0),
                                        tf.transformations.quaternion_about_axis(contour.angle, (0,0,1)),
                                        contour.header.stamp,
                                        "contour"+str(i),
                                        "ImageRect")
                
                

            # Figure out who is who in the camera image.
            try:
                self.map = self.MapObjectsToContours()
            except IndexError:
                self.map = None
                
            #rospy.loginfo ('CI map=%s' % self.map)
            #rospy.loginfo ('CI contour[0].x,y=%s' % [self.contours[0].x,self.contours[0].y])
            #rospy.loginfo ('CI contour[1].x,y=%s' % [self.contours[1].x,self.contours[1].y])

            # Update the robot state w/ the contour and end-effector positions.
            if self.map is not None:
                if self.stateEndEffector is not None:
                    if self.map[0] is not None:
                        self.objects[0].Update(self.contours[self.map[0]], self.stateEndEffector.pose.position)
                    else:
                        self.objects[0].Update(None,                            self.stateEndEffector.pose.position)
    
                    # Write a file (for getting Kalman covariances, etc).
                    #data = '%s, %s, %s, %s, %s, %s\n' % (self.stateEndEffector.pose.position.x,
                    #                                     self.stateEndEffector.pose.position.y,
                    #                                     self.objects[0].state.pose.position.x, 
                    #                                     self.objects[0].state.pose.position.y,
                    #                                     self.contours[self.map[0]].x,
                    #                                     self.contours[self.map[0]].y)
                    #self.fidRobot.write(data)
                    
                    #rospy.loginfo ('CI update robot    contour=%s' % contour)
                    
                
                # Update the flies' states.
                for iFly in range(1, len(self.objects)): #1+self.maxFlies):
                    if self.map[iFly] is not None:
                        self.objects[iFly].Update(self.contours[self.map[iFly]], None)
                    else:
                        self.objects[iFly].Update(None,                       None)
    
                    
                    #rospy.loginfo ('CI update state %s contour=%s' % (iFly,contour))
    
                    # Write a file.
                    #if self.map[1] is not None:
                    #    data = '%s, %s, %s, %s\n' % (self.contours[self.map[1]].x, 
                    #                                 self.contours[self.map[1]].y, 
                    #                                 self.objects[1].state.pose.position.x, 
                    #                                 self.objects[1].state.pose.position.y)
                    #    self.fidFly.write(data)
                
    
    
                # Construct the ArenaState message.
                arenastate = ArenaState()
                #if self.objects[0].state.pose.position.x is not None:
                if self.stateEndEffector is not None:
                    arenastate.robot.header.stamp    = self.objects[0].state.header.stamp
                    arenastate.robot.header.frame_id = self.objects[0].state.header.frame_id
                    arenastate.robot.pose            = self.objects[0].state.pose
                    arenastate.robot.velocity        = self.objects[0].state.velocity
                    #rospy.logwarn ('CI robot.position=%s, ptOffset=%s' % ([self.objects[0].state.pose.position.x,
                    #                                                            self.objects[0].state.pose.position.y],
                    #                                                           [self.objects[0].ptOffset.x,
                    #                                                            self.objects[0].ptOffset.y]))
                
                for iFly in range(1, len(self.objects)): #1+self.maxFlies):
                    if (self.map[iFly] is not None) and (self.objects[iFly].state.pose.position.x is not None):
                        arenastate.flies.append(MsgFrameState(header = self.objects[iFly].state.header, 
                                                              pose = self.objects[iFly].state.pose,
                                                              velocity = self.objects[iFly].state.velocity))
                        #rospy.logwarn('arenastate.flies.append(%s)' % self.objects[iFly].name)
                
                # Publish the ArenaState.
                self.pubArenaState.publish(arenastate)
                
                
                # Publish the EndEffectorOffset.
                self.pubEndEffectorOffset.publish(self.objects[0].ptOffset)
                
                
                # Publish a disc to indicate the arena extent.
                self.pubMarker.publish(self.markerArena)
                        


if __name__ == '__main__':
    rospy.init_node('ContourIdentifier')
    ci = ContourIdentifier()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

    #cv.DestroyAllWindows()

