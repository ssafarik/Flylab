#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('track_image_contours')

import sys
import rospy
import tf
import numpy as N
from track_image_contours.msg import *
from plate_tf.srv import *
from geometry_msgs.msg import Point, PointStamped, PoseArray, Pose, PoseStamped, Quaternion, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from flystage.msg import *
import copy
import filters
from pythonmodules import CircleFunctions
import MatchHungarian

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
    
    

# class Fly()
# Implements a fly (or robot) object in a 2D arena. 
# Updates its state based on an image "contour" from a single camera.
#
class Fly:
    def __init__(self, name=None):
        self.initialized = False
        self.tfbx = tf.TransformBroadcaster()

        self.kfState = filters.KalmanFilter()
        self.lpAngleF = filters.LowPassAngleFilter(RC=0.01)
        
        self.isVisible = False
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
        
        self.angleOfTravelRecent = None
        self.flip = False
        self.lpFlip = filters.LowPassFilter(RC=3.0)
        self.contour = None
        
        self.minEcc = 999
        self.maxEcc = 0
        self.minArea = 999
        self.maxArea = 0
        
        self.lpOffsetX = filters.LowPassFilter(RC=0.01)
        self.lpOffsetY = filters.LowPassFilter(RC=0.01)
        self.positionOffset = Point(x=0, y=0, z=0)  # Difference between contour position and (for robots) computed position.
        self.name = name
        self.isDead = False # To do.
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
                
            #if "Fly" in self.name:
            #    rospy.logwarn ('CI flip=%s, speed=%+0.3f angleC=%+0.3f, angleT=%+0.3f, flipValuePre=%+0.3f+%+0.3f=%+0.3f flipValuePost=%+0.3f' % (self.flip,
            #                                                                                                                                      speed,
            #                                                                                                                                      angleContour,
            #                                                                                                                                      angleOfTravel,
            #                                                                                                                                      (1/(1+speed) * flipValueLast),
            #                                                                                                                                      (1-(1/(1+speed))) * flipValueNew,
            #                                                                                                                                      flipValuePre,
            #                                                                                                                                      flipValuePost))
            
            self.state.pose.orientation = self.GetResolvedQuaternion()

        #if "Fly" in self.name:
        #    rospy.logwarn ('CI ecc=%s, eccmetric=%s speed=%s, angTravel=%s, angOrientation=%s, flip=%s' % (contour.ecc, eccmetric, speed, angleOfTravel, angleContour, self.flip))
    
                            

    # Update()
    # Update the current state using the visual position and the computed position (if applicable)
    def Update(self, contour, posComputed):
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
                if contour.ecc < self.minEcc:
                    self.minEcc = contour.ecc
                if self.maxEcc < contour.ecc:
                    self.maxEcc = contour.ecc
                if contour.area < self.minArea:
                    self.minArea = contour.area
                if self.maxArea < contour.area:
                    self.maxArea = contour.area
                
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
                            #if "Fly" in self.name:
                            #    rospy.logwarn('CI d=%+0.3f = %+0.3f-%+0.3f,  flipPrev=%s,  self.flip=%s' % (d, anglePrev, self.contour.angle, flipPrev, self.flip))
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
                rospy.logwarn('CI FILTER READ on contour==None')
                
                
            if self.isVisible:
                # Store the latest state.
                #if "Fly" in self.name:
                #    rospy.logwarn ('CI angleContour=%s' % angleContour)
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
                
                #rospy.logwarn ('CI %s ecc=%s,%s, area=%s,%s, speed=%0.3f' % (self.name, contour.ecc,[self.minEcc,self.maxEcc], contour.area,[self.minArea,self.maxArea], speed))
                
                # Update the tool offset.
                if posComputed is not None:
                    self.positionOffset = Point(x = self.lpOffsetX.Update(x-posComputed.x, t),
                                                y = self.lpOffsetY.Update(y-posComputed.y, t),
                                                z = 0) # Filtered(posContour)-posComputed
                else:
                    self.positionOffset = Point(x=0, y=0, z=0)
            

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

        #rospy.loginfo ('CI %s contour=%s, self.isVisible=%s' % (self.name, contour, self.isVisible))
            
        
class ContourIdentifier:

    def __init__(self, maxRobots=1, maxFlies=1):
        self.initialized = False
        self.maxFlies = maxFlies
        self.maxRobots = maxRobots
        
        self.contours = []
        self.map = []
        self.iContours = []
        
        self.objects = []
        for i in range(self.maxRobots+self.maxFlies):
            self.objects.append(Fly())
            self.objects[i].name = "Fly%s" % i
        
        if self.maxRobots>0:
            self.objects[0].name = "Robot"
        
        if self.maxFlies>0:
            self.objects[1].name = "Fly"
            
        
        # Messages
        self.sub_contourinfo = rospy.Subscriber("ContourInfo", ContourInfo, self.ContourInfo_callback)
        self.sub_EndEffector = rospy.Subscriber('EndEffector', MsgFrameState, self.EndEffector_callback)
        self.pub_arenastate = rospy.Publisher('ArenaState', ArenaState)
        self.pub_EndEffectorOffset = rospy.Publisher('EndEffectorOffset', Point)

        self.tfrx = tf.TransformListener()
        self.tfbx = tf.TransformBroadcaster()
        
        # Poses
        self.stateEndEffector = None
        self.poseRobot = Pose()
        self.posearrayFly = PoseArray()
        
        # Points
        self.endeffector_endeffectorframe = PointStamped()
        self.endeffector_endeffectorframe.header.frame_id = "EndEffector"
        self.endeffector_endeffectorframe.point.x = 0
        self.endeffector_endeffectorframe.point.y = 0
        self.endeffector_endeffectorframe.point.z = 0
        
        # Robot Info
        self.robot_min_ecc = 0.9 #rospy.get_param("robot_min_ecc", 0.5)
        self.robot_max_ecc = 1.50 #rospy.get_param("robot_max_ecc", 3.0)
        self.robot_min_area = 45 #rospy.get_param("robot_min_area", 10)
        self.robot_max_area = 75 #rospy.get_param("robot_max_area", 1000)
        #self.robot_visible = bool(rospy.get_param("robot_visible","true"))
        #self.found_robot = False

        self.fly_min_ecc = 1.51 #rospy.get_param("robot_min_ecc", 0.5)
        self.fly_max_ecc = 10.0 #rospy.get_param("robot_max_ecc", 3.0)
        self.fly_min_area = 35 #rospy.get_param("robot_min_area", 10)
        self.fly_max_area = 100 #rospy.get_param("robot_max_area", 1000)
        
        # Robot Info
        self.robot_visible = bool(rospy.get_param("robot_visible","true"))

        self.t = rospy.get_time()
        #self.mask_radius = int(rospy.get_param("mask_radius","225"))
        self.radiusInBounds = float(rospy.get_param("in_bounds_radius","85"))

        self.xSave = []
        self.ySave = []
        self.iSave = 0

        self.pubMarker = rospy.Publisher('visualization_marker', Marker)
        self.markerArena = Marker(header=Header(stamp = rospy.Time.now(),
                                                frame_id='Plate'),
                                  ns='arena',
                                  id=2,
                                  type=3, #CYLINDER,
                                  action=0,
                                  pose=Pose(position=Point(x=0, 
                                                           y=0, 
                                                           z=0)),
                                  scale=Vector3(x=self.radiusInBounds*2.0,
                                                y=self.radiusInBounds*2.0,
                                                z=0.01),
                                  color=ColorRGBA(a=0.05,
                                                  r=1.0,
                                                  g=1.0,
                                                  b=1.0),
                                  lifetime=rospy.Duration(0.1))


        rospy.wait_for_service('camera_to_plate')
        try:
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
        self.stateEndEffector = copy.deepcopy(state)
        self.stateEndEffector.header.frame_id = "Plate" # Interpret it as the Plate frame. 
        #rospy.loginfo ('CI received state=%s' % state)
        


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




    def ContourinfoWithinRadius(self, contourinfoIn, radius):
        contourinfoOut = ContourInfo()
        if self.initialized:
            contourinfoOut.header = contourinfoIn.header
            contourinfoOut.x = []
            contourinfoOut.y = []
            contourinfoOut.theta = []
            contourinfoOut.area = []
            contourinfoOut.ecc = []
            
            #contourinfoPlate = ContourinfoPlateFromCamera(contourinfoIn)
            for iContour in range(len(contourinfoIn.x)):
                if N.linalg.norm(N.array([contourinfoIn.x[iContour],contourinfoIn.y[iContour]])) <= radius:
                #if N.linalg.norm(N.array([contourinfoPlate.x[iContour],contourinfoPlate.y[iContour]])) <= radius:
                #if N.linalg.norm(N.array([ptPlate.x,ptPlate.y])) <= radius:
                    contourinfoOut.x.append(contourinfoIn.x[iContour])
                    contourinfoOut.y.append(contourinfoIn.y[iContour])
                    contourinfoOut.theta.append(contourinfoIn.theta[iContour])
                    contourinfoOut.area.append(contourinfoIn.area[iContour])
                    contourinfoOut.ecc.append(contourinfoIn.ecc[iContour])
                    
        return contourinfoOut
        
        
    def ContourinfoPlateFromCamera(self, contourinfoIn):
        contourinfoOut = ContourInfo()
        if self.initialized:
            response = self.camera_to_plate(contourinfoIn.x, contourinfoIn.y)
            contourinfoOut = contourinfoIn
            contourinfoOut.x = response.Xdst
            contourinfoOut.y = response.Ydst
            contourinfoOut.header.frame_id = "Plate"
    
        return contourinfoOut
      
    
    def DistanceMatrix(self, xy1, xy2, iPriorities):
        d = N.array([[N.inf for n in range(len(xy2))] for m in range(len(xy1))])
        for m in range(len(xy1)):
            for n in range(len(xy2)):
                try:
                    d[m,n] = N.linalg.norm([xy1[m][0]-xy2[n][0],
                                             xy1[m][1]-xy2[n][1]])
                    #d[m,n] = d[m,n]**3
                    
                    # Quick & dirty priority calc.
                    d[m,n] += iPriorities[m]
                except TypeError:
                    d[m,n] = None
        
        #rospy.loginfo('CI DM xy1=%s' % xy1)
        #rospy.loginfo('CI DM xy2=%s' % xy2)
        #rospy.loginfo('CI DM d=%s' % d)

        return d
    
    
    def DistanceMatrixContours(self, xyObjects, contours, contoursMin, contoursMax):
        d = N.array([[N.inf for n in range(len(contours))] for m in range(len(xyObjects))])
        for m in range(len(xyObjects)):
            for n in range(len(contours)):
                try:
                    d[m,n] = N.linalg.norm([xyObjects[m][0]-contours[n].x,
                                             xyObjects[m][1]-contours[n].y])
                    
                    # Penalize deviations from ideal visual characteristics.
                    if contours[n].ecc <= contoursMin[m].ecc:
                        d[m,n] += 200 
                    if contoursMax[m].ecc <= contours[n].ecc: 
                        d[m,n] += 200 
                    if contours[n].area <= contoursMin[m].area:
                        d[m,n] += 0 
                    if contoursMax[m].area <= contours[n].area: 
                        d[m,n] += 0 
                        
                except TypeError:
                    d[m,n] = None
        
        #rospy.loginfo('CI DM xyObjects=%s' % xyObjects)
        #rospy.loginfo('CI DM contours=%s' % contours)
        #rospy.loginfo('CI DM contoursMin=%s' % contoursMin)
        #rospy.loginfo('CI DM contoursMax=%s' % contoursMax)
        #rospy.loginfo('CI DM d=%s' % d)

        return d
    
    
    # GetStableMatching()
    #   Solve the "Stable Marriage Problem" for two sets of points and a distance metric.
    #   Returns the mapping.
    #
    def GetStableMatching(self, d):#xy1, xy2, iPriorities):
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
            posEndEffector = Point(x = self.stateEndEffector.pose.position.x,
                                   y = self.stateEndEffector.pose.position.y)
        else:
            posEndEffector = self.objects[0].state.pose.position

        
        # Make a list of object positions (i.e. robots & flies).
        xyObjects = []
        # Robots.
        for i in range(self.maxRobots): 
            xyObjects.append([posEndEffector.x + self.objects[i].positionOffset.x,
                              posEndEffector.y + self.objects[i].positionOffset.y])
            #xyObjects.append([self.objects[i].state.pose.position.x,
            #                  self.objects[i].state.pose.position.y])
            
            #rospy.logwarn ('CI   posEndEffector+Offset at %s' % ([posEndEffector.x + self.objects[i].positionOffset.x,
            #                                          posEndEffector.y + self.objects[i].positionOffset.y]))
            #rospy.logwarn ('CI Robot image at %s' % ([self.objects[i].state.pose.position.x,
            #                                          self.objects[i].state.pose.position.y]))
            
        # Flies.    
        for i in range(self.maxRobots, self.maxRobots + self.maxFlies): 
            xyObjects.append([self.objects[i].state.pose.position.x,
                              self.objects[i].state.pose.position.y])
            #rospy.loginfo ('CI known object %s at x,y=%s' % (i,[self.objects[i].state.pose.position.x,
            #                                                    self.objects[i].state.pose.position.y]))
            
        # Make a list of contour positions.
        xyContours = []
        for i in range(len(self.contours)):
            if self.contours[i] is not None:
                xyContours.append([self.contours[i].x,
                                   self.contours[i].y])
                #rospy.loginfo ('CI contour %s at x,y=%s' % (i,[self.contours[i].x,
                #                                               self.contours[i].y]))
        
                
        #rospy.loginfo ('CI flies[0,1].isVisible=%s' % [self.objects[0].isVisible, self.objects[1].isVisible])

        # Construct two lists of contours, to describe the range of good robot/fly properties.
        contoursMin = []
        contoursMax = []
        for i in range(self.maxRobots):
            contour = Contour()
            contour.area   = self.robot_min_area
            contour.ecc    = self.robot_min_ecc
            contoursMin.append(contour)
            contour = Contour()
            contour.area   = self.robot_max_area
            contour.ecc    = self.robot_max_ecc
            contoursMax.append(contour)
        for i in range(self.maxRobots, self.maxRobots + self.maxFlies): 
            contour = Contour()
            contour.area   = self.fly_min_area
            contour.ecc    = self.fly_min_ecc
            contoursMin.append(contour)
            contour = Contour()
            contour.area   = self.fly_max_area
            contour.ecc    = self.fly_max_ecc
            contoursMax.append(contour)

                
        # Match flies with contours.
        #d = self.DistanceMatrix(xyObjects, xyContours, iPriorities)
        d = self.DistanceMatrixContours(xyObjects, self.contours, contoursMin, contoursMax)
        (mapFliesStableMarriage, mapContours) = self.GetStableMatching(d)
        (mapFliesHungarian,unmapped) = MatchHungarian.MatchIdentities(d.transpose())
        #rospy.logwarn ('CI mapFliesStableMarriage=%s' % (mapFliesStableMarriage))
        #rospy.logwarn ('CI mapFliesHungarian=%s, unmapped=%s' % (mapFliesHungarian,unmapped))

        if True:
            mapFlies = mapFliesStableMarriage
        else:  
            mapFlies = [None for i in range(len(xyObjects))] #list(N.zeros(len(xyObjects)))
            for i in range(len(xyObjects)):
                if mapFliesHungarian[i] != -1:
                    mapFlies[i] = int(mapFliesHungarian[i])
                    #if not unmapped[i]:
                    #    mapFlies.append(mapFliesHungarian[i])
                    #else:
                    #    mapFlies.append(None)
                else:
                    mapFlies[i] = None
            #mapFlies = list(mapFlies[i] for i in range(len(mapFlies)))
            
        #rospy.logwarn ('CI xyObjects=%s' % xyObjects)
        #rospy.logwarn ('CI xyContours=%s' % xyContours)
        #rospy.logwarn ('CI mapFlies=%s' % (mapFlies))
        
        #map = [None for i in range(self.maxRobots + self.maxFlies)]
        
        return mapFlies
    

    def ContourInfo_callback(self, contourinfo):
        if self.initialized:
            #rospy.loginfo ('CI contourinfo0 %s' % contourinfo)
            contourinfo = self.ContourinfoPlateFromCamera(contourinfo)
            #rospy.loginfo ('CI contourinfo1 %s' % contourinfo)
            contourinfo = self.ContourinfoWithinRadius(contourinfo, self.radiusInBounds)
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
                                        "Camera")
                
                

            # Figure out who is who in the camera image.
            self.map = self.MapObjectsToContours()
            #rospy.loginfo ('CI map=%s' % self.map)
            #rospy.loginfo ('CI contour[0].x,y=%s' % [self.contours[0].x,self.contours[0].y])
            #rospy.loginfo ('CI contour[1].x,y=%s' % [self.contours[1].x,self.contours[1].y])

            # Update the robot state w/ the contour and end-effector positions.
            if self.stateEndEffector is not None:
                if self.map[0] is not None:
                    self.objects[0].Update(self.contours[self.map[0]], self.stateEndEffector.pose.position)
                else:
                    self.objects[0].Update(None,                       self.stateEndEffector.pose.position)

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
            for i in range(self.maxRobots, self.maxRobots+self.maxFlies):
                if self.map[i] is not None:
                    self.objects[i].Update(self.contours[self.map[i]], None)
                else:
                    self.objects[i].Update(None,                       None)

                
                #rospy.loginfo ('CI update state %s contour=%s' % (i,contour))

                # Write a file.
                #if self.map[1] is not None:
                #    data = '%s, %s, %s, %s\n' % (self.contours[self.map[1]].x, 
                #                                 self.contours[self.map[1]].y, 
                #                                 self.objects[1].state.pose.position.x, 
                #                                 self.objects[1].state.pose.position.y)
                #    self.fidFly.write(data)
            


            # Construct the ArenaState message.
            arenastate = ArenaState()
            if self.objects[0].state.pose.position.x is not None:
                arenastate.robot.header.stamp    = self.objects[0].state.header.stamp
                arenastate.robot.header.frame_id = self.objects[0].state.header.frame_id
                arenastate.robot.pose            = self.objects[0].state.pose
                arenastate.robot.velocity        = self.objects[0].state.velocity
                #rospy.logwarn ('CI robot.position=%s, positionOffset=%s' % ([self.objects[0].state.pose.position.x,
                #                                                            self.objects[0].state.pose.position.y],
                #                                                           [self.objects[0].positionOffset.x,
                #                                                            self.objects[0].positionOffset.y]))
            nFlies = len(self.objects)
            for iFly in range(nFlies-1):
                if (self.map[iFly+1] is not None) and (self.objects[iFly+1].state.pose.position.x is not None):
                    arenastate.flies.append(MsgFrameState(header = self.objects[iFly+1].state.header, 
                                                          pose = self.objects[iFly+1].state.pose,
                                                          velocity = self.objects[iFly+1].state.velocity))
            
            # Publish the ArenaState.
            self.pub_arenastate.publish(arenastate)
            
            # Publish the EndEffectorOffset.
            ptOffset = self.objects[0].positionOffset
            self.pub_EndEffectorOffset.publish(ptOffset)
            
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

