#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('track_image_contours')
import roslib; roslib.load_manifest('plate_tf')

import sys
import rospy
from track_image_contours.msg import ContourInfo
from plate_tf.msg import ImagePose
from plate_tf.srv import *
from geometry_msgs.msg import PointStamped, PoseArray, Pose, PoseStamped
from flystage.msg import *
import tf
import numpy as N
import copy
import filters
import stop_walk as sw
import choose_orientation as co
from pythonmodules import CircleFunctions



class ContourIdentifier:

    def __init__(self):
        self.initialized = False
        
        # Messages
        self.sub_contourinfo = rospy.Subscriber("ContourInfo", ContourInfo, self.contourinfo_callback)
        #self.pub_imagepose = rospy.Publisher("ImagePose", ImagePose)
        #self.sub_imagepose = rospy.Subscriber('ImagePose', ImagePose, self.imagepose_callback)
        self.pub_arenastate = rospy.Publisher('ArenaState', ArenaState)
        #self.pub_arenastate = rospy.Publisher("ArenaState", ArenaState)

        self.tfrx = tf.TransformListener()
        self.tfbx = tf.TransformBroadcaster()
        
        # Poses
        self.imagepose = ImagePose()
        self.poseRobot = Pose()
        self.posearrayFly = PoseArray()
        
        # Points
        self.magnet_magnetframe = PointStamped()
        self.magnet_magnetframe.header.frame_id = "Magnet"
        self.magnet_magnetframe.point.x = 0
        self.magnet_magnetframe.point.y = 0
        
        # Robot Info
        self.robot_min_ecc = rospy.get_param("robot_min_ecc", 0.5)
        self.robot_max_ecc = rospy.get_param("robot_max_ecc", 3.0)
        self.robot_min_area = rospy.get_param("robot_min_area", 10)
        self.robot_max_area = rospy.get_param("robot_max_area", 1000)
        self.robot_visible = bool(rospy.get_param("robot_visible","true"))
        self.found_robot = False

        # Robot Info
        self.robot_visible = bool(rospy.get_param("robot_visible","true"))

        self.kf_fly = filters.KalmanFilter()
        self.kf_robot = filters.KalmanFilter()
        self.lpf_fly_angle = filters.LowPassFilter()
        self.lpf_robot_angle = filters.LowPassFilter()
        self.fly_angle_previous = [None]
        self.robot_angle_previous = [None]

        self.fly_t_previous = None
        self.robot_t_previous = None

        self.sw_fly = sw.StopWalk()
        self.sw_robot = sw.StopWalk()

        self.co_fly = co.ChooseOrientation()
        self.co_robot = co.ChooseOrientation()

        self.angle_threshold = 0.2
        self.position_threshold = 2

        self.t = rospy.get_time()
        self.mask_radius = int(rospy.get_param("mask_radius","225"))

        self.xSave = []
        self.ySave = []
        self.iSave = 0

        rospy.wait_for_service('camera_to_plate')
        try:
            self.camera_to_plate = rospy.ServiceProxy('camera_to_plate', PlateCameraConversion)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        self.initialized = True


    def mag_angle_from_x_y(self, vx, vy):
        vel_mag = N.sqrt(vx**2 + vy**2)
        vel_ang = N.arctan2(vy,vx)
        return (vel_mag,vel_ang)


    def quaternion_camera_to_plate(self, quat):
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
            # rospy.logwarn("z = %s",str(z))
            # rospy.logwarn("w = %s",str(w))
            points_plate_rotated = N.array([points_plate_rotated_x,points_plate_rotated_y,z,w])
            points_plate_rotated = N.append(points_plate_rotated,[[0,0],[0,0],[scale_factor,-scale_factor],[1,1]],axis=1)
            # rospy.logwarn("points_plate_rotated = \n%s", str(points_plate_rotated))
            T = tf.transformations.superimposition_matrix(points_plate_rotated,points_plate)
            # rospy.logwarn("T = \n%s", str(T))
            # al, be, ga = tf.transformations.euler_from_matrix(T, 'rxyz')
            # rospy.logwarn("ga = %s" % str(ga*180/N.pi))
            quat_plate = tf.transformations.quaternion_from_matrix(T)
            return quat_plate
        # except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException):
        except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException, AttributeError, ValueError):
            return None


    def sendTransformFromPose(self, object_name, header, pose):
        object_name_filtered = object_name + "Filtered"
        object_name_image = object_name + "Image"

        if "Fly" in object_name:
            filterPos = self.kf_fly
            sw = self.sw_fly
            co = self.co_fly
            filterAngle = self.lpf_fly_angle
            angle_prev = self.fly_angle_previous
            t_prev = self.fly_t_previous
        else: # "Robot"
            filterPos = self.kf_robot
            sw = self.sw_robot
            co = self.co_robot
            filterAngle = self.lpf_robot_angle
            angle_prev = self.robot_angle_previous
            t_prev = self.robot_t_previous

        try:
            # Get position,orientation of object in plate frame.
            if (("Robot" in object_name) and self.robot_visible) or ("Fly" in object_name):
                xSrc = [pose.position.x]
                ySrc = [pose.position.y]
                if (xSrc is not None) and (ySrc is not None) and \
                       (0 < len(xSrc)) and (0 < len(ySrc)):
                    if abs(pose.orientation.w) < 0.01:
                        w = 1
                    else:
                        w = pose.orientation.w
                        
                    self.tfbx.sendTransform((pose.position.x, pose.position.y, 0),
                                            (pose.orientation.x, pose.orientation.y, pose.orientation.z, w),
                                            header.stamp,
                                            # rospy.Time.now(),
                                            object_name_image,
                                            "Camera")

                    # Convert to Plate coordinates.
                    if header.frame_id == "Plate":
                        x_plate = xSrc[0]
                        y_plate = ySrc[0]
                    else:
                        response = self.camera_to_plate(xSrc, ySrc)
                        x_plate = response.Xdst[0]
                        y_plate = response.Ydst[0]

                    quat_plate = self.quaternion_camera_to_plate((pose.orientation.x,
                                                                  pose.orientation.y,
                                                                  pose.orientation.z,
                                                                  pose.orientation.w))
                    
            elif ("Robot" in object_name) and (not self.robot_visible):
                (trans, quat_plate) = self.tfrx.lookupTransform('Plate', 'Magnet', rospy.Time(0))
                x_plate = trans[0]
                y_plate = trans[1]


            # 
            if quat_plate is not None:
                # rospy.logwarn("x_plate = %s" % (str(x_plate)))
                # rospy.logwarn("y_plate = %s" % (str(y_plate)))
                # rospy.logwarn("quat_plate = %s" % (str(quat_plate)))
                
                t = rospy.Time.now().to_sec() # BUG.  We should really use the header stamp, but am getting 3-in-a-row w/ same time -> dt==0.
                #t = header.stamp.to_sec()
                #rospy.logwarn('ASP t=%s' % t)
                (x,y,vx,vy) = filterPos.update((x_plate,y_plate),t)
                rospy.loginfo ('ASP filtered x,y=%s,%s' % (x,y))
                
                if (vx is not None) and (vy is not None):
                    vel_mag,vel_ang = self.mag_angle_from_x_y(vx,vy)
                    stopped = sw.classify(vel_mag)
                    #state.velocity.linear.x = vx
                    #state.velocity.linear.y = vy
                else:
                    vel_mag = vel_ang = stopped = None

                #if (stopped is not None):
                #    stop_state.stopped = stopped


                # If filtered position is within threshhold, then use it.
                if (x is not None) and (y is not None) and \
                       (abs(x_plate - x) < self.position_threshold) and \
                       (abs(y_plate - y) < self.position_threshold):
                    x_plate = x
                    y_plate = y


                # Choose an orientation.
                if vel_ang is not None:
                    quat_chosen = co.choose_orientation(quat_plate, vel_ang, stopped, angle_prev[0])
                else:
                    quat_chosen = None


                if "Robot" in object_name:
                    quat_chosen = quat_plate


                # Get the angle in Plate frame.
                if quat_chosen is not None:
                    angle_plate = CircleFunctions.mod_angle(tf.transformations.euler_from_quaternion(quat_chosen)[2])
                else:
                    angle_plate = CircleFunctions.mod_angle(tf.transformations.euler_from_quaternion(quat_plate)[2])

                angle_plate = CircleFunctions.unwrap_angle(angle_plate,angle_prev[0])
                angleFiltered = filterAngle.update(angle_plate, t)
                if angleFiltered is not None:
                    quat_plate = tf.transformations.quaternion_about_axis(CircleFunctions.mod_angle(angleFiltered), (0,0,1))

                if vel_ang is not None:
                    quat_chosen = co.choose_orientation(quat_plate, vel_ang, stopped, angle_prev[0])
                else:
                    quat_chosen = None

                #angle_prev[0] = angleFiltered

                if "Robot" in object_name:
                    quat_chosen = quat_plate


                # Finally, the transform of Plate to object.
                if quat_chosen is not None:
                    self.tfbx.sendTransform((x_plate, y_plate, 0),
                                                      quat_chosen,
                                                      header.stamp,
                                                      # rospy.Time.now(),
                                                      object_name,
                                                      "Plate")

        except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException):
            pass


    def arenastateFromImagepose(self, imagepose):
        arenastate = ArenaState()
        
        if imagepose.poseRobot is not None:
            pose = PoseStamped(header=imagepose.header, pose=imagepose.poseRobot)
            arenastate.robot.header = imagepose.header
            arenastate.robot.pose = self.tfrx.transformPose('Plate', pose).pose
            

        #rospy.logwarn ('len(imagepose.poseFlies) = %s' % len(imagepose.poseFlies))
        nFlies = len(imagepose.poseFlies)
        arenastate.flies = [MsgFrameState() for i in range(nFlies)]
        for iFly in range(nFlies):
            arenastate.flies[iFly].header = imagepose.header
            arenastate.flies[iFly].header.frame_id = 'Plate'
            pose = PoseStamped(header=imagepose.header, pose=imagepose.poseFlies[iFly])
            #arenastate.flies[iFly].pose = self.tfrx.transformPose('Plate', pose).pose
            arenastate.flies[iFly].pose = pose.pose
    
        rospy.loginfo ('ASP imagepose flies %s' % imagepose.poseFlies)
        rospy.loginfo ('ASP platepose flies %s' % arenastate.flies)
        
        return arenastate
    
        

    def arenastate_callback(self, arenastate):
        if self.initialized:
            rospy.loginfo ('ASP arenastate.flies: %s' % arenastate.flies)
            
            # Robot
            self.sendTransformFromPose("Robot", arenastate.robot.header, arenastate.robot.pose)
            
            # Flies
            if len(imagepose.poseFlies)>0:
                self.sendTransformFromPose("Fly", imagepose.poseFlies[0].header, imagepose.poseFlies[0].pose) # Send the first as "Fly"
                
            for iFly in range(len(imagepose.poseFlies)):
                self.sendTransformFromPose("Fly%s"%iFly, imagepose.poseFlies[iFly].header, imagepose.poseFlies[iFly].pose) # Send all as "Fly#"
                
                
            # Publish the robot & fly states.
            #rospy.loginfo ('ASP fly=(%s,%s), robot=(%s,%s)' % (self.arenastate.fly.pose.position.x,self.arenastate.fly.pose.position.y,self.arenastate.robot.pose.position.x,self.arenastate.robot.pose.position.y)
            #self.pub_arenastate.publish(arenastateFromImagepose(imagepose))


    def GetRobotListByAreaEcc(self, contourinfo):
        iRobot_list = []
        nContours = min(len(contourinfo.x), len(contourinfo.y), len(contourinfo.theta), len(contourinfo.area), len(contourinfo.ecc))
        for iContour in range(nContours):
            x = contourinfo.x[iContour]
            y = contourinfo.y[iContour]
            theta = contourinfo.theta[iContour]
            area = contourinfo.area[iContour]
            ecc = contourinfo.ecc[iContour]
            rospy.loginfo ('CI contour area=%s<%s<%s, ecc=%s<%s<%s' % (self.robot_min_area, area, self.robot_max_area, self.robot_min_ecc, ecc, self.robot_max_ecc))
            
            # List of potential robots with a valid area and ecc. 
            #if ((self.robot_min_area < area) and (area < self.robot_max_area)) and ((self.robot_min_ecc < ecc) and (ecc < self.robot_max_ecc)):
            iRobot_list.append(iContour)

        return iRobot_list
    
    
    # GetRobotFromContours()
    #   Choose the entry with the minimum magnet-to-contour distance.
    #
    def GetRobotFromContours(self, contourinfo):
        # Construct a list of distances from robot to each contour.
        nContours = min(len(contourinfo.x), len(contourinfo.y), len(contourinfo.theta), len(contourinfo.area), len(contourinfo.ecc))
        dMagnetContour = []
        try:
            self.magnet_stageframe = self.tfrx.transformPoint('Stage', self.magnet_magnetframe)
            rospy.logwarn("CI  magnet_stageframe=%s" % [self.magnet_stageframe.point.x,self.magnet_stageframe.point.y])

            self.magnet_plateframe = self.tfrx.transformPoint('Plate', self.magnet_magnetframe)
            rospy.logwarn("CI  magnet_plateframe=%s" % [self.magnet_plateframe.point.x,self.magnet_plateframe.point.y])

            rospy.logwarn("CI contourinfo=\n%s" % contourinfo)
            
            xMagnet = self.magnet_stageframe.point.x
            yMagnet = self.magnet_stageframe.point.y #+3.0
            for iContour in range(nContours):
              dMagnetContour.append(N.sqrt((contourinfo.x[iContour] - xMagnet)**2 + (contourinfo.y[iContour] - yMagnet)**2))
            rospy.loginfo("CI dMagnetContour = %s" % dMagnetContour)
        except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException):
            pass
        
        # Construct a list of potential robots, and pick one as "the robot".
        iRobot = None
        self.found_robot = False
        if self.robot_visible:
            iRobot_list = self.GetRobotListByAreaEcc(contourinfo)
    
            # Select the entry nearest to the magnet.
            if (0 < len(iRobot_list)) and (len(iRobot_list) <= len(dMagnetContour)):
                dMagnetContourFiltered = [dMagnetContour[iRobot] for iRobot in iRobot_list]
                # rospy.logwarn("dMagnetContourFiltered = %s" % str(dMagnetContourFiltered))
                iRobot = dMagnetContour.index(min(dMagnetContourFiltered))
                # rospy.logwarn("robot = %s" % str(robot))
                self.found_robot = True

        return iRobot


    def ContourinfoWithinRadius(self, contourinfoIn):
        contourinfoOut = ContourInfo()
        if self.initialized:
            contourinfoOut.header = contourinfoIn.header
            contourinfoOut.x = []
            contourinfoOut.y = []
            contourinfoOut.theta = []
            contourinfoOut.area = []
            contourinfoOut.ecc = []
            
            for iContour in range(len(contourinfoIn.x)):
                if N.linalg.norm(N.array([contourinfoIn.x[iContour],contourinfoIn.y[iContour]])) < self.mask_radius:
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
      
    
    def contourinfo_callback(self, contourinfo):
        if self.initialized:
            t = rospy.get_time()
            dt = t - self.t
            f = 1.0/dt
            rospy.loginfo('CI rate=%s hz' % f)
            self.t = t
            
            # Convert the contour points into Plate coordinates.
#            nContours = min(len(contourinfo.x), len(contourinfo.y), len(contourinfo.theta), len(contourinfo.area), len(contourinfo.ecc))
#            xpixels_per_mm = 5.73 #6.25 
#            ypixels_per_mm = 7.21 #6.25 
#            
#            x_list_plate = []            
#            y_list_plate = []            
#            pt = PointStamped()
#            pt.header.frame_id = contourinfo.header.frame_id
#            for i in range(nContours):
#                pt.point.x = -contourinfo.x[i] / xpixels_per_mm
#                pt.point.y = contourinfo.y[i] / ypixels_per_mm
#                ptPlate = pt#self.tfrx.transformPoint('Plate', pt)
#                x_list_plate.append(-ptPlate.point.x) 
#                y_list_plate.append(-ptPlate.point.y)
#
#            contourinfo.x = x_list_plate
#            contourinfo.y = y_list_plate
            rospy.logwarn ('CI contourinfo0 %s' % contourinfo)
            contourinfo = self.ContourinfoWithinRadius(contourinfo)
            rospy.logwarn ('CI contourinfo1 %s' % contourinfo)
            contourinfo = self.ContourinfoPlateFromCamera(contourinfo)
            rospy.logwarn ('CI contourinfo2 %s' % contourinfo)
                
            iRobot = self.GetRobotFromContours(contourinfo)
            if iRobot is not None:      
                # Make a Pose from the robot contour.          
                self.poseRobot.position.x = contourinfo.x[iRobot]
                self.poseRobot.position.y = contourinfo.y[iRobot]
                #q = tf.transformations.quaternion_about_axis(contourinfo.theta[iRobot], (0, 0, 1))
                q = tf.transformations.quaternion_about_axis(0.0, (0, 0, 1))
                self.poseRobot.orientation.x = q[0]
                self.poseRobot.orientation.y = q[1]
                self.poseRobot.orientation.z = q[2]
                self.poseRobot.orientation.w = q[3]
                
                # Put the pose into imagepose
                self.imagepose.poseRobot = self.poseRobot

                #self.xSave.append([self.magnet_plateframe.point.x, self.poseRobot.position.x])
                #self.ySave.append([self.magnet_plateframe.point.y, self.poseRobot.position.y])
                #self.iSave = self.iSave+1
            else:
                self.imagepose.poseRobot = Pose()

            
            #if (self.iSave % 2000) == 0:
            #    rospy.logwarn ('xSave=%s' % self.xSave)
            #    rospy.logwarn ('ySave=%s' % self.ySave)
                
            
            
            # Construct the list of potential flies (as indices).
            nContours = min(len(contourinfo.x), len(contourinfo.y), len(contourinfo.theta), len(contourinfo.area), len(contourinfo.ecc))
            if self.found_robot and (1 < nContours):
                iFly_list = list(set(range(nContours)).difference([iRobot]))
            elif (not self.found_robot) and (1 <= nContours):
                iFly_list = range(nContours)
            else:
                iFly_list = []
                
            # Construct a list of distances robot-to-contour.
            dRobotContour=[]
            if iRobot is not None: 
                for i in range(nContours):
                    dRobotContour.append(N.linalg.norm([contourinfo.x[i]-contourinfo.x[iRobot],
                                                        contourinfo.y[i]-contourinfo.y[iRobot]]))
            else:
                for i in range(nContours):
                    dRobotContour.append(N.linalg.norm([contourinfo.x[i]-0.0,
                                                        contourinfo.y[i]-0.0]))
                
            rospy.logwarn('CI dRobotContour        = %s' % dRobotContour)
            #rospy.logwarn('CI iFly_list        = %s' % iFly_list)
            
            # Sort iFly_list by distance to robot.
            iFly_list_sorted = iFly_list
            if len(iFly_list_sorted)>1:
                iFly_list_sorted.sort(lambda iFlyA,iFlyB: cmp(dRobotContour[iFlyA],dRobotContour[iFlyB]))
             
            rospy.logwarn('CI iFly_list_sorted = %s' % iFly_list_sorted)
            
            
            # Construct the list of potential flies (as posearrayFly).
            poseFly = Pose()
            if 0 < len(iFly_list_sorted):
                self.posearrayFly.poses = []
                rospy.loginfo ('CI iRobot=%s, iFly_list_sorted=%s' % (iRobot, iFly_list_sorted))
                for iFly in iFly_list_sorted:
                    rospy.loginfo('CI copying iFly=%s' % iFly)
                    poseFly.position.x = contourinfo.x[iFly]
                    poseFly.position.y = contourinfo.y[iFly]
                    q = tf.transformations.quaternion_about_axis(contourinfo.theta[iFly], (0, 0, 1))
                    poseFly.orientation.x = q[0]
                    poseFly.orientation.y = q[1]
                    poseFly.orientation.z = q[2]
                    poseFly.orientation.w = q[3]
                    self.posearrayFly.poses.append(copy.deepcopy(poseFly))
            else:
                self.posearrayFly = PoseArray()
            
            rospy.logwarn ('CI len(posearrayFly.poses)=%s' % len(self.posearrayFly.poses))
            
            # Put the fly poses into imagepose, and publish it.
            self.imagepose.poseFlies = self.posearrayFly.poses
            self.imagepose.header = contourinfo.header
            #self.pub_imagepose.publish(self.imagepose)
            #rospy.loginfo ('ASP len(poseFlies)=%s' % len(self.imagepose.poseFlies))
            rospy.loginfo ('ASP imagepose.fly: %s' % self.imagepose.poseFlies)
            
            # Robot
            self.sendTransformFromPose("Robot", self.imagepose.header, self.imagepose.poseRobot)
            
            # Flies
            if len(self.imagepose.poseFlies)>0:
                self.sendTransformFromPose("Fly", self.imagepose.header, self.imagepose.poseFlies[0]) # Send the first as "Fly"
#                
#            for iFly in range(len(self.imagepose.poseFlies)):
#                self.sendTransformFromPose("Fly%s"%iFly, self.imagepose.header, self.imagepose.poseFlies[iFly]) # Send all as "Fly#"
                
                
            # Find the nearest fly.
            arenastate = self.arenastateFromImagepose(self.imagepose)
            self.pub_arenastate.publish(arenastate)


            #rospy.logwarn ('ASP len(arenastate.flies)=%s, rBest=%s' % (len(arenastate.flies), rBest))



if __name__ == '__main__':
    rospy.init_node('ContourIdentifier')
    ci = ContourIdentifier()

    #rospy.init_node('ArenaStatePublisher')
    #ptc = ArenaStatePublisher()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

    #cv.DestroyAllWindows()

