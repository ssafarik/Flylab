#!/usr/bin/env python
import roslib; roslib.load_manifest('plate_tf')
import rospy

import tf
import numpy as N
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from plate_tf.srv import *
import filters
import stop_walk as sw
import choose_orientation as co
from plate_tf.msg import ImagePose
from plate_tf.msg import ArenaState
from flystage.msg import *
import plate_tf.msg
from pythonmodules import CircleFunctions


class ArenaStatePublisher:
    def __init__(self):
        self.initialized = False
        self.tfrx = tf.TransformListener()
        self.tfbx = tf.TransformBroadcaster()
        
        self.sub_imagepose = rospy.Subscriber('ImagePose', ImagePose, self.imagepose_callback)
        self.pub_arenastate = rospy.Publisher('ArenaState', ArenaState)

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
    
        
    def imagepose_callback(self, imagepose):
        if self.initialized:
            #rospy.loginfo ('ASP len(poseFlies)=%s' % len(imagepose.poseFlies))
            rospy.loginfo ('ASP imagepose.fly: %s' % imagepose.poseFlies)
            
            # Robot
            self.sendTransformFromPose("Robot", imagepose.header, imagepose.poseRobot)
            
            # Flies
            if len(imagepose.poseFlies)>0:
                self.sendTransformFromPose("Fly", imagepose.header, imagepose.poseFlies[0]) # Send the first as "Fly"
#                
#            for iFly in range(len(imagepose.poseFlies)):
#                self.sendTransformFromPose("Fly%s"%iFly, imagepose.header, imagepose.poseFlies[iFly]) # Send all as "Fly#"
                
                
            # Find the nearest fly.
            arenastate = self.arenastateFromImagepose(imagepose)
            self.pub_arenastate.publish(arenastate)


            #rospy.logwarn ('ASP len(arenastate.flies)=%s, rBest=%s' % (len(arenastate.flies), rBest))


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


if __name__ == '__main__':
    rospy.init_node('ArenaStatePublisher')
    ptc = ArenaStatePublisher()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
