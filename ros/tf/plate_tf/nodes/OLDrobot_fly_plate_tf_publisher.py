#!/usr/bin/env python
import roslib
roslib.load_manifest('plate_tf')
import rospy

import tf, numpy, math
from geometry_msgs.msg import PoseStamped,Pose,PoseArray
from plate_tf.srv import *
import filters
import stop_walk as sw
import choose_orientation as co
from plate_tf.msg import StopState, InBoundsState, FilteredData
from pythonmodules import CircleFunctions

class PoseTFConversion:
    def __init__(self):
        self.initialized = False
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.robot_image_pose_sub = rospy.Subscriber('ImagePose/Robot',PoseStamped,self.handle_robot_image_pose)
        self.fly_image_pose_sub = rospy.Subscriber('ImagePose/Fly',PoseArray,self.handle_fly_image_pose)

        self.fly_image_pose = PoseStamped()

        self.fly_stop_pub = rospy.Publisher('StopState/Fly',StopState)
        self.fly_in_bounds_pub = rospy.Publisher('InBoundsState/Fly',InBoundsState)
        self.robot_stop_pub = rospy.Publisher('StopState/Robot',StopState)
        self.robot_in_bounds_pub = rospy.Publisher('InBoundsState/Robot',InBoundsState)

        self.fly_stop_state = StopState()
        self.fly_in_bounds_state = InBoundsState()
        self.robot_stop_state = StopState()
        self.robot_in_bounds_state = InBoundsState()

        self.kf_fly = filters.KalmanFilter()
        self.kf_robot = filters.KalmanFilter()
        self.lpf_fly_angle = filters.LowPassFilter()
        self.lpf_robot_angle = filters.LowPassFilter()
        self.fly_angle_previous = [None]
        self.robot_angle_previous = [None]

        self.sw_fly = sw.StopWalk()
        self.sw_robot = sw.StopWalk()

        self.co_fly = co.ChooseOrientation()
        self.co_robot = co.ChooseOrientation()

        self.in_bounds_radius = rospy.get_param('in_bounds_radius',100)

        self.robot_x_filtered_data = FilteredData()
        self.robot_x_filtered_data_pub = rospy.Publisher('FilteredData/RobotX',FilteredData)
        self.robot_y_filtered_data = FilteredData()
        self.robot_y_filtered_data_pub = rospy.Publisher('FilteredData/RobotY',FilteredData)
        self.robot_a_filtered_data = FilteredData()
        self.robot_vx_filtered_data = FilteredData()
        self.robot_vx_filtered_data_pub = rospy.Publisher('FilteredData/RobotVX',FilteredData)
        self.robot_vy_filtered_data = FilteredData()
        self.robot_vy_filtered_data_pub = rospy.Publisher('FilteredData/RobotVY',FilteredData)
        self.robot_a_filtered_data = FilteredData()
        self.robot_a_filtered_data_pub = rospy.Publisher('FilteredData/RobotAngle',FilteredData)

        self.fly_x_filtered_data = FilteredData()
        self.fly_x_filtered_data_pub = rospy.Publisher('FilteredData/FlyX',FilteredData)
        self.fly_y_filtered_data = FilteredData()
        self.fly_y_filtered_data_pub = rospy.Publisher('FilteredData/FlyY',FilteredData)
        self.fly_vx_filtered_data = FilteredData()
        self.fly_vx_filtered_data_pub = rospy.Publisher('FilteredData/FlyVX',FilteredData)
        self.fly_vy_filtered_data = FilteredData()
        self.fly_vy_filtered_data_pub = rospy.Publisher('FilteredData/FlyVY',FilteredData)
        self.fly_a_filtered_data = FilteredData()
        self.fly_a_filtered_data_pub = rospy.Publisher('FilteredData/FlyAngle',FilteredData)

        self.angle_threshold = 0.2
        self.position_threshold = 2

        rospy.wait_for_service('camera_to_plate')
        try:
            self.camera_to_plate = rospy.ServiceProxy('camera_to_plate', PlateCameraConversion)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        self.initialized = True

    def mag_angle_from_x_y(self,vx,vy):
        vel_mag = math.sqrt(vx**2 + vy**2)
        vel_ang = math.atan2(vy,vx)
        return (vel_mag,vel_ang)

    def quaternion_camera_to_plate(self,quat):
        # Must be cleverer way to calculate this using quaternion math...
        R = tf.transformations.quaternion_matrix(quat)
        scale_factor = 100
        points_camera = numpy.array(\
            [[0,  1, -1,  0,  0,  1, -1],
             [0,  0,  0,  1, -1,  1, -1],
             [0,  0,  0,  0,  0,  0,  0],
             [1,  1,  1,  1,  1,  1,  1]])
        points_camera = points_camera*scale_factor
        # rospy.logwarn("points_camera = \n%s", str(points_camera))
        points_camera_rotated = numpy.dot(R,points_camera)
        # rospy.logwarn("points_camera_rotated = \n%s", str(points_camera_rotated))
        try:
            Xsrc = list(points_camera[0,:])
            Ysrc = list(points_camera[1,:])
            # rospy.logwarn("Xsrc = %s", str(Xsrc))
            # rospy.logwarn("Ysrc = %s", str(Ysrc))
            response = self.camera_to_plate(Xsrc,Ysrc)
            points_plate_x = list(response.Xdst)
            points_plate_y = list(response.Ydst)
            z = [0]*len(points_plate_x)
            w = [1]*len(points_plate_y)
            points_plate = numpy.array([points_plate_x,points_plate_y,z,w])
            points_plate = numpy.append(points_plate,[[0,0],[0,0],[scale_factor,-scale_factor],[1,1]],axis=1)
            # rospy.logwarn("points_plate = \n%s", str(points_plate))

            Xsrc = list(points_camera_rotated[0,:])
            Ysrc = list(points_camera_rotated[1,:])
            response = self.camera_to_plate(Xsrc,Ysrc)
            points_plate_rotated_x = list(response.Xdst)
            points_plate_rotated_y = list(response.Ydst)
            # rospy.logwarn("points_plate_rotated_x = %s",str(points_plate_rotated_x))
            # rospy.logwarn("points_plate_rotated_y = %s",str(points_plate_rotated_y))
            # rospy.logwarn("z = %s",str(z))
            # rospy.logwarn("w = %s",str(w))
            points_plate_rotated = numpy.array([points_plate_rotated_x,points_plate_rotated_y,z,w])
            points_plate_rotated = numpy.append(points_plate_rotated,[[0,0],[0,0],[scale_factor,-scale_factor],[1,1]],axis=1)
            # rospy.logwarn("points_plate_rotated = \n%s", str(points_plate_rotated))
            T = tf.transformations.superimposition_matrix(points_plate_rotated,points_plate)
            # rospy.logwarn("T = \n%s", str(T))
            # al, be, ga = tf.transformations.euler_from_matrix(T, 'rxyz')
            # rospy.logwarn("ga = %s" % str(ga*180/numpy.pi))
            quat_plate = tf.transformations.quaternion_from_matrix(T)
            return quat_plate
        # except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException):
        except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException, AttributeError, ValueError):
            return None

    def image_pose_handler(self,object_name,msg):
        if "Fly" in object_name:
            image_frame_name = "FlyImage"
            frame_name = "Fly"
            kf = self.kf_fly
            sw = self.sw_fly
            co = self.co_fly
            stop_pub = self.fly_stop_pub
            in_bounds_pub = self.fly_in_bounds_pub
            stop_state = self.fly_stop_state
            in_bounds_state = self.fly_in_bounds_state
            x_filtered_data = self.fly_x_filtered_data
            x_filtered_data_pub = self.fly_x_filtered_data_pub
            y_filtered_data = self.fly_y_filtered_data
            y_filtered_data_pub = self.fly_y_filtered_data_pub
            vx_filtered_data = self.fly_vx_filtered_data
            vx_filtered_data_pub = self.fly_vx_filtered_data_pub
            vy_filtered_data = self.fly_vy_filtered_data
            vy_filtered_data_pub = self.fly_vy_filtered_data_pub
            a_filtered_data = self.fly_a_filtered_data
            a_filtered_data_pub = self.fly_a_filtered_data_pub
            lpf_angle = self.lpf_fly_angle
            a_prev = self.fly_angle_previous
        else:
            image_frame_name = "RobotImage"
            frame_name = "Robot"
            kf = self.kf_robot
            sw = self.sw_robot
            co = self.co_robot
            stop_pub = self.robot_stop_pub
            in_bounds_pub = self.robot_in_bounds_pub
            stop_state = self.robot_stop_state
            in_bounds_state = self.robot_in_bounds_state
            x_filtered_data = self.robot_x_filtered_data
            x_filtered_data_pub = self.robot_x_filtered_data_pub
            y_filtered_data = self.robot_y_filtered_data
            y_filtered_data_pub = self.robot_y_filtered_data_pub
            vx_filtered_data = self.robot_vx_filtered_data
            vx_filtered_data_pub = self.robot_vx_filtered_data_pub
            vy_filtered_data = self.robot_vy_filtered_data
            vy_filtered_data_pub = self.robot_vy_filtered_data_pub
            a_filtered_data = self.robot_a_filtered_data
            a_filtered_data_pub = self.robot_a_filtered_data_pub
            lpf_angle = self.lpf_robot_angle
            a_prev = self.robot_angle_previous

        try:
            Xsrc = [msg.pose.position.x]
            Ysrc = [msg.pose.position.y]
            if (Xsrc is not None) and (Ysrc is not None) and \
                   (0 < len(Xsrc)) and (0 < len(Ysrc)):
                self.tf_broadcaster.sendTransform((msg.pose.position.x, msg.pose.position.y, 0),
                                                  (msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w),
                                                  rospy.Time.now(),
                                                  image_frame_name,
                                                  "Camera")
                response = self.camera_to_plate(Xsrc,Ysrc)
                x_plate = response.Xdst[0]
                y_plate = response.Ydst[0]

                quat_plate = self.quaternion_camera_to_plate((msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w))

                if quat_plate is not None:
                    t = msg.header.stamp.to_sec()
                    (x,y,vx,vy) = kf.update((x_plate,y_plate),t)

                    if (vx is not None) and (vy is not None):
                        vel_mag,vel_ang = self.mag_angle_from_x_y(vx,vy)
                        stopped = sw.classify(vel_mag)
                        vx_filtered_data.Filtered = vx
                        vy_filtered_data.Filtered = vy
                    else:
                        vel_mag = vel_ang = stopped = None

                    if stopped:
                        stop_state.Stopped = 1
                    else:
                        stop_state.Stopped = 0

                    stop_pub.publish(stop_state)

                    x_filtered_data.Unfiltered = x_plate
                    y_filtered_data.Unfiltered = y_plate
                    x_filtered_data.UsingFiltered = 0
                    y_filtered_data.UsingFiltered = 0
                    vx_filtered_data.Unfiltered = 0
                    vy_filtered_data.Unfiltered = 0
                    vx_filtered_data.UsingFiltered = 1
                    vy_filtered_data.UsingFiltered = 1

                    if (x is not None) and (y is not None) and \
                           (abs(x_plate - x) < self.position_threshold) and \
                           (abs(y_plate - y) < self.position_threshold):
                        x_filtered_data.Filtered = x
                        y_filtered_data.Filtered = y
                        x_filtered_data.UsingFiltered = 1
                        y_filtered_data.UsingFiltered = 1
                        x_plate = x
                        y_plate = y

                    if vel_ang is not None:
                        quat_chosen = co.choose_orientation(quat_plate,vel_ang,stopped,a_prev[0])
                    else:
                        quat_chosen = None

                    if "Robot" in object_name:
                        quat_chosen = quat_plate

                    if quat_chosen is not None:
                        a_plate = CircleFunctions.mod_angle(tf.transformations.euler_from_quaternion(quat_chosen)[2])
                    else:
                        a_plate = CircleFunctions.mod_angle(tf.transformations.euler_from_quaternion(quat_plate)[2])

                    # if "Fly" in object_name:
                    #     rospy.logwarn("a_prev = %s" % (str(a_prev[0])))
                    #     rospy.logwarn("a_plate = %s" % (str(a_plate)))
                    a_plate = CircleFunctions.unwrap_angle(a_plate,a_prev[0])
                    # if "Fly" in object_name:
                    #     rospy.logwarn("a_plate_unwrapped = %s" % (str(a_plate)))


                    if a_plate is not None:
                        a_filtered_data.Unfiltered = a_plate
                    else:
                        a_filtered_data.Unfiltered = 0

                    a = lpf_angle.update(a_plate,t)

                    # if "Fly" in object_name:
                    #     rospy.logwarn("a = %s" % (str(a)))

                    a_filtered_data.UsingFiltered = 0

                    if a is not None:
                        a_filtered_data.Filtered = a
                        a_mod = CircleFunctions.mod_angle(a)
                        quat_plate = tf.transformations.quaternion_about_axis(a_mod, (0,0,1))
                        a_filtered_data.UsingFiltered = 1
                        # if abs(CircleFunctions.circle_dist(a_plate,a)) < self.angle_threshold:
                            # quat_plate = tf.transformations.quaternion_about_axis(a, (0,0,1))
                            # a_filtered_data.UsingFiltered = 1
                    else:
                        a_filtered_data.Filtered = 0

                    x_filtered_data_pub.publish(x_filtered_data)
                    y_filtered_data_pub.publish(y_filtered_data)
                    vx_filtered_data_pub.publish(vx_filtered_data)
                    vy_filtered_data_pub.publish(vy_filtered_data)
                    a_filtered_data_pub.publish(a_filtered_data)

                    if vel_ang is not None:
                        quat_chosen = co.choose_orientation(quat_plate,vel_ang,stopped,a_prev[0])
                    else:
                        quat_chosen = None

                    a_prev[0] = a

                    if "Robot" in object_name:
                        quat_chosen = quat_plate

                    if quat_chosen is not None:
                        self.tf_broadcaster.sendTransform((x_plate, y_plate, 0),
                                              quat_chosen,
                                              rospy.Time.now(),
                                              frame_name,
                                              "Plate")

                    dist = math.sqrt(x_plate**2 + y_plate**2)
                    if dist < self.in_bounds_radius:
                        in_bounds_state.InBounds = 1
                    else:
                        in_bounds_state.InBounds = 0
                    in_bounds_pub.publish(in_bounds_state)

        except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException):
        # except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException, AttributeError):
            pass


    def handle_robot_image_pose(self,msg):
        if self.initialized:
            self.image_pose_handler("Robot",msg)

    def handle_fly_image_pose(self,msg):
        if self.initialized:
            fly_count = len(msg.poses)
            # rospy.logwarn("fly count = %s" % (str(fly_count)))
            if 0 < fly_count:
                self.fly_image_pose.header = msg.header
                self.fly_image_pose.pose = msg.poses[0]
                self.image_pose_handler("Fly",self.fly_image_pose)

if __name__ == '__main__':
    rospy.init_node('robot_fly_plate_tf_broadcaster')
    ptc = PoseTFConversion()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

