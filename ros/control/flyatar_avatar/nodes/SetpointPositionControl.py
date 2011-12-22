#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flyatar_avatar')
import sys
import rospy
import math
import tf
import cv
import numpy
import copy
from plate_tf.srv import *
from flystage.srv import *
from flystage.msg import StageCommands,Setpoint
from joystick_commands.msg import JoystickCommands
from geometry_msgs.msg import PointStamped
from pythonmodules import CircleFunctions


class SetpointControl:

    def __init__(self):
        self.initialized = False
        self.control_dt = rospy.get_param("Control_Update_dt")

        self.tf_listener = tf.TransformListener()

        self.joy_sub = rospy.Subscriber("Joystick/Commands", JoystickCommands, self.joystick_commands_callback)
        self.sc_pub = rospy.Publisher("StageCommands",StageCommands)
        self.setpoint_pub = rospy.Publisher("setpoint",Setpoint)

        self.sc_ok_to_publish = False

        self.stage_commands = StageCommands()
        self.stage_commands.position_control = False
        self.robot_velocity_max = rospy.get_param("robot_velocity_max",100) # mm/s
        self.vel_vector_plate = numpy.array([[0],[0],[0],[1]])
        self.vel_vector_robot = numpy.array([[0],[0],[0],[1]])

        self.control_frame = "Plate"
        self.start_frame = "Plate"

        self.setpoint = Setpoint()
        self.setpoint.header.frame_id = self.control_frame
        self.setpoint.radius = 20
        self.setpoint.theta = 0
        self.setpoint_int = Setpoint()
        self.setpoint_int.header.frame_id = self.control_frame
        self.setpoint_int.radius = 20
        self.setpoint_int.theta = 0
        self.inc_radius = 1
        self.inc_theta = 0.05
        self.setpoint_radius_max = 80
        self.setpoint_radius_min = 0


        self.dummy_point = PointStamped()
        self.dummy_point.header.frame_id = "Plate"
        self.dummy_point.point.x = 0
        self.dummy_point.point.y = 0
        self.dummy_point.point.z = 0

        self.robot_origin = PointStamped()
        self.robot_origin.header.frame_id = "Robot"
        self.robot_origin.point.x = 0
        self.robot_origin.point.y = 0
        self.robot_origin.point.z = 0
        self.robot_plate = PointStamped()
        self.robot_plate.header.frame_id = "Plate"
        self.robot_control_frame = PointStamped()
        self.robot_control_frame.header.frame_id = self.control_frame
        self.setpoint_center_origin = PointStamped()
        self.setpoint_center_origin.header.frame_id = self.control_frame
        self.setpoint_center_origin.point.x = 0
        self.setpoint_center_origin.point.y = 0
        self.setpoint_center_origin.point.z = 0
        self.setpoint_center_plate = PointStamped()
        self.setpoint_center_plate.header.frame_id = "Plate"
        self.setpoint_origin = PointStamped()
        self.setpoint_origin.header.frame_id = self.control_frame
        self.setpoint_origin.point.x = self.setpoint.radius*math.cos(self.setpoint.theta)
        self.setpoint_origin.point.y = self.setpoint.radius*math.sin(self.setpoint.theta)
        self.setpoint_origin.point.z = 0
        self.setpoint_plate = PointStamped()
        self.setpoint_plate.header.frame_id = "Plate"
        self.setpoint_plate.point.x = 0
        self.setpoint_plate.point.y = 0
        self.setpoint_plate.point.z = 0
        self.setpoint_int_origin = PointStamped()
        self.setpoint_int_origin.header.frame_id = self.control_frame
        self.setpoint_int_origin.point.x = self.setpoint_int.radius*math.cos(self.setpoint_int.theta)
        self.setpoint_int_origin.point.y = self.setpoint_int.radius*math.sin(self.setpoint_int.theta)
        self.setpoint_int_origin.point.z = 0
        self.setpoint_int_plate = PointStamped()
        self.setpoint_int_plate.header.frame_id = "Plate"

        self.setpoint_plate_previous = copy.deepcopy(self.setpoint_plate)

        self.tries_limit = 4

        self.chord_length = 1
        self.point_count_max = 100
        self.plate_points_x = []
        self.plate_points_y = []

        self.setpoint_move_threshold = 0.75   # mm

        # self.setpoint_plate_initialized = False
        # while not self.setpoint_plate_initialized:
        #     try:
        #         self.setpoint_plate = self.tf_listener.transformPoint("Stage",self.setpoint_plate)
        #         self.setpoint_plate_initialized = True
        #     except (tf.LookupException, tf.ConnectivityException):
        #         pass

        self.moving_to_start = False
        self.moving_to_setpoint = False
        self.setpoint_moved = False
        self.goto_start = False

        self.rate = rospy.Rate(1/self.control_dt)
        self.gain_radius = rospy.get_param("gain_radius","4")
        self.gain_theta = rospy.get_param("gain_theta","40")

        self.radius_velocity = 0
        self.tangent_velocity = 0
        self.tracking = False

        rospy.wait_for_service('plate_to_stage')
        try:
            self.plate_to_stage = rospy.ServiceProxy('plate_to_stage', PlateCameraConversion)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # Xsrc = [0,10,15.4,-34.1]
        # Ysrc = [0,10,-15.3,-65.9]
        # rospy.logwarn("self.plate_points_x = %s" % (str(Xsrc)))
        # rospy.logwarn("self.plate_points_y = %s" % (str(Ysrc)))
        # response = self.plate_to_stage(Xsrc,Ysrc)
        # rospy.logwarn("stage_points_x = %s" % (str(response.Xdst)))
        # rospy.logwarn("stage_points_y = %s" % (str(response.Ydst)))

        self.initialized = True

    def update_setpoint_moved(self):
        # rospy.logwarn("setpoint_plate_previous.point.x = %s" % (str(self.setpoint_plate_previous.point.x)))
        # rospy.logwarn("setpoint_plate.point.x = %s" % (str(self.setpoint_plate.point.x)))
        # rospy.logwarn("setpoint_plate_previous.point.y = %s" % (str(self.setpoint_plate_previous.point.y)))
        # rospy.logwarn("setpoint_plate.point.y = %s" % (str(self.setpoint_plate.point.y)))
        # rospy.logwarn("abs(self.setpoint_plate_previous.point.x - self.setpoint_plate.point.x) = %s" % (str(abs(self.setpoint_plate_previous.point.x - self.setpoint_plate.point.x))))
        # rospy.logwarn("abs(self.setpoint_plate_previous.point.y - self.setpoint_plate.point.y) = %s" % (str(abs(self.setpoint_plate_previous.point.y - self.setpoint_plate.point.y))))
        if self.setpoint_move_threshold < abs(self.setpoint_plate_previous.point.x - self.setpoint_plate.point.x) or \
           self.setpoint_move_threshold < abs(self.setpoint_plate_previous.point.y - self.setpoint_plate.point.y):
            self.setpoint_moved = True
            self.setpoint_plate_previous = copy.deepcopy(self.setpoint_plate)
        # else:
        #     self.setpoint_moved = False
        # rospy.logwarn("setpoint_moved = %s" % (str(self.setpoint_moved)))

    def angle_divide(self,angle_start,angle_stop):
        diff = CircleFunctions.circle_dist(angle_start,angle_stop)
        if 0 < diff:
            if angle_stop < angle_start:
                angle_start = angle_start - 2*math.pi
        else:
            if angle_start < angle_stop:
                angle_stop = angle_stop - 2*math.pi
        # rospy.logwarn("angle_start = \n%s" % (str(angle_start)))
        # rospy.logwarn("angle_stop = \n%s" % (str(angle_stop)))
        # rospy.logwarn("diff = \n%s" % (str(diff)))
        r = self.setpoint.radius
        if 0 < r:
            angle_inc = self.chord_length/r
            point_count = math.ceil(abs(diff)/angle_inc)
            if self.point_count_max < point_count:
                point_count = self.point_count_max
            angle_list = list(numpy.linspace(angle_start,angle_stop,num=point_count,endpoint=True))
        else:
            angle_list = []
        # rospy.logwarn("angle_list = \n%s" % (str(angle_list)))
        return angle_list

    def vel_vector_convert(self,vel_vector,frame):
        (trans,q) = self.tf_listener.lookupTransform("Stage",frame,rospy.Time(0))
        rot_matrix = tf.transformations.quaternion_matrix(q)
        vel_vector_stage = numpy.dot(rot_matrix,vel_vector)
        x_vel_stage = vel_vector_stage[0]
        y_vel_stage = vel_vector_stage[1]
        return [x_vel_stage],[y_vel_stage]

    def convert_to_plate(self,point):
        point_converted = False
        tries = 0
        point_plate = copy.deepcopy(self.dummy_point)
        self.dummy_point.header.frame_id = "Plate"
        while (not point_converted) and (tries < self.tries_limit):
            tries += 1
            try:
                point_plate = self.tf_listener.transformPoint("Plate",point)
                point_converted = True
            except (tf.LookupException, tf.ConnectivityException):
                pass
        return point_plate

    def convert_to_control_frame(self,point):
        point_converted = False
        tries = 0
        point_control_frame = copy.deepcopy(self.dummy_point)
        self.dummy_point.header.frame_id = self.control_frame
        while (not point_converted) and (tries < self.tries_limit):
            tries += 1
            try:
                point_control_frame = self.tf_listener.transformPoint(self.control_frame,point)
                point_converted = True
            except (tf.LookupException, tf.ConnectivityException):
                pass
        return point_control_frame

    # def find_robot_plate(self):
    #     robot_plate_acquired = False
    #     while not robot_plate_acquired:
    #         try:
    #             self.robot_plate = self.tf_listener.transformPoint("Plate",self.robot_origin)
    #             robot_plate_acquired = True
    #         except (tf.LookupException, tf.ConnectivityException):
    #             pass

    # def find_setpoint_center_plate(self):
    #     setpoint_center_plate_acquired = False
    #     while not setpoint_center_plate_acquired:
    #         try:
    #             self.setpoint_center_plate = self.tf_listener.transformPoint("Plate",self.setpoint_center_origin)
    #             setpoint_center_plate_acquired = True
    #         except (tf.LookupException, tf.ConnectivityException):
    #             pass

    # def find_setpoint_plate(self):
    #     # rospy.logwarn("self.setpoint_origin.header.frame_id = %s" % (str(self.setpoint_origin.header.frame_id)))
    #     # rospy.logwarn("self.setpoint_origin.x = %s" % (str(self.setpoint_origin.x)))
    #     # rospy.logwarn("self.setpoint_origin.y = %s" % (str(self.setpoint_origin.y)))
    #     setpoint_plate_acquired = False
    #     while not setpoint_plate_acquired:
    #         try:
    #             self.setpoint_plate = self.tf_listener.transformPoint("Plate",self.setpoint_origin)
    #             # rospy.logwarn("self.setpoint_plate.x = %s" % (str(self.setpoint_plate.x)))
    #             # rospy.logwarn("self.setpoint_plate.y = %s" % (str(self.setpoint_plate.y)))
    #             setpoint_plate_acquired = True
    #         except (tf.LookupException, tf.ConnectivityException):
    #             pass

    def find_velocity_from_position(self,pos_x,pos_y,vel_mag):
        point_count = min(len(pos_x),len(pos_y))
        vel_x = []
        vel_y = []
        vel_mag = abs(vel_mag)          # Just to be sure
        for point_n in range(point_count):
            if point_n == 0:
                vel_x.append(math.sqrt(2)*vel_mag)
                vel_y.append(math.sqrt(2)*vel_mag)
            else:
                # rospy.logwarn("pos_x[point_n] = %s" % (str(pos_x[point_n])))
                # rospy.logwarn("pos_x[point_n - 1] = %s" % (str(pos_x[point_n - 1])))
                # rospy.logwarn("pos_y[point_n] = %s" % (str(pos_y[point_n])))
                # rospy.logwarn("pos_y[point_n - 1] = %s" % (str(pos_y[point_n -1])))
                delta_x = abs(pos_x[point_n] - pos_x[point_n - 1])
                delta_y = abs(pos_y[point_n] - pos_y[point_n - 1])
                # try:
                alpha = math.sqrt((vel_mag**2)/(delta_x**2 + delta_y**2))
                # except:
                #     alpha = 1
                vel_x.append(alpha*delta_x)
                vel_y.append(alpha*delta_y)
        return vel_x,vel_y

    def set_stage_commands_from_plate_points(self,vel_mag):
        response = self.plate_to_stage(self.plate_points_x,self.plate_points_y)
        # rospy.logwarn("self.plate_points_x = %s" % (str(self.plate_points_x)))
        # rospy.logwarn("self.plate_points_y = %s" % (str(self.plate_points_y)))
        stage_points_x = response.Xdst
        stage_points_y = response.Ydst
        # rospy.logwarn("stage_points_x = %s" % (str(stage_points_x)))
        # rospy.logwarn("stage_points_y = %s" % (str(stage_points_y)))
        stage_velocity_x,stage_velocity_y = self.find_velocity_from_position(stage_points_x,stage_points_y,vel_mag)
        self.stage_commands.x_position = stage_points_x
        self.stage_commands.y_position = stage_points_y
        self.stage_commands.x_velocity = stage_velocity_x
        self.stage_commands.y_velocity = stage_velocity_y

        # rospy.logwarn("self.plate_points_x = %s" % (str(self.plate_points_x)))
        # rospy.logwarn("self.plate_points_y = %s" % (str(self.plate_points_y)))
        # rospy.logwarn("stage_points_x = %s" % (str(stage_points_x)))
        # rospy.logwarn("stage_points_y = %s" % (str(stage_points_y)))

    def set_path_to_start(self,vel_mag):
        self.robot_plate = self.convert_to_plate(self.robot_origin)
        self.plate_points_x = [self.robot_plate.point.x,0]
        self.plate_points_y = [self.robot_plate.point.y,0]
        self.set_stage_commands_from_plate_points(vel_mag)
        # self.set_position_velocity_point(0,0,self.start_frame,vel_mag)

    def append_int_setpoint_to_plate_points(self,setpoint_angle):
        self.setpoint_int.theta = setpoint_angle
        self.setpoint_int_origin.point.x = self.setpoint_int.radius*math.cos(self.setpoint_int.theta)
        self.setpoint_int_origin.point.y = self.setpoint_int.radius*math.sin(self.setpoint_int.theta)
        self.setpoint_int_plate = self.convert_to_plate(self.setpoint_int_origin)
        xi = self.setpoint_int_plate.point.x
        yi = self.setpoint_int_plate.point.y
        self.plate_points_x.append(xi)
        self.plate_points_y.append(yi)

    def set_path_to_setpoint(self,vel_mag):
        self.robot_control_frame = self.convert_to_control_frame(self.robot_origin)
        self.robot_plate = self.convert_to_plate(self.robot_origin)
        # self.setpoint_center_plate = self.convert_to_plate(self.setpoint_center_origin)
        # self.find_setpoint_center_plate()
        x_rp = self.robot_plate.point.x
        y_rp = self.robot_plate.point.y
        self.plate_points_x = [x_rp]
        self.plate_points_y = [y_rp]
        # x_scp =  self.setpoint_center_plate.point.x
        # y_scp =  self.setpoint_center_plate.point.y
        # dx = x_ro - x_so
        # dy = y_ro - y_so
        dx = self.robot_control_frame.point.x
        dy = self.robot_control_frame.point.y
        # self.append_int_setpoint_to_plate_points(math.atan2(dy,dx))
        # self.setpoint_int.theta = math.atan2(dy,dx)
        # self.setpoint_int_origin.point.x = self.setpoint_int.radius*math.cos(self.setpoint_int.theta)
        # self.setpoint_int_origin.point.y = self.setpoint_int.radius*math.sin(self.setpoint_int.theta)
        # self.setpoint_int_plate = self.convert_to_plate(self.setpoint_int_origin)
        # xi = self.setpoint_int_plate.point.x
        # yi = self.setpoint_int_plate.point.y
        start_theta = math.atan2(dy,dx)

        # angle_list = self.angle_divide(self.setpoint_int.theta,self.setpoint.theta)
        angle_list = self.angle_divide(start_theta,self.setpoint.theta)
        # rospy.logwarn("angle_list = %s" % (str(angle_list)))
        # rospy.logwarn("len(angle_list) = %s" % (str(len(angle_list))))
        for angle_n in range(len(angle_list)):
            self.append_int_setpoint_to_plate_points(angle_list[angle_n])
            # self.setpoint_int.theta = angle_list(angle_n)
            # self.setpoint_int_origin.point.x = self.setpoint_int.radius*math.cos(self.setpoint_int.theta)
            # self.setpoint_int_origin.point.y = self.setpoint_int.radius*math.sin(self.setpoint_int.theta)
            # self.setpoint_int_plate = self.convert_to_plate(self.setpoint_int_origin)
            # xi = self.setpoint_int_plate.point.x
            # yi = self.setpoint_int_plate.point.y
            # self.plate_points_x.append(xi)
            # self.plate_points_y.append(yi)
        # rospy.logwarn("plate_points_y = %s" % (str(self.plate_points_y)))

        # rospy.logwarn("xi = %s, yi = %s" % (str(xi),str(yi)))
        # dr = math.sqrt(dx**2 + dy**2)
        # dx_norm = dx/dr
        # dy_norm = dy/dr
        # xi = x_so + dx_norm*self.setpoint.radius
        # yi = y_so + dy_norm*self.setpoint.radius
        # rospy.logwarn("xi = %s, yi = %s" % (str(xi),str(yi)))

        # self.setpoint_plate = self.convert_to_plate(self.setpoint_origin)
        # self.find_setpoint_plate()
        # self.plate_points_x.append(self.setpoint_plate.point.x)
        # self.plate_points_y.append(self.setpoint_plate.point.y)
        self.set_stage_commands_from_plate_points(vel_mag)
        # rospy.logwarn("self.stage_commands.x_position = %s" % (str(self.stage_commands.x_position)))
        # rospy.logwarn("self.stage_commands.y_position = %s" % (str(self.stage_commands.y_position)))
        # rospy.logwarn("self.stage_commands.x_velocity = %s" % (str(self.stage_commands.x_velocity)))
        # rospy.logwarn("self.stage_commands.y_velocity = %s" % (str(self.stage_commands.y_velocity)))

    # def set_position_velocity_point(self,x_target,y_target,frame_target,vel_mag):
    #     self.setpoint_plate.header.frame_id = frame_target
    #     self.setpoint_plate.point.x = x_target
    #     self.setpoint_plate.point.y = y_target

    #     target_acquired = False
    #     while not target_acquired:
    #         try:
    #             setpoint_plate_stage = self.tf_listener.transformPoint("Stage",self.setpoint_plate)
    #             target_acquired = True
    #         except (tf.LookupException, tf.ConnectivityException):
    #             rospy.logwarn("Error finding setpoint_plate_stage!")

    #     self.stage_commands.x_position = [robot_plate_stage.point.x,setpoint_plate_stage.point.x]
    #     self.stage_commands.y_position = [robot_plate_stage.point.y,setpoint_plate_stage.point.y]

    #     delta_x = abs(setpoint_plate_stage.point.x - robot_plate_stage.point.x)
    #     delta_y = abs(setpoint_plate_stage.point.y - robot_plate_stage.point.y)
    #     alpha = math.sqrt((vel_mag**2)/(delta_x**2 + delta_y**2))
    #     self.stage_commands.x_velocity = [vel_mag,alpha*delta_x]
    #     self.stage_commands.y_velocity = [vel_mag,alpha*delta_y]

    def joystick_commands_callback(self,data):
        if self.initialized:
            self.control_frame = data.header.frame_id
            self.setpoint.header.frame_id = self.control_frame
            self.setpoint_center_origin.header.frame_id = self.control_frame
            self.setpoint_origin.header.frame_id = self.control_frame
            self.setpoint_int_origin.header.frame_id = self.control_frame
            self.setpoint.radius += self.inc_radius*data.radius_inc
            if self.setpoint.radius < self.setpoint_radius_min:
                self.setpoint.radius = self.setpoint_radius_min
            elif self.setpoint_radius_max < self.setpoint.radius:
                self.setpoint.radius = self.setpoint_radius_max
            self.setpoint.theta += self.inc_theta*data.theta_inc
            self.setpoint.theta = math.fmod(self.setpoint.theta,2*math.pi)
            if self.setpoint.theta < 0:
                self.setpoint.theta = 2*math.pi - self.setpoint.theta
            if data.tracking and (not self.tracking):
                self.tracking = True
            if data.goto_start and (not self.goto_start):
                self.goto_start = True
                self.tracking = False
            if data.stop:
                self.goto_start = False
                self.tracking = False
            self.setpoint_pub.publish(self.setpoint)
            self.setpoint_int.radius = self.setpoint.radius
            self.setpoint_origin.point.x = self.setpoint.radius*math.cos(self.setpoint.theta)
            self.setpoint_origin.point.y = self.setpoint.radius*math.sin(self.setpoint.theta)
            self.setpoint_plate = self.convert_to_plate(self.setpoint_origin)
            self.update_setpoint_moved()
            # rospy.logwarn("setpoint_origin.point.x = %s" % (str(self.setpoint_origin.point.x)))
            # rospy.logwarn("setpoint_origin.point.y = %s" % (str(self.setpoint_origin.point.y)))

            if not self.tracking:
                if self.goto_start:
                    if not self.moving_to_start:
                        self.moving_to_start = True
                        self.stage_commands.position_control = True
                        self.set_path_to_start(self.robot_velocity_max)
                        self.sc_ok_to_publish = True
                        self.goto_start = False
                    else:
                        self.sc_ok_to_publish = False
                else:
                    self.sc_ok_to_publish = True
                    if self.moving_to_start:
                        self.moving_to_start = False
                    self.stage_commands.position_control = False
                    self.radius_velocity = data.radius_velocity*self.robot_velocity_max
                    self.tangent_velocity = data.tangent_velocity*self.robot_velocity_max
                    self.vel_vector_plate[0,0] = data.x_velocity*self.robot_velocity_max
                    self.vel_vector_plate[1,0] = data.y_velocity*self.robot_velocity_max

                    try:
                        (self.stage_commands.x_velocity,self.stage_commands.y_velocity) = self.vel_vector_convert(self.vel_vector_plate,"Plate")
                    except (tf.LookupException, tf.ConnectivityException):
                        self.stage_commands.x_velocity = [self.vel_vector_plate[0,0]]
                        self.stage_commands.y_velocity = [-self.vel_vector_plate[1,0]]

                if self.sc_ok_to_publish:
                    self.sc_pub.publish(self.stage_commands)

    def control_loop(self):
        while not rospy.is_shutdown():
            if self.tracking:
                # if not self.moving_to_setpoint:
                if (not self.moving_to_setpoint) or self.setpoint_moved:
                    self.setpoint_moved = False
                    self.moving_to_setpoint = True
                    self.stage_commands.position_control = True
                    self.set_path_to_setpoint(self.robot_velocity_max/2)
                    self.sc_ok_to_publish = True
                else:
                    self.sc_ok_to_publish = False

                if self.sc_ok_to_publish:
                    self.sc_pub.publish(self.stage_commands)
                # self.stage_commands.position_control = True
                # self.set_path_to_setpoint(self.robot_velocity_max/4)
                # self.sc_pub.publish(self.stage_commands)
            else:
                self.moving_to_setpoint = False

                # try:
                    # self.gain_radius = rospy.get_param("gain_radius")
                    # self.gain_theta = rospy.get_param("gain_theta")

                    # (self.stage_commands.x_velocity,self.stage_commands.y_velocity) = self.vel_vector_convert(self.vel_vector_plate,"Plate")

                    # (trans,q) = self.tf_listener.lookupTransform(self.control_frame,'/Robot',rospy.Time(0))
                    # x = trans[0]
                    # y = trans[1]
                    # theta = math.atan2(y,x)
                    # radius = math.sqrt(x**2 + y**2)

                    # # rospy.logwarn("self.setpoint.radius = \n%s",str(self.setpoint.radius))
                    # # rospy.logwarn("radius = \n%s",str(radius))
                    # # rospy.logwarn("self.radius_velocity = \n%s",str(self.radius_velocity))

                    # self.gain_radius = self.gain_radius + (6/math.pi)*abs(self.circle_dist(self.setpoint.theta,theta))
                    # self.radius_velocity = self.gain_radius*(self.setpoint.radius - radius)
                    # # rospy.logwarn("self.gain_radius = \n%s",str(self.gain_radius))
                    # self.tangent_velocity = self.gain_theta*self.circle_dist(self.setpoint.theta,theta)

                    # rot_matrix = tf.transformations.rotation_matrix(theta, (0,0,1))
                    # self.vel_vector_robot[0,0] = self.radius_velocity
                    # self.vel_vector_robot[1,0] = self.tangent_velocity
                    # vel_vector_plate = numpy.dot(rot_matrix,self.vel_vector_robot)
                    # self.vel_vector_plate[0,0] = vel_vector_plate[0]
                    # self.vel_vector_plate[1,0] = vel_vector_plate[1]

                    # (x_vel_stage,y_vel_stage) = self.vel_vector_convert(self.vel_vector_plate,"Plate")
                    # self.stage_commands.x_velocity += x_vel_stage
                    # self.stage_commands.y_velocity += y_vel_stage

                #     self.sc_pub.publish(self.stage_commands)
                # except (tf.LookupException, tf.ConnectivityException):
                #     pass

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('SetpointPositionControl')
    sc = SetpointControl()
    sc.control_loop()
