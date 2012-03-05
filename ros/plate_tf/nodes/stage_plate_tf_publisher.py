#!/usr/bin/env python
import roslib
roslib.load_manifest('plate_tf')
import rospy

import tf
# import numpy, math
from geometry_msgs.msg import PointStamped
# from plate_tf.srv import *
import filters
import copy
from plate_tf.msg import StopState

class StagePlateTFPublisher:
    def __init__(self):
        self.bInitialized = False
        self.tfrx = tf.TransformListener()
        self.tfbx = tf.TransformBroadcaster()

        self.robot_stop_state_sub = rospy.Subscriber('StopState/Robot', StopState, self.cbRobotStopState)
        self.robot_stop_state = StopState()

        self.dt = 0.01
        self.rate = rospy.Rate(1/self.dt)

        self.stage_plate_offset_x = rospy.get_param("stage_plate_offset_x")
        self.stage_plate_offset_y = rospy.get_param("stage_plate_offset_y")
        self.stage_plate_quat_z = rospy.get_param("stage_plate_quat_z")
        self.stage_plate_quat_w = rospy.get_param("stage_plate_quat_w")

        self.stage_plate_offset_x_error = 0
        self.stage_plate_offset_y_error = 0

        self.robot_origin = PointStamped()
        self.robot_origin.header.frame_id = "Robot"
        self.robot_origin.point.x = 0
        self.robot_origin.point.y = 0
        self.robot_origin.point.z = 0
        self.endeffector_origin = PointStamped()
        self.endeffector_origin.header.frame_id = "EndEffector"
        self.endeffector_origin.point.x = 0
        self.endeffector_origin.point.y = 0
        self.endeffector_origin.point.z = 0
        self.dummy_point = PointStamped()
        self.dummy_point.header.frame_id = "Plate"
        self.dummy_point.point.x = 0
        self.dummy_point.point.y = 0
        self.dummy_point.point.z = 0

        self.position_threshold = 0.01

        self.adjusted = False

        self.tries_limit = 4

        self.kf_stage_plate_offset = filters.KalmanFilter()
        self.bInitialized = True


    def convert_to_plate(self,point):
        bConverted = False
        nTries = 0
        point_plate = copy.deepcopy(self.dummy_point)
        self.dummy_point.header.frame_id = "Plate"

        while (not bConverted) and (nTries < self.tries_limit):
            nTries += 1
            try:
                point_plate = self.tfrx.transformPoint("Plate", point)
                bConverted = True
            except (tf.LookupException, tf.ConnectivityException):
                pass
            
        return point_plate


    def cbRobotStopState(self,data):
        self.robot_stop_state = copy.deepcopy(data)


    def broadcast(self):
        if self.bInitialized:
            # try:
            if self.robot_stop_state.Stopped and (not self.adjusted):
                robot_plate = self.convert_to_plate(self.robot_origin)
                endeffector_plate = self.convert_to_plate(self.endeffector_origin)
                # rospy.logwarn("robot_plate.point.x = \n%s" % (str(robot_plate.point.x)))
                # rospy.logwarn("endeffector_plate.point.x = \n%s" % (str(endeffector_plate.point.x)))
                # rospy.logwarn("robot_plate.point.y = \n%s" % (str(robot_plate.point.y)))
                # rospy.logwarn("endeffector_plate.point.y = \n%s" % (str(endeffector_plate.point.y)))
                if (abs(endeffector_plate.point.x) < self.position_threshold) and \
                   (abs(endeffector_plate.point.y) < self.position_threshold):
                    self.stage_plate_offset_x_error = endeffector_plate.point.x - robot_plate.point.x
                    self.stage_plate_offset_y_error = endeffector_plate.point.y - robot_plate.point.y
                    # rospy.logwarn("self.stage_plate_offset_x_error = \n%s" % (str(self.stage_plate_offset_x_error)))
                    # rospy.logwarn("self.stage_plate_offset_y_error = \n%s" % (str(self.stage_plate_offset_y_error)))
    
                    stage_plate_offset_x_adjusted = self.stage_plate_offset_x + self.stage_plate_offset_x_error
                    stage_plate_offset_y_adjusted = self.stage_plate_offset_y + self.stage_plate_offset_y_error
                    # rospy.logwarn("stage_plate_offset_x_adjusted = \n%s" % (str(stage_plate_offset_x_adjusted)))
                    # rospy.logwarn("stage_plate_offset_y_adjusted = \n%s" % (str(stage_plate_offset_y_adjusted)))
    
                    # t = rospy.get_time()
                    # (x,y,vx,vy) = self.kf_stage_plate_offset.update((stage_plate_offset_x_adjusted,stage_plate_offset_y_adjusted),t)
                    # rospy.logwarn("x = \n%s" % (str(x)))
                    # rospy.logwarn("y = \n%s" % (str(y)))
                    self.adjusted = True
                    # self.stage_plate_offset_x = stage_plate_offset_x_adjusted
                    # self.stage_plate_offset_y = stage_plate_offset_y_adjusted
            elif (not self.robot_stop_state.Stopped) and self.adjusted:
                self.adjusted = False
    
            #rospy.loginfo ('self.stage_plate_offset_x,y=' % (self.stage_plate_offset_x,self.stage_plate_offset_y)
            self.tfbx.sendTransform((self.stage_plate_offset_x, self.stage_plate_offset_y, 0),
                                              (0, 0, self.stage_plate_quat_z, self.stage_plate_quat_w),
                                              rospy.Time.now(),
                                              "Stage",
                                              "Plate")
            # except (tf.LookupException, tf.ConnectivityException, rospy.service.ServiceException):
            #     pass


    def mainloop(self):
        while not rospy.is_shutdown():
            broadcast()
            self.rate.sleep()
            

if __name__ == '__main__':
    rospy.init_node('stage_plate_tf_publisher')
    sptb = StagePlateTFPublisher()
    sptb.mainloop()
