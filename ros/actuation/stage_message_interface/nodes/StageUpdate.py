#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('stage_message_interface')
import rospy
import tf
from stage_message_interface.msg import StageCommands,StageState
from flystage.srv import *

class StageUpdate:

  def __init__(self):
    self.initialized = False
    self.dt = rospy.get_param("Stage_Update_dt","0.010")

    self.rate = rospy.Rate(1/self.dt)

    self.tfbx = tf.TransformBroadcaster()

    self.stage_commands = Stage_StateRequest()
    self.update_position = False
    self.update_velocity = False

    self.update_lock = False

    self.sc_sub = rospy.Subscriber("Stage/Commands", StageCommands, self.stage_commands_callback)
    self.ss_pub = rospy.Publisher("Stage/State",StageState)
    self.ss = StageState()

    rospy.wait_for_service('get_stage_state')
    try:
      self.get_stage_state = rospy.ServiceProxy('get_stage_state', Stage_State)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

    rospy.wait_for_service('set_stage_velocity')
    try:
      self.set_stage_velocity = rospy.ServiceProxy('set_stage_velocity', Stage_State)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

    rospy.wait_for_service('set_stage_position')
    try:
      self.set_stage_position = rospy.ServiceProxy('set_stage_position', Stage_State)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

    self.initialized = True

  def stage_commands_callback(self,data):
    if self.initialized and not self.update_lock:
      position_list_length = min(len(data.x_position),len(data.y_position))
      velocity_list_length = min(len(data.x_velocity),len(data.y_velocity))
      if (0 < position_list_length):
        self.update_position = True
        self.update_velocity = False
      elif (0 < velocity_list_length):
        self.update_position = False
        self.update_velocity = True
      else:
        self.update_position = False
        self.update_velocity = False

      self.stage_commands.x_position = data.x_position
      self.stage_commands.y_position = data.y_position
      self.stage_commands.x_velocity = data.x_velocity
      self.stage_commands.y_velocity = data.y_velocity
      self.stage_commands.speed = data.speed

  def updater(self):
    while not rospy.is_shutdown():
      if self.initialized:
        try:
          self.update_lock = True
          up = self.update_position
          uv = self.update_velocity
          self.update_position = False
          self.update_velocity = False
          self.update_lock = False

          if up:
            response = self.set_stage_position(self.stage_commands)
            # rospy.logwarn("set_stage_position()")
          elif uv:
            response = self.set_stage_velocity(self.stage_commands)
            # rospy.logwarn("set_stage_velocity()")
          else:
            response = self.get_stage_state()
            # rospy.logwarn("get_stage_state()")

          x = response.x
          y = response.y
          self.ss.bAllJointsInPosition = response.bAllJointsInPosition
          self.ss.bMoveInProgress = response.bMoveInProgress

          self.ss_pub.publish(self.ss)
          self.tfbx.sendTransform((x, y, 0),
                                            tf.transformations.quaternion_from_euler(0, 0, 0),
                                            rospy.Time.now(),
                                            "Magnet",
                                            "Stage")
        except (tf.LookupException, tf.ConnectivityException, rospy.service.ServiceException):
          pass

        self.rate.sleep()

if __name__ == '__main__':
  rospy.init_node('StageUpdate') #, anonymous=True)
  su = StageUpdate()
  su.updater()
