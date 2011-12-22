#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('joystick_commands')
import sys
import rospy
from joystick_ps3.msg import JoystickValues
from joystick_commands.msg import JoystickCommands


class JoystickCommandPublisher:

  def __init__(self):
    self.dt = rospy.get_param("Joystick_Update_dt","0.010")
    self.rate = rospy.Rate(1/self.dt)

    self.joy_sub = rospy.Subscriber("Joystick/Values", JoystickValues, self.joy_callback)
    self.command_pub = rospy.Publisher("Joystick/Commands", JoystickCommands)
    self.command_values = JoystickCommands()
    self.output_frame = {False: "Plate", True:"Fly"}
    self.frame_bool = False
    # self.tracking_bool = False
    # self.goto_start_bool = False

  def joy_callback(self,data):
    self.command_values.x_velocity = data.x_left
    self.command_values.y_velocity = data.y_left
    self.command_values.radius_velocity = data.y_right
    self.command_values.tangent_velocity = data.x_right
    self.frame_bool = self.frame_bool ^ data.select

    # self.tracking_bool = self.tracking_bool ^ data.playstation
    # self.command_values.tracking = self.tracking_bool
    # self.goto_start_bool = self.goto_start_bool ^ data.start
    # self.command_values.goto_start = self.goto_start_bool
    self.command_values.tracking = data.playstation
    self.command_values.goto_start = data.start

    self.command_values.start = data.R2
    self.command_values.stop = data.L2
    self.command_values.yes = data.R1
    self.command_values.no = data.L1

    self.command_values.header.frame_id = self.output_frame[self.frame_bool]
    if data.triangle:
      self.command_values.radius_inc = 1
    elif data.x:
      self.command_values.radius_inc = -1
    else:
      self.command_values.radius_inc = 0
    if data.square:
      self.command_values.theta_inc = 1
    elif data.circle:
      self.command_values.theta_inc = -1
    else:
      self.command_values.theta_inc = 0

  def updater(self):
    while not rospy.is_shutdown():
      self.command_pub.publish(self.command_values)
      self.rate.sleep()


if __name__ == '__main__':
  rospy.init_node('JoystickCommands') #, anonymous=True)
  jc = JoystickCommandPublisher()
  jc.updater()
