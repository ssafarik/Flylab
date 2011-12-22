#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flysim_teleop')
import sys
import rospy
import math
from joystick_commands.msg import JoystickCommands
from flysim.msg import Velocity,Pose

class FlyCommandPublisher:

  def __init__(self):
    self.command_sub = rospy.Subscriber("Joystick/Commands", JoystickCommands, self.joy_callback)
    self.pose_sub = rospy.Subscriber("fly/pose",Pose,self.pose_callback)
    self.command_pub = rospy.Publisher("fly/command_velocity",Velocity)
    self.command_velocity = Velocity()
    self.fly_theta = 0
    self.scale_linear = rospy.get_param("scale_linear")
    self.scale_angular = rospy.get_param("scale_angular")

  def pose_callback(self,msg):
    self.fly_theta = msg.theta

  def joy_callback(self,msg):
    self.command_velocity.linear = msg.radius_velocity*self.scale_linear
    self.command_velocity.angular = -msg.tangent_velocity*self.scale_angular
    self.command_pub.publish(self.command_velocity)

if __name__ == '__main__':
    rospy.init_node('FlyCommands', anonymous=True)
    fc = FlyCommandPublisher()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
