#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('track_image_contours')
import rospy


class StopWalk:
    def __init__(self):
        self.vel_threshold_low = 1
        self.vel_threshold_high = 2.5
        self.states = {'stopped' : True, 'walking' : False}
        self.state = self.states['stopped']
        self.state_prev = self.state

    def classify(self,vel_mag):
        if (self.state == self.states['stopped']) and (self.vel_threshold_high < vel_mag):
            state = self.states['walking']
        elif (self.state == self.states['walking']) and (vel_mag < self.vel_threshold_low):
            state = self.states['stopped']
        else:
            state = self.state

        # Must be in same state for at least two frames
        if state == self.state_prev:
            self.state = state

        self.state_prev = state

        return self.state
