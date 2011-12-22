#!/usr/bin/env python
from __future__ import division
import numpy as N

pi = N.pi

def mod_angle(angle):
    if angle is not None:
        angle = angle%(2*pi)
    return angle

def unwrap_angle(angle,angle_prev):
    if (angle is not None) and (angle_prev is not None):
        delta_angle = circle_dist(angle_prev,angle)
        unwrapped_angle = angle_prev + delta_angle
    elif angle is not None:
        unwrapped_angle = angle
    return unwrapped_angle


# circle_dist()
# Get the distance between two angles.  Cannot be further than pi apart.
#
def circle_dist(angle1, angle2):
    angle1 = angle1 % (2.0*N.pi)
    angle2 = angle2 % (2.0*N.pi)
    dist = abs(angle1 - angle2)
    if dist>N.pi:
        dist = (2.0*N.pi) - dist
    
    return dist
    
def circle_dist_OLD(start_angle, stop_angle):
    if (start_angle is not None) and (stop_angle is not None):
        start_angle = mod_angle(start_angle)
        stop_angle = mod_angle(stop_angle)
        diff1 = stop_angle - start_angle
        if start_angle < stop_angle:
            diff2 = stop_angle - 2*N.pi - start_angle
        else:
            diff2 = stop_angle + 2*N.pi - start_angle
            
        abs_min = min(abs(diff1),abs(diff2))
        if abs_min == abs(diff1):
            return diff1
        else:
            return diff2

def radians_to_degrees(angle):
    if angle is not None:
        return angle*180/pi

def degrees_to_radians(angle):
    if angle is not None:
        return angle*pi/180

if __name__ == '__main__':
    pi = N.pi
    start_angle = [pi/4, pi/4, 7/4*pi, 7/4*pi]
    stop_angle = [0, 7/4*pi, 0, pi/4]
    for ang in range(len(start_angle)):
        start_ang = start_angle[ang]
        stop_ang = stop_angle[ang]
        diff = circle_dist(start_ang,stop_ang)
        print "start_angle = %s" % (str(radians_to_degrees(start_ang)))
        print "stop_angle = %s" % (str(radians_to_degrees(stop_ang)))
        print "diff = %s" % (str(radians_to_degrees(diff)))

        ang = stop_ang
        ang_prev = start_ang
        unwrapped_ang = unwrap_angle(ang,ang_prev)
        print "ang_prev = %s" % (str(radians_to_degrees(ang_prev)))
        print "ang = %s" % (str(radians_to_degrees(ang)))
        print "unwrapped_ang = %s" % (str(radians_to_degrees(unwrapped_ang)))
