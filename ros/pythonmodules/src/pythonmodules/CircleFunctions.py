#!/usr/bin/env python
from __future__ import division
import numpy as N



def mod_angle(angle):
    if angle is not None:
        angle = angle%(2.0*N.pi)
    return angle


def unwrap_angle(angle, angle_prev):
    if (angle is not None) and (angle_prev is not None):
        delta_angle = circle_dist(angle_prev,angle)
        unwrapped_angle = angle_prev + delta_angle
    elif angle is not None:
        unwrapped_angle = angle
    return unwrapped_angle



# Rerange()
#   Put the angle into the range of -pi to +pi.
#
def Rerange(self, angle):
    angleOut = ((angle+N.pi) % (2.0*N.pi)) - N.pi
    
    return angleOut
    
    

# circle_dist()
# Get the distance between two angles.  Cannot be further than pi apart.
#
def circle_dist_OLD(angle1, angle2):
    angle1 = angle1 % (2.0*N.pi)
    angle2 = angle2 % (2.0*N.pi)
    dist = abs(angle1 - angle2)
    if dist>N.pi:
        dist = (2.0*N.pi) - dist
    
    return dist


# Compute the distance from one angle to another.
def circle_dist(angle1, angle2):
    angle1 = angle1 % (2.0*N.pi)
    angle2 = angle2 % (2.0*N.pi)
    dist = angle1 - angle2
    if dist > N.pi:
        dist = (2.0*N.pi) - dist
    elif dist < -N.pi:
        dist = (2.0*N.pi) + dist
    
    return dist
    


def radians_to_degrees(angle):
    if angle is not None:
        return angle*180.0/N.pi

def degrees_to_radians(angle):
    if angle is not None:
        return angle*N.pi/180.0

if __name__ == '__main__':
    pi = N.pi
    start_angle = [N.pi/4.0, N.pi/4.0, 7.0/4.0*N.pi, 7.0/4.0*N.pi]
    stop_angle = [0.0, 7.0/4.0*N.pi, 0.0, N.pi/4.0]
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
