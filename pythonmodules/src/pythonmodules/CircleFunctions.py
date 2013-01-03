#!/usr/bin/env python
from __future__ import division
import numpy as N



# Rerange()
#   Put the angle into the range of -pi to +pi.
#
def Rerange(self, angle):
    angleOut = ((angle+N.pi) % (2.0*N.pi)) - N.pi
    
    return angleOut
    
    

# Get the distance between two angles.  Cannot be further than pi apart.
def DistanceCircle(angle2, angle1):
    angle1 = angle1 % (2.0*N.pi)
    angle2 = angle2 % (2.0*N.pi)
    dist = angle2 - angle1
    if dist > N.pi:
        dist = dist - (2.0*N.pi) # - dist
    elif dist < -N.pi:
        dist = dist + (2.0*N.pi) # + dist
    
    return dist
    
# Compute the shortest distance from one angle to another, where an angle can wrap at the half-circle,
# meaning that the arrow can be pointing either way.  Cannot be further than pi/2 apart.
#
def DistanceHalfCircle(angle2, angle1):
    angle1 = angle1 % (N.pi)
    angle2 = angle2 % (N.pi)
    dist = angle2 - angle1
    if dist > N.pi/2:
        dist = dist - (N.pi)
    elif dist < -N.pi/2:
        dist = dist + (N.pi)
        
    return dist


# UnwrapCircle()
# Output a continuous angle, without jumps greater than pi.
# If there was a jump greater than pi, it means that the new angle was
# off by 2*pi.
#
def UnwrapCircle(angle, angle_prev):
    if (angle is not None) and (angle_prev is not None):
        delta_angle = circle_dist(angle_prev,angle)
        unwrapped_angle = angle_prev + delta_angle
    elif angle is not None:
        unwrapped_angle = angle
    return unwrapped_angle


# UnwrapHalfCircle()
# Output a continuous angle, without jumps greater than pi/2.
# If there was a jump greater than pi/2, it means that the new angle was
# off by pi.
#
def UnwrapHalfCircle(angle, anglePrev):
    if (angle is not None):
        if (anglePrev is not None):
            delta_angle = DistanceHalfCircle(angle, anglePrev)
            angleUnwrapped = anglePrev + delta_angle
        else:
            angleUnwrapped = angle

    return angleUnwrapped


def DegreesFromRadians(angle):
    if angle is not None:
        return angle*180.0/N.pi

def RadiansFromDegrees(angle):
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
        print "start_angle = %s" % (str(DegreesFromRadians(start_ang)))
        print "stop_angle = %s" % (str(DegreesFromRadians(stop_ang)))
        print "diff = %s" % (str(DegreesFromRadians(diff)))

        ang = stop_ang
        ang_prev = start_ang
        unwrapped_ang = UnwrapCircle(ang, ang_prev)
        print "ang_prev = %s" % (str(DegreesFromRadians(ang_prev)))
        print "ang = %s" % (str(DegreesFromRadians(ang)))
        print "unwrapped_ang = %s" % (str(DegreesFromRadians(unwrapped_ang)))
