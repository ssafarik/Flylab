#!/usr/bin/env python
from __future__ import division
import numpy as np



# Rerange()
#   Put the angle into the range of -pi to +pi.
#
def Rerange(self, angle):
    angleOut = ((angle+np.pi) % (2.0*np.pi)) - np.pi
    
    return angleOut
    
    

# Get the distance between two angles.  Cannot be further than pi apart.
def DistanceCircle(angle2, angle1):
    angle1b = angle1 % (2.0*np.pi)
    angle2b = angle2 % (2.0*np.pi)
    distb = angle2b - angle1b
    if distb > np.pi:
        dist = distb - (2.0*np.pi)
    elif distb < -np.pi:
        dist = distb + (2.0*np.pi)
    else:
        dist = distb
        
    return dist
    
# Compute the shortest distance from one angle to another, where an angle can wrap at the half-circle,
# meaning that the arrow can be pointing either way.  Cannot be further than pi/2 apart.
#
def DistanceHalfCircle(angle2, angle1):
    angle1b = angle1 % (2.0*np.pi)   # angle1 is good on 2pi.
    angle2b = angle2 % (np.pi)       # angle2b on range [0,pi)
    angle2c = angle2b + np.pi        # angle2c on range [pi,2pi)
    
    distb = angle2b - angle1b
    distc = angle2c - angle1b
    
    if distb > np.pi:
        distb -= 2*np.pi
    if distb < -np.pi:
        distb += 2*np.pi
    if distc > np.pi:
        distc -= 2*np.pi
    if distc < -np.pi:
        distc += 2*np.pi

    if np.abs(distb) < np.abs(distc):
        dist = distb
    else:
        dist = distc
                        
#    print distb, distc
#    if distb > np.pi/2:
#        dist = distb - (np.pi)
#        print "A"
##    elif distb < -np.pi:
##        dist = distb + (2*np.pi)
##        print "B"
#    elif distb < -np.pi/2:
#        dist = distb + (np.pi)
#        print "C"
#    else:
#        dist = distb
#        print "D"
        
    return dist


# UnwrapCircle()
# Output a continuous angle, without jumps greater than pi.
# If there was a jump greater than pi, it means that the new angle was
# off by 2*pi.
#
def UnwrapCircle(angle, anglePrev):
    if (angle is not None):
        if (anglePrev is not None):
            delta = DistanceCircle(angle, anglePrev)
            angleUnwrapped = anglePrev + delta
        else:
            angleUnwrapped = angle
    else:
        angleUnwrapped = None
        
    return angleUnwrapped


# UnwrapHalfCircle()
# Output a continuous angle, without jumps greater than pi/2.
# If there was a jump greater than pi/2, it means that the new angle was
# off by pi.
#
def UnwrapHalfCircle(angle, anglePrev):
    if (angle is not None):
        if (anglePrev is not None):
            delta = DistanceHalfCircle(angle, anglePrev)
            angleUnwrapped = anglePrev + delta
        else:
            angleUnwrapped = angle
    else:
        angleUnwrapped = None
        
    return angleUnwrapped



if __name__ == '__main__':
    pi = np.pi
    start_angle = [np.pi/4.0, np.pi/4.0, 7.0/4.0*np.pi, 7.0/4.0*np.pi]
    stop_angle = [0.0, 7.0/4.0*np.pi, 0.0, np.pi/4.0]
    for ang in range(len(start_angle)):
        start_ang = start_angle[ang]
        stop_ang = stop_angle[ang]
        diff = DistanceCircle(start_ang,stop_ang)
        print "start_angle = %s" % (str(np.rad2deg(start_ang)))
        print "stop_angle = %s" % (str(np.rad2deg(stop_ang)))
        print "diff = %s" % (str(np.rad2deg(diff)))

        ang = stop_ang
        ang_prev = start_ang
        unwrapped_ang = UnwrapCircle(ang, ang_prev)
        print "ang_prev = %s" % (str(np.rad2deg(ang_prev)))
        print "ang = %s" % (str(np.rad2deg(ang)))
        print "unwrapped_ang = %s" % (str(np.rad2deg(unwrapped_ang)))
