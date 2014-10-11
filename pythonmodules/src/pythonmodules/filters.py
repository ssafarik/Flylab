#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('arena_tf')
import rospy
import numpy as np
from pythonmodules import CircleFunctions

import cv
from geometry_msgs.msg import PoseStamped



class ButterworthFilter:
    def __init__(self, a=[1,0,0,0,0], b=[1,0,0,0,0]):
        self.a = a
        self.b = b
        self.z = [0,0,0,0] # 4th order
        
        
    def GetValue(self):
        return self.z[0]

    def Update(self, x, t):
        a = self.a
        b = self.b
        z = self.z
        if (t is not None) and (self.tminus1 is not None):
            dt = t - self.tminus1 # TODO: use dt.
            #alpha = dt/(self.RC + dt)
            y    = b[0]*x + z[0]
            z[0] = b[1]*x + z[1] - a[1]*y
            z[1] = b[2]*x + z[2] - a[2]*y
            z[2] = b[3]*x + z[3] - a[3]*y
            z[3] = b[4]*x        - a[4]*y
            self.z = z
                
                
        self.tminus1 = t

        return y


# Filter an angle where update values only range over a distance of pi, and where
# the update angle can be interpreted in either direction (+0 or +-180 degrees).  
# The actual angle can range over 2pi.  The filter keeps an internal unwrapped 
# angle (across 2pi) with no range limits.
#
class LowPassHalfCircleFilter:
    def __init__(self, RC=1.0):
        self.RC = RC
        self.t = None
        self.zf = None

        
    def GetValue(self):
        return self.zf


    def SetValue(self, z):
        self.zf = z


    def Update(self, z, t):
        if not isinstance(z,float):
            z = None
            
        if (self.zf is not None) and (self.t is not None): 
            if (z is not None) and (t is not None):
                zUnwrapped = CircleFunctions.UnwrapHalfCircle(z, self.zf)
                
                dt = t - self.t
                tt = (self.RC + dt)
                if (tt>0.0):
                    alpha = dt/(self.RC + dt)
                else:
                    alpha = 0.0
                    
                self.zf = (1.0 - alpha)*self.zf + alpha*zUnwrapped
            else: # Initialized, but no measurement.
                pass
        else: # Not initialized, but have a measurement.
            self.zf = z


        self.t = t

        return self.zf


# Filter an angle, with unwrapping by 2*pi.
class LowPassCircleFilter:
    def __init__(self, RC=1.0):
        self.RC = RC
        self.t = None
        self.zf = None

        
    def GetValue(self):
        return self.zf


    def SetValue(self, z):
        self.zf = z


    def Update(self, z, t):
        if not isinstance(z,float):
            z = None
            
        if (self.zf is not None) and (self.t is not None): 
            if (z is not None) and (t is not None):
                
                # Unwrap big jumps
                if (z - self.zf) > np.pi:
                    self.zf += (2.0*np.pi)
                if (z - self.zf) < -np.pi:
                    self.zf -= (2.0*np.pi)
                    
                dt = t - self.t
                alpha = dt/(self.RC + dt)
                self.zf = (1 - alpha)*self.zf + alpha*z
            else: # Initialized, but no measurement.
                pass
        else: # Not initialized, but have a measurement.
            self.zf = z


        self.t = t

        return self.zf


class LowPassFilter:
    def __init__(self, RC=1.0):
        self.RC = RC
        self.t = None
        self.zf = None

        
    def GetValue(self):
        return self.zf


    def SetValue(self, z):
        self.zf = z


    def Update(self, z, t):
        if not isinstance(z,float):
            z = None
            
        if (self.zf is not None) and (self.t is not None):
            if (z is not None) and (t is not None):
                dt = t - self.t
                alpha = dt/(self.RC + dt)
                zf = alpha*z + (1 - alpha)*self.zf
            else: # Initialized, but no measurement.
                zf = self.zf
        else: # Not initialized, but have a measurement.
            zf = z

        self.t = t
        self.zf = zf

        return zf


class KalmanFilter:
# Covariance of the magnet contour, with magnet at rest, and after subtracting the mean.
#   5.0514e-05   7.6803e-06
#   7.6803e-06   8.5906e-06
#
# Covariance of the magnet contour, moving in a spiral, and after subtracting the commanded position & mean.
#   15.3408    2.3540
#    2.3540   20.0380

# Covariance of [x,y,vx,vy] of magnet contour, moving in a spiral, and after subtracting the commanded position & mean,
# and using the 1st order position difference as velocity.
#   15.341     2.3540    -1.4330    -0.10971
#   2.3540    20.038     -0.097233  -1.6238
#  -1.4330    -0.097233   0.22291    0.0055711
#  -0.10971   -1.6238     0.0055711  0.21742

    
    def __init__(self):
        self.initialized = False
        
        self.kal = cv.CreateKalman(4,4,0)
        cv.SetIdentity(self.kal.transition_matrix)
        cv.SetIdentity(self.kal.measurement_matrix)
        
        cv.SetIdentity(self.kal.process_noise_cov, 1.0)
        self.kal.process_noise_cov[2,2] = 0.5
        self.kal.process_noise_cov[3,3] = 0.5
        
        cv.SetIdentity(self.kal.measurement_noise_cov, 80.0)
        self.kal.measurement_noise_cov[0,0] = 5E-5 #15.341
        self.kal.measurement_noise_cov[0,1] = 8E-6 # 2.354
        self.kal.measurement_noise_cov[1,0] = 8E-6 #1E-2 # 2.354
        self.kal.measurement_noise_cov[1,1] = 5E-5 # Likely to be similar to the x value.
        self.kal.measurement_noise_cov[2,2] =  40.0#0.22291
        self.kal.measurement_noise_cov[3,3] =  40.0#0.21742
        
        
        self.measurement = cv.CreateMat(4,1,cv.GetElemType(self.kal.state_pre))
        self.t = None
        self.zf = None


    def Update(self, z, t=None):
        # note: z=(x,y)
        if t is not None:
            tNew = t
        else:
            tNew = rospy.Time.now()
            

        if self.initialized:
            # Update the state transition matrix for dt.
            self.dt = tNew - self.t
            self.kal.transition_matrix[0,2] = self.dt
            self.kal.transition_matrix[1,3] = self.dt
            #rospy.logwarn ('KF dt=' + '*' * int(1/(20*self.dt)))

            # Kalman Filtering      
            state_pre = cv.KalmanPredict(self.kal)          
            if (z is not None) and (self.zf is not None) and (self.dt != 0):
                self.measurement[0,0] = z[0]
                self.measurement[1,0] = z[1]
                self.measurement[2,0] = (z[0] - self.zf[0]) / self.dt
                self.measurement[3,0] = (z[1] - self.zf[1]) / self.dt
    
                state_post = cv.KalmanCorrect(self.kal, self.measurement)
                x = state_post[0,0]
                y = state_post[1,0]
                vx = state_post[2,0]
                vy = state_post[3,0]
            else:
                x = state_pre[0,0]
                y = state_pre[1,0]
                vx = state_pre[2,0]
                vy = state_pre[3,0]
                #rospy.loginfo('KF z==None -> x,y=%s' % [x,y])
                
            self.zf = z

        else: # not initialized.
            if (z is not None):
                # Set initial conditions
                x = z[0]
                y = z[1]
                vx = 0.0
                vy = 0.0
                cv.Set2D(self.kal.state_post, 0, 0, z[0])
                cv.Set2D(self.kal.state_post, 1, 0, z[1])
                cv.Set2D(self.kal.state_post, 2, 0, vx)
                cv.Set2D(self.kal.state_post, 3, 0, vy)
                rospy.loginfo ('KF initialized kalman filter to %s' % [z[0], z[1], vx, vy])

                self.zf = z
                
                self.initialized = True
            else:
                x = None
                y = None
                vx = None
                vy = None
                
        self.t = tNew
        
        return (x, y, vx, vy)


