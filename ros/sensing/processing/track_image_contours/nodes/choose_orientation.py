#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('plate_tf')
import rospy
from pythonmodules import CircleFunctions

import math, tf
# import numpy
# from geometry_msgs.msg import PoseStamped

class ChooseOrientation:
    def __init__(self):
        self.zaxis = (0,0,1)
        # self.angleOfTravelPreviousious = None
        self.disagreement_count = 0
        self.disagreement_count_limit = 10
        self.flipped_previous = None

    def angle_from_quaternion(self, quat):
        # Find and return angle of rotation about z-axis
        euler_angles = tf.transformations.euler_from_quaternion(quat, axes='sxyz')
        return euler_angles[2]

    def flip_quaternion(self, quat):
        # Cheesy way to flip quaternion, knowing it is a vector at origin in xy-plane

        ang = self.angle_from_quaternion(quat)
        # Add pi to angle and compute new quaternion
        qz = tf.transformations.quaternion_about_axis((ang + math.pi), self.zaxis)
        return qz

    # def mod_angle(self,angle):
    #     angle = angle%2*math.pi
    #     return angle

    # def condition_angle(self,angle):
    #     angle = self.mod_angle(angle)
    #     if math.pi < angle:
    #         angle = 2*math.pi - angle
    #     return angle

    def choose_orientation(self, q, angleOfTravel, stopped, angleOfTravelPrevious):
        # angleQ is ambiguous modulo pi radians
        angleOfTravel = CircleFunctions.mod_angle(angleOfTravel)
        angleOfTravelPrevious = CircleFunctions.mod_angle(angleOfTravelPrevious)

        # rospy.logwarn("angleQ = %s, angleOfTravel = %s" % (str(angleQ*180/math.pi),str(angleOfTravel*180/math.pi)))
        if not stopped:
            angleReference = angleOfTravel
        elif angleOfTravelPrevious is not None:
            angleReference = angleOfTravelPrevious
        else:
            return None

        angleQ = CircleFunctions.mod_angle(self.angle_from_quaternion(q))
        angleQ_flipped = CircleFunctions.mod_angle(angleQ + math.pi)
        diff_vel_ang         = abs(CircleFunctions.circle_dist(angleQ,         angleReference))
        diff_vel_ang_flipped = abs(CircleFunctions.circle_dist(angleQ_flipped, angleReference))

        if diff_vel_ang < diff_vel_ang_flipped:
            vel_ang_vote_on_flipped = False
        else:
            vel_ang_vote_on_flipped = True
        # rospy.logwarn("angleReference = %s, angleQ = %s, angleQ_flipped = %s, diff_vel_ang = %s, diff_vel_ang_flipped = %s" % (str(angleReference),str(angleQ*180/math.pi),str(angleQ_flipped*180/math.pi),str(diff_vel_ang*180/math.pi),str(diff_vel_ang_flipped*180/math.pi)))

        if angleOfTravelPrevious is not None:
            diff_prev_ang = abs(CircleFunctions.circle_dist(angleQ,angleOfTravelPrevious))
            diff_prev_ang_flipped = abs(CircleFunctions.circle_dist(angleQ_flipped,angleOfTravelPrevious))

            if diff_prev_ang < diff_prev_ang_flipped:
                prev_ang_vote_on_flipped = False
            else:
                prev_ang_vote_on_flipped = True

        else:
            prev_ang_vote_on_flipped = vel_ang_vote_on_flipped

        if vel_ang_vote_on_flipped and prev_ang_vote_on_flipped:
            flipped = True
            self.disagreement_count = 0
        elif (not vel_ang_vote_on_flipped) and (not prev_ang_vote_on_flipped):
            flipped = False
            self.disagreement_count = 0
        else:
            self.disagreement_count += 1
            # rospy.logwarn("angleQ = %s" % (str(angleQ)))
            # rospy.logwarn("angleQ_flipped = %s" % (str(angleQ_flipped)))
            # rospy.logwarn("disagreement_count = %s" % (str(self.disagreement_count)))

            # if vel_ang_vote_on_flipped:
            #     rospy.logwarn("diff_vel_ang_flipped = %s" % (str(diff_vel_ang_flipped)))
            # else:
            #     rospy.logwarn("diff_vel_ang = %s" % (str(diff_vel_ang)))

            # if prev_ang_vote_on_flipped:
            #     rospy.logwarn("diff_prev_ang_flipped = %s" % (str(diff_prev_ang_flipped)))
            # else:
            #     rospy.logwarn("diff_prev_ang = %s" % (str(diff_prev_ang)))

            if self.disagreement_count < self.disagreement_count_limit:
                flipped = prev_ang_vote_on_flipped
            else:
                flipped = vel_ang_vote_on_flipped

            # rospy.logwarn("flipped = %s" % (str(flipped)))
            # if self.flipped_previous is not None:
            #     rospy.logwarn("flipped_previous = %s" % (str(self.flipped_previous)))

        # if flipped != self.flipped_previous:
        #     rospy.logwarn("Flipping!!")
        #     rospy.logwarn("prev_ang_vote_on_flipped = %s" % (str(prev_ang_vote_on_flipped)))
        #     rospy.logwarn("vel_ang_vote_on_flipped = %s" % (str(vel_ang_vote_on_flipped)))
        #     rospy.logwarn("flipped = %s" % (str(flipped)))

        self.flipped_previous = flipped

        if not flipped:
            return q
        else:
            q_flipped = self.flip_quaternion(q)
            return q_flipped

    # def choose_orientations(rows, directions, frames_per_second=None,
    #                         velocity_weight_gain=0.5,
    #                         #min_velocity_weight=0.0,
    #                         max_velocity_weight=0.9,
    #                         elevation_up_bias_degrees=45.0, # tip the velocity angle closer +Z by this amount (maximally)
    #                         up_dir=None,
    #                         ):
    #     """Take input data which is wrapped (mod pi) and unwrap it.

    #     This uses a dynamic programming algorithm. The basic idea is that
    #     there are 2 constraints: a direction we think we should be
    #     oriented (with a time-varying weight of how much we believe that
    #     direction) and the belief that we shouldn't flip between
    #     orientations. Our belief direction comes from velocity (point in
    #     the direction of travel) and, optionally, is tipped towards up_dir
    #     by elevation_up_bias_degrees. The assignment of directions is
    #     based on the least-cost of there two terms where on each from the
    #     sign is flipped or not.

    #     This is heavily inspired by (i.e. some code stolen from) Kristin
    #     Branson's choose orientations in ctrax (formerly known as mtrax).

    #     Arguments
    #     ---------
    #     rows: structured array
    #         position and frame number (has columns 'x','y','z','frame') for N frames
    #     directions: Nx3 array
    #         direction to choose with is undetermined to a sign flip
    #     frames_per_second : float like
    #         framerate, used to determine velocity from position
    #     velocity_weight_gain : float like
    #         scale factor of importance to treat speed (units: importance per speed)
    #     max_velocity_weight : float like
    #         maximum weight to attribute to velocity term
    #     elevation_up_bias_degrees : float like
    #         number of degrees towards up_dir to bias the velocity direction
    #     up_dir : 3 vector
    #         direction to bias velocity direction

    #     Returns
    #     -------
    #     directions : Nx3 array
    #         directions above with sign chosen to minimize cost
    #     """
    #     if (up_dir is None) and (elevation_up_bias_degrees != 0):
    #         #up_dir = np.array([0,0,1],dtype=np.float)
    #         raise ValueError("up_dir must be specified. "
    #                          "(Hint: --up-dir='0,0,1')")
    #     D2R = np.pi/180

    #     if DEBUG:
    #         frames = rows['frame']
    #         if 1:
    #             cond1 = (128125 < frames) & (frames < 128140 )
    #             cond2 = (128460 < frames) & (frames < 128490 )
    #             cond = cond1 | cond2
    #             idxs = np.nonzero(cond)[0]
    #         else:
    #             idxs = np.arange( len(frames) )

    #     directions = np.array(directions,copy=True) # don't modify input data

    #     X = np.array([rows['x'], rows['y'], rows['z']]).T
    #     #ADS print "rows['x'].shape",rows['x'].shape
    #     assert len(X.shape)==2
    #     velocity = (X[1:]-X[:-1])*frames_per_second
    #     #ADS print 'velocity.shape',velocity.shape
    #     speed = np.sqrt(np.sum(velocity**2,axis=1))
    #     #ADS print 'speed.shape',speed.shape
    #     w = velocity_weight_gain*speed
    #     w = np.min( [max_velocity_weight*np.ones_like(speed), w], axis=0 )
    #     #w = np.max( [min_velocity_weight*np.ones_like(speed), w], axis=0 )
    #     #ADS print 'directions.shape',directions.shape
    #     #ADS print 'w.shape',w.shape

    #     velocity_direction = velocity/speed[:,np.newaxis]
    #     if elevation_up_bias_degrees != 0:

    #         # bias the velocity direction

    #         rot1_axis = np.cross(velocity_direction, up_dir)

    #         dist_from_zplus = np.arccos( np.dot(velocity_direction,up_dir))
    #         bias_radians = elevation_up_bias_degrees*D2R
    #         rot1_axis[abs(dist_from_zplus)>(np.pi-1e-14)]=up_dir # pathological case
    #         velocity_biaser = [ cgtypes.quat().fromAngleAxis(bias_radians,ax) for ax in rot1_axis ]
    #         biased_velocity_direction = [ rotate_vec( velocity_biaser[i],
    #                                                   cgtypes.vec3(*(velocity_direction[i]))) for i in range(len(velocity))]
    #         biased_velocity_direction = numpy.array([ [v[0], v[1], v[2]] for v in biased_velocity_direction ])
    #         biased_velocity_direction[ dist_from_zplus <= bias_radians, : ] = up_dir

    #         if DEBUG:
    #             R2D = 180.0/np.pi
    #             for i in idxs:
    #                 print
    #                 print 'frame %s ====================='%frames[i]
    #                 print 'X[i]',X[i,:]
    #                 print 'X[i+1]',X[i+1,:]
    #                 print 'velocity',velocity[i]
    #                 print
    #                 print 'rot1_axis',rot1_axis[i]
    #                 print 'up_dir',up_dir
    #                 print 'cross',np.cross(velocity_direction[i], up_dir)
    #                 print 'velocity_direction',velocity_direction[i]
    #                 print
    #                 print 'dist_from_zplus',dist_from_zplus[i]
    #                 print 'dist (deg)',(dist_from_zplus[i]*R2D)
    #                 print 'bias_radians',bias_radians
    #                 print
    #                 print 'velocity_biaser',velocity_biaser[i]
    #                 print 'biased_velocity_direction',biased_velocity_direction[i]

    #     else:
    #         biased_velocity_direction = velocity_direction

    #     # allocate space for storing the optimal path
    #     signs = [1,-1]
    #     stateprev = np.zeros((len(directions)-1,len(signs)),dtype=bool)

    #     tmpcost = [0,0]
    #     costprevnew = [0,0]
    #     costprev = [0,0]

    #     # iterate over each time point
    #     for i in range(1,len(directions)):
    #         #ADS print 'i',i

    #         #ADS print 'directions[i]',directions[i]
    #         #ADS print 'directions[i-1]',directions[i-1]
    #         if DEBUG and i in idxs:
    #             print
    #             #print 'i',i
    #             print 'frame',frames[i],'='*50
    #             print 'directions[i]',directions[i]
    #             print 'directions[i-1]',directions[i-1]
    #             print 'velocity weight w[i-1]',w[i-1]
    #             print 'speed',speed[i-1]
    #             print 'velocity_direction[i-1]',velocity_direction[i-1]
    #             print 'biased_velocity_direction[i-1]',biased_velocity_direction[i-1]

    #         for enum_current,sign_current in enumerate(signs):
    #             direction_current = sign_current*directions[i]
    #             this_w = w[i-1]
    #             vel_term = np.arccos( np.dot( direction_current, biased_velocity_direction[i-1] ))
    #             up_term  = np.arccos( np.dot( direction_current, up_dir))
    #             #ADS print
    #             #ADS print 'sign_current',sign_current,'-'*50
    #             for enum_previous,sign_previous in enumerate(signs):
    #                 direction_previous = sign_previous*directions[i-1]
    #                 ## print 'direction_current'
    #                 ## print direction_current
    #                 ## print 'biased_velocity_direction'
    #                 ## print biased_velocity_direction
    #                 #ADS print 'sign_previous',sign_previous,'-'*20
    #                 #ADS print 'w[i-1]',w[i-1]
    #                 ## a=(1-w[i-1])*np.arccos( np.dot( direction_current, direction_previous))

    #                 ## b=np.dot( direction_current, biased_velocity_direction[i] )
    #                 ## print a.shape
    #                 ## print b.shape

    #                 flip_term = np.arccos( np.dot( direction_current, direction_previous))
    #                 #ADS print 'flip_term',flip_term,'*',(1-w[i-1])
    #                 #ADS print 'vel_term',vel_term,'*',w[i-1]

    #                 cost_current = 0.0
    #                 # old way
    #                 if not np.isnan(vel_term):
    #                     cost_current += this_w*vel_term
    #                 if not np.isnan(flip_term):
    #                     cost_current += (1-this_w)*flip_term
    #                 if not np.isnan(up_term):
    #                     cost_current += (1-this_w)*up_term

    #                 ## if (not np.isnan(direction_current[0])) and (not np.isnan(direction_previous[0])):
    #                 ##     # normal case - no nans
    #                 ##     cost_current = ( (1-w[i-1])*flip_term + w[i-1]*vel_term )
    #                 ##     cost_current = 0.0

    #                 #ADS print 'cost_current', cost_current
    #                 tmpcost[enum_previous] = costprev[enum_previous] + cost_current
    #                 if DEBUG and i in idxs:
    #                     print '  (sign_current %d)'%sign_current, '-'*10
    #                     print '  (sign_previous %d)'%sign_previous
    #                     print '  flip_term',flip_term
    #                     print '  vel_term',vel_term
    #                     print '  up_term',up_term
    #                     print '  cost_current',cost_current

    #             best_enum_previous = np.argmin( tmpcost )
    #             ## if DEBUG and i in idxs:
    #             ##     print 'tmpcost',tmpcost
    #             ##     print 'enum_current',enum_current
    #             ##     print 'best_enum_previous',best_enum_previous
    #             stateprev[i-1,enum_current] = best_enum_previous
    #             costprevnew[enum_current] = tmpcost[best_enum_previous]
    #         ## if DEBUG and i in idxs:
    #         ##     print 'costprevnew',costprevnew
    #         costprev[:] = costprevnew[:]
    #     #ADS print '='*100
    #     #ADS print 'costprev',costprev
    #     best_enum_current = np.argmin(costprev)
    #     #ADS print 'best_enum_current',best_enum_current
    #     sign_current = signs[best_enum_current]
    #     directions[-1] *= sign_current
    #     for i in range(len(directions)-2,-1,-1):
    #         #ADS print 'i',i
    #         #ADS print 'stateprev[i]',stateprev[i]
    #         best_enum_current = stateprev[i,best_enum_current]
    #         #ADS print 'best_enum_current'
    #         #ADS print best_enum_current
    #         sign_current = signs[best_enum_current]
    #         #ADS print 'sign_current',sign_current
    #         directions[i] *= sign_current

    #     if DEBUG:
    #         for i in idxs:
    #             print 'ultimate directions:'
    #             print 'frame',frames[i],directions[i]
    #     return directions

