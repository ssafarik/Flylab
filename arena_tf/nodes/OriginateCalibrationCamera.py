#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('arena_tf')
import rospy
from arena_tf.msg import CalibrationCamera




###############################################################################
###############################################################################
###############################################################################
# The class CalibrationOriginate simply publishes the calibration info at startup.
# The purpose is that since the calibration is input to the system, we want 
# to be able to change whether we use the params on disk, or the params 
# in a .bag file.
#
# Don't run this node when replaying a .bag file.  The data will come from the .bag file instead.
#
class OriginateCalibrationCamera:

    def __init__(self):
        
        # Messages
        self.pubCalibrationOriginate         = rospy.Publisher("camera/calibration_originate", CalibrationCamera, latch=True)
        

        # Load the calibration params.
        calibration = CalibrationCamera()
        
        calibration.xMask        = rospy.get_param('camera/mask/x', 0.0) 
        calibration.yMask        = rospy.get_param('camera/mask/y', 0.0) 
        calibration.arena_rvec_0 = rospy.get_param('camera/arena_rvec_0', 3.14159)
        calibration.arena_rvec_1 = rospy.get_param('camera/arena_rvec_1', 0.0)
        calibration.arena_rvec_2 = rospy.get_param('camera/arena_rvec_2', 0.0)
        calibration.arena_tvec_0 = rospy.get_param('camera/arena_tvec_0', 0.0)
        calibration.arena_tvec_1 = rospy.get_param('camera/arena_tvec_1', 0.0)
        calibration.arena_tvec_2 = rospy.get_param('camera/arena_tvec_2', 0.0)
            
        self.pubCalibrationOriginate.publish(calibration)
            


    def Main(self):
        rospy.spin()
      

if __name__ == '__main__':
    rospy.init_node('OriginateCalibrationCamera')
    rospy.sleep(1)
    try:
        cal = OriginateCalibrationCamera()
        cal.Main()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down")
  
