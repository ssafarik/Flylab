#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('arena_tf')
import rospy
from arena_tf.msg import CalibrationStage




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
class OriginateCalibrationStage:

    def __init__(self):

        # Messages
        self.pubCalibrationOriginate = rospy.Publisher("stage/calibration_originate", CalibrationStage, latch=True)
        
        

        # Load the calibration params.
        calibration = CalibrationStage()
        
        calibration.arena_x  = rospy.get_param('stage/arena_x', 0.0)
        calibration.arena_y  = rospy.get_param('stage/arena_y', 0.0)
        calibration.arena_z  = rospy.get_param('stage/arena_z', 0.0)
        calibration.arena_qx = rospy.get_param('stage/arena_qx', 0.0)
        calibration.arena_qy = rospy.get_param('stage/arena_qy', 0.0)
        calibration.arena_qz = rospy.get_param('stage/arena_qz', 0.0)
        calibration.arena_qw = rospy.get_param('stage/arena_qw', 1.0)
            
        self.pubCalibrationOriginate.publish(calibration)
            

    def Main(self):
        rospy.spin()
      

if __name__ == '__main__':
    rospy.init_node('OriginateCalibrationStage')
    try:
        cal = OriginateCalibrationStage()
        cal.Main()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down")
  
