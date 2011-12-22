#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('stage')
import rospy
import StageDevice
import flystage.srv
import threading

class StageCommunicator():
    def __init__(self):
        rospy.loginfo("Opening Flyatar stage device...")
        self.dev = StageDevice.StageDevice()
        rospy.on_shutdown(self.cbOnShutdown)
        self.dev.print_values()
        self.reentrant_lock = threading.Lock()

    def close(self):
        self.dev.close()
        rospy.loginfo("Flyatar stage device closed.")

    def cbGetStageState(self,req):
        with self.reentrant_lock:
            response = self.dev.get_stage_state()
        return response

    def cbSetStageVelocity(self,req):
        x_vel_list = req.x_velocity
        y_vel_list = req.y_velocity
        vel_mag_list = req.speed
        if (len(x_vel_list) == 0) or (len(y_vel_list) == 0):
            rospy.logerr("Error in set_stage_velocity call: x_vel_list or y_vel_list length == 0")
        else:
            with self.reentrant_lock:
                response = self.dev.update_stage_velocity(x_vel_list,y_vel_list,vel_mag_list)
            return response

    def cbSetStagePosition(self,req):
        x_pos_list = req.x_position
        y_pos_list = req.y_position
        vel_mag_list = req.speed
        if (len(x_pos_list) == 0) or (len(y_pos_list) == 0) or (len(vel_mag_list) == 0):
            rospy.logerr("Error in set_stage_position call: x_pos_list or y_pos_list or vel_mag_list length == 0")
        else:
            with self.reentrant_lock:
                response = self.dev.update_stage_position(x_pos_list,y_pos_list,vel_mag_list)
            return response

    def cbHomeStage(self,req):
        with self.reentrant_lock:
            response = self.dev.home_stage()
        return response
    
    def cbOnShutdown(self):
        close()
        

if __name__ == '__main__':
    rospy.init_node('StageCommunicator') #, anonymous=True)
    sc = StageCommunicator()
    s_gss = rospy.Service('get_stage_state',    flystage.srv.Stage_State, sc.cbGetStageState)
    s_ssv = rospy.Service('set_stage_velocity', flystage.srv.Stage_State, sc.cbSetStageVelocity)
    s_ssp = rospy.Service('set_stage_position', flystage.srv.Stage_State, sc.cbSetStagePosition)
    s_hs  = rospy.Service('home_stage',         flystage.srv.Stage_State, sc.cbHomeStage)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  sc.close()
