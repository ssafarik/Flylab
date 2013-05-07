#!/usr/bin/env python
from __future__ import division
import sys

try:
 	import pygtk
  	pygtk.require("3.0")
except:
  	pass

try:
	import gtk
  	import gtk.glade
except:
	sys.exit(1)

import roslib; roslib.load_manifest('ui')
import rospy

from std_msgs.msg import String
from flycore.msg import TrackingCommand



class FlylabGUI:
    def __init__(self):
    	rospy.init_node('FlylabGUI')
    
    	#Set the path to Glade file.  Edit with 'Glade Interface Designer'
    	self.gladefile = roslib.packages.get_pkg_dir('ui')+"/nodes/FlylabGUI.glade" 
    	self.builder = gtk.Builder()
    	self.builder.add_from_file(self.gladefile)
    	self.builder.connect_signals(self) #Connect GUI event functions.
    	
    	self.pubBroadcastCommand = rospy.Publisher('broadcast/command', String)
        self.subBroadcastCommand = rospy.Subscriber('broadcast/command', String, self.CommandExperiment_callback)
    	self.pubTrackingCommand   = rospy.Publisher('tracking/command', TrackingCommand)


    def CommandExperiment_callback(self, msgString):
        if (msgString.data=='exit_now'):
            sys.exit(0)
    	
    
    def BtnPauseAfterTrial_clicked_cb(self, widget):
        if not rospy.is_shutdown():
            command = String(data='pause_after_trial')
            self.pubBroadcastCommand.publish(command)
    
    def BtnPauseNow_clicked_cb(self, widget):
        if not rospy.is_shutdown():
            command = String(data='pause_now')
            self.pubBroadcastCommand.publish(command)
    
    def BtnContinue_clicked_cb(self, widget):
    	if not rospy.is_shutdown():
    		command = String(data='continue')
    		self.pubBroadcastCommand.publish(command)
    
    def BtnExitAfterTrial_clicked_cb(self, widget):
        if not rospy.is_shutdown():
            command = String(data='exit_after_trial')
            self.pubBroadcastCommand.publish(command)
    
    def BtnExitNow_clicked_cb(self, widget):
        if not rospy.is_shutdown():
            command = String(data='exit_now')
            self.pubBroadcastCommand.publish(command)
            rospy.sleep(1)
            sys.exit(0)
    
    def BtnSaveBackground_clicked_cb(self, widget):
        if not rospy.is_shutdown():
            command = TrackingCommand(command='save_background')
            self.pubTrackingCommand.publish(command)
    
    def BtnEstablishBackground_clicked_cb(self, widget):
        if not rospy.is_shutdown():
            command = TrackingCommand(command='establish_background')
            self.pubTrackingCommand.publish(command)
    
    def MainWindow_destroy_cb(self, widget):
    	sys.exit(0)

    	
if __name__ == "__main__":
    gui = FlylabGUI()
    while (gtk.main_iteration(block=False)):
        if (rospy.is_shutdown()):
            sys.exit(0)
            
        rospy.sleep(-1)  # Returns immediately.  Equivalent to rospy.spinOnce(), if it existed.
    
    
