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
    	
    	self.pubExperimentCommand = rospy.Publisher('experiment/command', String)
        self.subExperimentCommand = rospy.Subscriber('experiment/command', String, self.CommandExperiment_callback)
    	self.pubTrackingCommand   = rospy.Publisher('tracking/command', TrackingCommand)


    def CommandExperiment_callback(self, msgString):
        if (msgString.data=='exitnow'):
            sys.exit(0)
    	
    
    def BtnPause_clicked_cb(self, widget):
    	if not rospy.is_shutdown():
    		command = String(data='pause')
    		self.pubExperimentCommand.publish(command)
    
    def BtnContinue_clicked_cb(self, widget):
    	if not rospy.is_shutdown():
    		command = String(data='continue')
    		self.pubExperimentCommand.publish(command)
    
    def BtnExit_clicked_cb(self, widget):
        if not rospy.is_shutdown():
            command = String(data='exit')
            self.pubExperimentCommand.publish(command)
    
    def BtnExitNow_clicked_cb(self, widget):
        if not rospy.is_shutdown():
            command = String(data='exitnow')
            self.pubExperimentCommand.publish(command)
            rospy.sleep(1)
            sys.exit(0)
    
    def BtnSaveBackground_clicked_cb(self, widget):
    	if not rospy.is_shutdown():
    		command = TrackingCommand(command='savebackground')
    		self.pubTrackingCommand.publish(command)
    
    def MainWindow_destroy_cb(self, widget):
    	sys.exit(0)

    	
if __name__ == "__main__":
    gui = FlylabGUI()
    while (gtk.main_iteration(block=False)):
        if (rospy.is_shutdown()):
            sys.exit(0)
            
        rospy.sleep(-1)  # Returns immediately.  Equivalent to rospy.spinOnce(), if it existed.
    
    
