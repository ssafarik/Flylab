#!/usr/bin/env python


# This file reads a .bag file and saves the contained images to .png files.  
# Specify the .bag file and optionally the image topic on the command line, 
# for example:
#   rosrun save bag2png.py yourfile.bag image_rect
#


import sys

import Image
import rospy
import rosbag

iImage = 0

if (len(sys.argv)>=2):

    if (len(sys.argv)>=3):
        topicToSave = sys.argv[2]
    else:
        topicToSave = 'image_rect'
    
        
    for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages():
        if topic.endswith(topicToSave):
            if msg.encoding=='mono8':
                im = Image.fromstring('L',(msg.width, msg.height), msg.data)
                if im:
                    im.save('%06d.png' % (iImage))
                iImage += 1
            else:
                rospy.logwarn ('Only image encoding==mono8 is supported.  This one has %s' % msg.encoding)
else:
    rospy.logwarn ('Please specify the .bag filename:  bag2png filename.bag [imagetopic]')
    rospy.logwarn (' for example')
    rospy.logwarn ('                                   bag2png asdf.bag image_rect')
    