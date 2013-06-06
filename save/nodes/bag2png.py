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
import cv

sys.path.insert(0,'/opt/ros/fuerte/stacks/vision_opencv/cv_bridge/src') # There's probably a better way to find cv_bridge.
import cv_bridge

iImage = 0
cvbridge = cv_bridge.CvBridge()


if (len(sys.argv)>=2):

    bag = rosbag.Bag(sys.argv[1])
    
    if (len(sys.argv)>=3):
        topicToSave = sys.argv[2]
    else:
        topicToSave = 'image_rect'
    
        
    for (topic, msg, t) in bag.read_messages(topicToSave):
        
        # Handle compressed images.
        if ('compressed' in topic):
            if ('png' in msg.format):
                ext = 'png'
            elif ('jpeg' in msg.format):
                ext = 'jpg'
            else:
                ext = 'xxx'
                
            filename = '%06d.%s' % (iImage,ext)
            file = open(filename,'w')
            file.write(msg.data)
            file.close()
            iImage += 1
                
        else:
            # Handle uncompressed images. 
            if msg.encoding=='mono8':
                filename = '%06d.png' % iImage
    
    #            image = Image.fromstring('L',(msg.width, msg.height), msg.data)
    #            image.save(filename)
    
                # Way faster to use opencv for saving...
                matImage = cv.GetImage(cvbridge.imgmsg_to_cv(msg, 'passthrough'))
                cv.SaveImage(filename, matImage)
    
                    
                rospy.logwarn ('Wrote %s' % filename)
                iImage += 1
            else:
                rospy.logwarn ('Only image encoding==mono8 is supported.  This one has %s' % msg.encoding)
                    
    if (iImage==0):
        rospy.logwarn('Please specify an image topic:  bag2png filename.bag imagetopic')
        rospy.logwarn('Topics in %s are:' % sys.argv[1])
        for (i,c) in bag._connections.iteritems():
            rospy.logwarn(c.topic)
            
else:
    rospy.logwarn ('Please specify the .bag filename:  bag2png filename.bag [imagetopic]')
    
    