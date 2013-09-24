#!/usr/bin/env python


# This file reads a .bag file and saves the contained images to .png files.  
# Specify the .bag file and the image topic on the command line, and
# optionally the .mov filename to write as a movie file. 
#
#   rosrun save bag2png.py <bagfile> <imagetopic>] [filename.mov]
#
# for example:
#   rosrun save bag2png.py yourfile.bag /camera/image_rect /home/rancher/asdf.mov
#


import os
import subprocess
import sys

import Image
import rospy
import rosbag
import cv

sys.path.insert(0,'/opt/ros/fuerte/stacks/vision_opencv/cv_bridge/src') # There's probably a better way to find cv_bridge.
import cv_bridge

iImage = 0
cvbridge = cv_bridge.CvBridge()


# Get the .mov filename 3rd parameter.
filenameMov = None
if (len(sys.argv)==4):
    if ('.mov' in sys.argv[3]):
        filenameMov = sys.argv[3]
    else:
        rospy.logwarn('bag2png: Third parameter must be the full path spec of the .mov file to write.')
    
if (len(sys.argv)>=3):
    # Get the first and second command parameters.
    bag = rosbag.Bag(sys.argv[1])
    topicToSave = sys.argv[2]
    
    # Use the tail end of the topic as the image dir, e.g. 'camnode/image_raw' uses 'image_raw'
    dirImages = topicToSave.split(os.sep)[-1]
    # Make sure dir exists.
    try:
        os.makedirs(dirImages)
    except OSError:
        pass
        
    
    
    # Read all the messages in the bag file.    
    for (topic, msg, t) in bag.read_messages(topicToSave):
        
        # Handle compressed images.
        if ('compressed' in topic):
            if ('png' in msg.format):
                ext = 'png'
            elif ('jpeg' in msg.format):
                ext = 'jpg'
            else:
                ext = 'xxx'
                
                # Make a filename like 'image_raw/000123.png'
            filename = '%s%s%06d.%s' % (dirImages, os.sep, iImage, ext)

            # Save the image file.
            file = open(filename,'w')
            file.write(msg.data)
            file.close()

            iImage += 1
                
        else:
            # Handle uncompressed images. 
            if msg.encoding=='mono8':
                ext = 'png'
                
                # Make a filename like 'image_raw/000123.png'
                filename = '%s%s%06d.%s' % (dirImages, os.sep, iImage, ext)
    
                # Save the image file.
                matImage = cv.GetImage(cvbridge.imgmsg_to_cv(msg, 'passthrough'))
                cv.SaveImage(filename, matImage)
    
                    
                rospy.logwarn ('Wrote %s' % filename)
                iImage += 1
            else:
                rospy.logwarn ('Only image encoding==mono8 is supported.  This one has %s' % msg.encoding)
                

    # If requested, convert the image files to an .mov file, then delete the images.                
    if (filenameMov is not None):
        # Run avconv, and then remove all the image files.
        cmdCreateVideoFile = 'avconv -y -r 60 -i %s/%%06d.%s -same_quant -r 60 %s && rm -rf %s && echo Finished.' % (dirImages, ext, filenameMov, dirImages)
        rospy.logwarn('Converting images to video using command:')
        rospy.logwarn (cmdCreateVideoFile)
        try:
            processAvconv = subprocess.Popen(cmdCreateVideoFile, shell=True)
        except:
            rospy.logerr('Exception running avconv to convert to .mov')

        
                    
    if (iImage==0):
        rospy.logwarn('Please specify a valid image topic:  bag2png filename.bag imagetopic [filename.mov]')
        rospy.logwarn('Topics in %s are:' % sys.argv[1])
        for (i,c) in bag._connections.iteritems():
            rospy.logwarn(c.topic)
            
else:
    rospy.logwarn ('Usage:  bag2png filename.bag imagetopic [filename.mov]')
    rospy.logwarn ('  Extracts the image files from a .bag file, and writes them to')
    rospy.logwarn ('  disk in a subdirectory named from the imagetopic.')
    rospy.logwarn ('  Can also then optionally convert those images to a .mov file.')
    rospy.logwarn ('  If the optional .mov file is specified, the images will be')
    rospy.logwarn ('  converted to a .mov file, and then images deleted.')
    
    