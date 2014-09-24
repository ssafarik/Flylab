#!/usr/bin/env python


# This program reads a .bag file and saves the contained 
# images to .png / .mov / .fmf files.
#  
# Specify the .bag file and the image topic on the command line, and
# optionally the .mov filename to write as a movie file. 
#
#   rosrun save bagconverter.py <bagfile> <imagetopic>] [filename.mov | filename.fmf]
#
# for example:
#   rosrun save bagconverter.py yourfile.bag /camera/image_rect /home/rancher/asdf.mov
# or:
#   rosrun save bagconverter.py yourfile.bag /camera/image_rect /home/rancher/asdf.fmf
#
#


import os
import subprocess
import sys

import Image
import rospy
import rosbag
import cv
import cv2
import motmot.FlyMovieFormat.FlyMovieFormat as FlyMovieFormat
import numpy as np

#sys.path.insert(0,'/opt/ros/fuerte/stacks/vision_opencv/cv_bridge/src') # There's probably a better way to find cv_bridge.
import cv_bridge

iImage = 0
cvbridge = cv_bridge.CvBridge()
fmf = None

# Get the video filename parameter.
filenameMov = None
filenameFmf = None
if (len(sys.argv)==4):
    if ('.mov' in sys.argv[3]):
        filenameMov = sys.argv[3]
    elif ('.fmf' in sys.argv[3]):
        filenameFmf = sys.argv[3]
    else:
        print('bagconverter: Third parameter must be the full path spec of the .mov or .fmf file to write.')

# Process the .bag file.    
if (len(sys.argv)>=3):
    # Get the first and second command parameters.
    bag = rosbag.Bag(sys.argv[1])
    topicToConvert = sys.argv[2]
    
    # Use the tail end of the topic as the image dir, e.g. 'camnode/image_raw' uses 'image_raw'
    dirImages = topicToConvert.split(os.sep)[-1]
    # Make sure dir exists.
    try:
        os.makedirs(dirImages)
    except OSError:
        pass
        

    
    # Read all the messages in the bag file.    
    for (topic, msg, t) in bag.read_messages(topicToConvert):
        pixels = None
        encoding = None
        
        # Handle compressed images.
        if ('compressed' in topic):
            if ('png' in msg.format):
                ext = 'png'
            elif ('jpeg' in msg.format):
                ext = 'jpg'
            else:
                ext = 'xxx'

            if ('png' in msg.format) or ('jpeg' in msg.format):
                pngdata = np.array(msg.data, 'c').view(np.uint8)
                pixels = cv2.imdecode(pngdata, flags=cv2.CV_LOAD_IMAGE_UNCHANGED)
                bpp = pixels.itemsize * 8
                format_from_bpp_dict = {8:'MONO8', 16:'MONO16', 24:'RGB8', 32:'ARGB8'}
                format = format_from_bpp_dict[bpp]
                
            # Make a filename like 'image_raw/000123.png'
            filename = '%s%s%06d.%s' % (dirImages, os.sep, iImage, ext)

            # Save the image file.
            file = open(filename,'w')
            file.write(msg.data)
            file.close()
            print ('Wrote %s' % filename)

            iImage += 1
                
        else:
            # Handle uncompressed images. 
            if msg.encoding=='mono8':
                ext = 'png'
                
                # Make a filename like 'image_raw/000123.png'
                filename = '%s%s%06d.%s' % (dirImages, os.sep, iImage, ext)
    
                matImage = cv.GetImage(cvbridge.imgmsg_to_cv(msg, 'passthrough'))
                pixels = np.array(msg.data, 'c').view(np.uint8).reshape((msg.height, msg.width))
                bpp = pixels.itemsize * 8
                format = msg.encoding.upper() # BUG: This isn't quite right, as the ROS encodings don't all match those in FlyMovieFormat.py
                
                # Save the image file.
                cv.SaveImage(filename, matImage)
                print ('Wrote %s' % filename)

                iImage += 1
            else:
                print ('Only image encoding==mono8 is supported.  This one has %s' % msg.encoding)

                
        if (filenameFmf is not None):
            if (fmf is None):
                fmf = FlyMovieFormat.FlyMovieSaver(filenameFmf, 
                                                   version=3, 
                                                   format=format, 
                                                   bits_per_pixel=bpp)
            
            # Add the frame to the .fmf
            if (pixels is not None):
                fmf.add_frame(pixels, msg.header.stamp.to_sec())


    # If requested, convert the image files to an .mov file, then delete the images.                
    if (filenameMov is not None):
        # Run avconv, and then remove all the image files.
        cmdCreateVideoFile = 'avconv -y -r 60 -i %s/%%06d.%s -same_quant -r 60 %s && rm -rf %s && echo Finished.' % (dirImages, ext, filenameMov, dirImages)
        print('Converting images to video using command:')
        print (cmdCreateVideoFile)
        try:
            processAvconv = subprocess.Popen(cmdCreateVideoFile, shell=True)
        except:
            rospy.logerr('Exception running avconv to convert to .mov')


    if (filenameFmf is not None) and (fmf is not None):
        fmf.close()
        print ('Wrote %s' % filenameFmf) 
        
                    
    if (iImage==0):
        print('Please specify a valid image topic:  bagconverter filename.bag imagetopic [filename.mov]')
        print('Topics in %s are:' % sys.argv[1])
        for (i,c) in bag._connections.iteritems():
            print(c.topic)
            
else:
    print ('Usage:  bagconverter filename.bag imagetopic [filename.mov]')
    print ('  Extracts the image files from a .bag file, and writes them to')
    print ('  disk in a subdirectory named from the imagetopic.')
    print ('  Can also then optionally convert those images to a .mov file.')
    print ('  If the optional .mov file is specified, the images will be')
    print ('  converted to a .mov file, and then images deleted.')
    
    