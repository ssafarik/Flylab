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

import cv_bridge

iImage = 0
cvbridge = cv_bridge.CvBridge()
fmf = None
extImage = None
relpathImages = None

# Get the video filename parameter.
fullpathnameextMov = None
fullpathnameextFmf = None

# Process the .bag file.    
if (len(sys.argv)>=3):
    fullpathnameextBag = os.path.realpath(sys.argv[1])
    
    # Open the .bag file.
    bag = rosbag.Bag(fullpathnameextBag)

    # Get the list of topics in the .bag file.
    topicsBag_list = []    
    for (i,c) in bag._connections.iteritems():
        topicsBag_list.append(c.topic)


    # Get the topic name the user wants, with some possible variants.
    topicVariants_list = [        sys.argv[2],
                                  sys.argv[2] + os.sep + 'compressed',  
                         os.sep + sys.argv[2],  
                         os.sep + sys.argv[2] + os.sep + 'compressed'
                          ]
    topicVariants_list.reverse()  
        
    # Try to find the requested topic, or a variant, in the .bag file.
    topicRequested = None
    while (topicRequested not in topicsBag_list) and (len(topicVariants_list)>0):
        topicRequested = topicVariants_list.pop()

    if (topicRequested in topicsBag_list):
        topicRequested_parts = topicRequested.split(os.sep)
        print ('Using topic=%s' % topicRequested)
        
        
        # Get filename parts for the .bag file.
        (fullpathBag, nameextBag) = os.path.split(fullpathnameextBag)
        (nameBag, extBag) = os.path.splitext(nameextBag)
    
    
        # Get filename parts for the image files.
        # Use the tail end of the topic as the image dir, 
        # e.g. 'camnode/image_raw' uses 'image_raw'
        # e.g. 'camnode/image_raw/compressed' also uses 'image_raw'
        if (topicRequested_parts[-1]=='compressed'):
            relpathImages = topicRequested_parts[-2]
        else:
            relpathImages = topicRequested_parts[-1]
        fullpathImages = '%s%s%s' % (fullpathBag, os.sep, relpathImages)
        
    
        # Get filename parts for the timestamps.csv file.
        fullpathCsv = '%s%s%s' % (fullpathBag, os.sep, relpathImages)
        nameCsv = 'timestamps' # '%s_timestamps' % nameBag
        extCsv = 'csv'
    
    
        # Get the filename parts of the .mov or .fmf file.  If location not specified, then put it with the images.
        if (len(sys.argv)==4):
            (pathVideo, nameextVideo) = os.path.split(sys.argv[3])
            if (len(pathVideo)==0): # unspecified path.
                fullpathnameextVideo = '%s%s%s' % (fullpathBag, os.sep, nameextVideo)   # Put it with the .bag file.
            else:
                fullpathnameextVideo = os.path.realpath(sys.argv[3])                    # Put it where specified.
                
            if ('.mov' in sys.argv[3]):
                fullpathnameextMov = fullpathnameextVideo
            elif ('.fmf' in sys.argv[3]):
                fullpathnameextFmf = fullpathnameextVideo
            else:
                print('bagconverter: Third parameter must be the full path spec of the .mov or .fmf file to write.')
    
        fullpathnameextCsv = '%s%s%s.%s' % (fullpathCsv, os.sep, nameCsv, extCsv)
    
        
        # Make sure dir exists.
        try:
            os.makedirs(fullpathImages)
        except OSError:
            pass
            
        # Create the timestamps.csv file
        fidCsv = open(fullpathnameextCsv, 'w')
        print('Saving timestamps to .csv file:  %s' % fullpathnameextCsv)
    
        # Write a header line to the .csv file.
        #fidCsv.write('filename, timestamp\n')
        
        
        
        # Read all the messages in the bag file.    
        for (topic, msg, t) in bag.read_messages(topicRequested):
            pixels = None
            encoding = None
            
            nameImage = '%08d' % iImage
    
            # Handle compressed images.
            if ('compressed' in topic):
                if ('png' in msg.format):
                    extImage = 'png'
                elif ('jpeg' in msg.format):
                    extImage = 'jpg'
                    
    
                if ('png' in msg.format) or ('jpeg' in msg.format):
                    pngdata = np.array(msg.data, 'c').view(np.uint8)
                    pixels = cv2.imdecode(pngdata, flags=cv2.CV_LOAD_IMAGE_UNCHANGED)
                    bpp = pixels.itemsize * 8
                    format_from_bpp_dict = {8:'MONO8', 16:'MONO16', 24:'RGB8', 32:'ARGB8'}
                    format = format_from_bpp_dict[bpp]
                    
                # Make a filename like '/home/user/bagfiles/image_raw/000123.png'
                fullpathnameextImage = '%s%s%s%s%s.%s' % (fullpathBag, os.sep, relpathImages, os.sep, nameImage, extImage)
    
                # Save the image file.
                file = open(fullpathnameextImage,'w')
                file.write(msg.data)
                file.close()
                print ('Wrote %s' % fullpathnameextImage)
    
                iImage += 1
                    
            else:
                # Handle uncompressed images. 
                if msg.encoding=='mono8':
                    extImage = 'png'
                    
                    # Make a filename like '/home/user/bagfiles/image_raw/000123.png'
                    fullpathnameextImage = '%s%s%s%s%s.%s' % (fullpathBag, os.sep, relpathImages, os.sep, nameImage, extImage)
        
                    matImage = cv.GetImage(cvbridge.imgmsg_to_cv(msg, 'passthrough'))
                    pixels = np.array(msg.data, 'c').view(np.uint8).reshape((msg.height, msg.width))
                    bpp = pixels.itemsize * 8
                    format = msg.encoding.upper() # BUG: This isn't quite right, as the ROS encodings don't all match those in FlyMovieFormat.py
                    
                    # Save the image file.
                    cv.SaveImage(fullpathnameextImage, matImage)
                    print ('Wrote %s' % fullpathnameextImage)
    
                    iImage += 1
                else:
                    print ('Only image encoding==mono8 is supported.  This one has %s' % msg.encoding)
    
                    
            if (fullpathnameextFmf is not None):
                if (fmf is None):
                    fmf = FlyMovieFormat.FlyMovieSaver(fullpathnameextFmf, 
                                                       version=3, 
                                                       format=format, 
                                                       bits_per_pixel=bpp)
                
                # Add the frame to the .fmf
                if (pixels is not None):
                    fmf.add_frame(pixels, msg.header.stamp.to_sec())
    
            # Write the filename,timestamp info the the .csv file.
            fidCsv.write('%s.%s, %0.9f\n' % (nameImage, extImage, msg.header.stamp.to_sec()))
    
    
        # If requested, convert the image files to an .mov file, then delete the images.                
        if (relpathImages is not None) and (extImage is not None) and (fullpathnameextMov is not None):
            # Run avconv.
            #cmdCreateVideoFile = 'avconv -y -r 60 -i %s/%%08d.%s -same_quant -r 60 %s && rm -rf %s && echo Finished.' % (fullpathImages, extImage, fullpathnameextMov, fullpathImages)
            cmdCreateVideoFile = 'avconv -y -r 60 -i %s/%%08d.%s -same_quant -r 60 %s && echo Finished.' % (fullpathImages, extImage, fullpathnameextMov)
            print('Converting images to video using command:')
            print (cmdCreateVideoFile)
            try:
                processAvconv = subprocess.Popen(cmdCreateVideoFile, shell=True)
            except:
                rospy.logerr('Exception running avconv to convert to .mov')
    
    
        if (fullpathnameextFmf is not None) and (fmf is not None):
            fmf.close()
            print ('Wrote %s' % fullpathnameextFmf) 
            
                        
        fidCsv.close()
        print('Wrote timestamps to .csv file:  %s' % fullpathnameextCsv)

    
    else:
        print('Please specify a valid image topic:  bagconverter filename.bag imagetopic [filename.mov]')
        print('Topics in %s are:' % fullpathnameextBag)
        for (i,c) in bag._connections.iteritems():
            print(c.topic)

            
else:
    print ('Usage:  bagconverter filename.bag imagetopic [filename.mov | filename.fmf]')
    print ('  Extracts the image files from a .bag file, and writes them to')
    print ('  disk in a subdirectory named from the imagetopic.')
    print ('  Can also then optionally convert those images to a .mov file.')
    print ('  If the optional .mov or .fmf file is specified, the images will be')
    print ('  converted to video.')
    
    