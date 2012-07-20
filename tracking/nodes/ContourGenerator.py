#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('tracking')
import sys
import rospy
import cv
import tf
import numpy as N
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from tracking.msg import ContourInfo
from plate_tf.srv import PlateCameraConversion

FILENAME_BACKGROUND="/cameras/background.png"



###############################################################################
###############################################################################
###############################################################################
# The class ContourGenerator subscribes to image_rect, and finds the contours of the objects in the image.
# Publishes a ContourInfo message, and several intermediary images.
#
class ContourGenerator:

    def __init__(self):
        
        # Image Initialization
        self.preinit = False
        self.initialized = False
        
        # Messages
        self.camerainfo = None
        self.subCameraInfo       = rospy.Subscriber("camera/camera_info", CameraInfo, self.CameraInfo_callback)
        self.subImageRect        = rospy.Subscriber("camera/image_rect", Image, self.Image_callback, queue_size=1, buff_size=262144, tcp_nodelay=True)
        #self.subImageRaw         = rospy.Subscriber("camera/image_raw", Image, self.ImageRaw_callback, queue_size=1, buff_size=262144, tcp_nodelay=True)
        
        self.pubImageProcessed   = rospy.Publisher("camera/image_processed", Image)
        self.pubImageBackground  = rospy.Publisher("camera/image_background", Image)
        self.pubImageForeground  = rospy.Publisher("camera/image_foreground", Image)
        self.pubContourInfo      = rospy.Publisher("ContourInfo", ContourInfo)
        
        self.tfrx = tf.TransformListener()
        
        # Contour Info
        self.contourinfo = ContourInfo()
        self.x0_list = []
        self.y0_list = []
        self.angle_list = []
        self.area_list = []
        self.ecc_list = []
        self.nContours = 0
        self.nContours_max = 20 #int(rospy.get_param("nContours_max", 2))
        self.min_ecc = 1.75
        self.minSumImage = 100
        
        # Robot Info
        #self.robot_visible = bool(rospy.get_param("robot/visible", "true"))
        #if (not self.robot_visible) and (1 < self.nContours_max):
        if (1 < self.nContours_max):
            self.nContours_max -= 1
        
        self.header = None
        self.cvimage = None
        
        
        # OpenCV
        self.max_8U = 255
        self.color_max = 255
        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
        self.storage = cv.CreateMemStorage()
        self.cvbridge = CvBridge()
        
        # Coordinate Systems
        self.frameidOutput = rospy.get_param("frameid_contours", "ImageRect")
        
        self.ptsOriginImage = PointStamped()
        self.ptsOriginImage.header.frame_id = "ImageRect"
        self.ptsOriginImage.point.x = 0
        self.ptsOriginImage.point.y = 0
        
        self.ptsOriginPlate = PointStamped()
        self.ptsOriginPlate.header.frame_id = "Plate"
        self.ptsOriginPlate.point.x = 0
        self.ptsOriginPlate.point.y = 0
        

        # Mask Info
        self.radiusMask = int(rospy.get_param("camera/mask/radius", 25))

        self.timePrev = rospy.Time.now().to_sec()
        self.preinit = True
        

    def InitializeImages(self):
        if (self.camerainfo is not None) and (self.cvimage is not None):
            self.sizeImageRect = cv.GetSize(self.cvimage)
            
            # ROI Setup
            self.rectImage = (int(self.ptsOriginImage.point.x),
                              int(self.ptsOriginImage.point.y),
                              int(self.camerainfo.width),
                              int(self.camerainfo.height))
            
            
            self.cvimageProcessed         = cv.CreateImage(self.sizeImageRect, cv.IPL_DEPTH_8U,3)
            cv.SetImageROI(self.cvimageProcessed, self.rectImage)
            self.cvimageProcessed2        = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,3)
            self.cvimageMask              = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,1)
            self.cvimageBackground        = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,1)
            self.cvimageForeground        = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,1)
            self.cvimageForeground_binary = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,1)
            #self.cvimageDilate            = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,1)
            #self.cvimageErode             = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,1)
            self.cvimageZeros             = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,1)
            self.cvimageContour           = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,1)
            self.cvimageContourDisplay    = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,3)
            cv.Zero(self.cvimageZeros)
            
            b = False
            while not b:
                try:
                    self.tfrx.waitForTransform('Plate', 
                                               self.ptsOriginImage.header.frame_id, 
                                               self.ptsOriginImage.header.stamp, 
                                               rospy.Duration(1.0))
                    
                    self.ptsOriginMask = self.tfrx.transformPoint('Plate', self.ptsOriginImage)
                    self.ptsOriginMask.point.x = -self.ptsOriginMask.point.x
                    self.ptsOriginMask.point.y = -self.ptsOriginMask.point.y
                    #rospy.logwarn(self.ptsOriginMask.point)
                    b = True
                except tf.Exception, e:
                    rospy.logwarn('Exception transforming mask frames %s->Plate:  %s' % (self.ptsOriginImage.header.frame_id, e))
                    
            cv.Zero(self.cvimageMask)
            cv.Circle(self.cvimageMask,
                      (int(self.ptsOriginMask.point.x),int(self.ptsOriginMask.point.y)),
                      int(self.radiusMask), 
                      self.color_max, 
                      cv.CV_FILLED)
            #rospy.logwarn('ROI=%s' % [int(self.ptsOriginPlate.point.x),int(self.ptsOriginPlate.point.y), int(self.radiusMask)])
            
            # Create Background image
            # First image is background unless one can be loaded
            #q = cv.GetSize(self.cvimage)
            #rospy.logwarn('size1=%s' % [q[0],q[1]])
            cv.SetImageROI(self.cvimage, self.rectImage)
            
            #q = cv.GetSize(self.cvimage)
            #rospy.logwarn('size2=%s' % [q[0],q[1]])
            try:
                self.cvimageBackground = cv.LoadImage(FILENAME_BACKGROUND, cv.CV_LOAD_IMAGE_GRAYSCALE)
            except:
                rospy.logwarn ('Saving new background image %s' % FILENAME_BACKGROUND)
                cv.And(self.cvimage, self.cvimageMask, self.cvimageBackground)
                cv.SaveImage(FILENAME_BACKGROUND, self.cvimageBackground)
                #q = cv.GetSize(self.cvimageBackground)
                #rospy.logwarn('size3=%s' % [q[0],q[1]])
              
            self.initialized = True
            

    def CameraInfo_callback (self, msgCameraInfo):
        if not self.preinit:
            return
        
        self.camerainfo = msgCameraInfo
        
        if not self.initialized:
            self.InitializeImages()
            

    
    def MmFromPixels (self, xIn):
        response = self.camera_from_plate(xIn, xIn)
        return (response.Xdst, response.Ydst)
        
        
    def PixelsFromMm (self, xIn):
        ptsIn = PointStamped()
        ptsIn.header.frame_id = "Plate"
        ptsIn.point.x = 0.0
        ptsIn.point.y = 0.0
        ptsOrigin = self.tfrx.transformPoint("ImageRect", ptsIn)        
        
        ptsIn.point.x = xIn
        ptsIn.point.y = xIn
        ptsOut = self.tfrx.transformPoint("ImageRect", ptsIn)
                
        return (ptsOut.point.x - ptsOrigin.point.x)
        
        
    # Given the various image moments, compute the angle and eccentricity.
    # Angle is set to NaN when the image is circular. 
    def FindAngleEcc(self, Uu20, Uu11, Uu02):
        angle = float('NaN')
        ecc = 1.0
        
        if Uu11 != 0: # Div by zero.
            inside = Uu20*Uu20 + 4*Uu11*Uu11 - 2*Uu20*Uu02 + Uu02*Uu02
            if inside >= 0: # Complex answer.
                inside = N.sqrt(inside)
                evalA = 0.5*(Uu20+Uu02-inside)
                evalB = 0.5*(Uu20+Uu02+inside)
                evecA1 = (-Uu20+Uu02+inside)/(-2*Uu11)
                evecB1 = (-Uu20+Uu02-inside)/(-2*Uu11)
                rise = 1
                try:
                    if evalB < evalA:
                        run = evecA1
                        ecc = evalA/evalB
                      
                    else:
                        run = evecB1
                        ecc = evalB/evalA
                      
                    angle = -N.arctan2(rise, run)
                    #rospy.loginfo ('IP angle=%s, rise/run=%s/%s' % (angle, rise,run))

                except:
                    rospy.logwarn ('Exception in FindAngleEcc()')
                    pass
        
        #if N.isnan(angle):
        #    angle = 0.0
            #rospy.logwarn('isnan(angle): %0.4f/%0.4f=%s' % (rise,run,angle))

#        A = N.array([[Uu20, Uu11],[Uu11, Uu20]])
#        (l,V) = N.linalg.eig(A)
#        L = N.diag(l)
#        iMinor = N.argmin(abs(l))
#        iMajor = N.argmax(abs(l))
#        axisMinor = V.T[iMinor]
#        axisMajor = V.T[iMajor]
#        rospy.logwarn('axisMajor=%s' % axisMajor)
#        angleMajor = N.angle(N.complex(axisMajor[0],axisMajor[1]))
#        angle = angleMajor
#        
#        ecc = N.sqrt(1-l[1]/l[0])
#        #rospy.logwarn('ecc: %0.2f, %0.2f' % (ecc1,ecc))
              
        return angle, ecc
        

    def ContourFromMoments(self, moments):
        Mu00 = cv.GetSpatialMoment(moments,0,0)
        Mu10 = cv.GetSpatialMoment(moments,1,0)
        Mu01 = cv.GetSpatialMoment(moments,0,1)
        
        if Mu00 != 0:
            x = Mu10/Mu00
            y = Mu01/Mu00
        
        else:
            x = None
            y = None
          
        Uu11 = cv.GetCentralMoment(moments,1,1)
        Uu20 = cv.GetCentralMoment(moments,2,0)
        Uu02 = cv.GetCentralMoment(moments,0,2)
        area = Mu00
        angle, ecc = self.FindAngleEcc(Uu20, Uu11, Uu02)
        #rospy.logwarn('u: %s, %s, %s, %s' % (Mu00, Uu20, Uu11, Uu02))

        return x, y, area, angle, ecc
    
    
    def DrawAngleLine(self, cvimage, x0, y0, angle, ecc, length):
        if (not N.isnan(angle)) and (self.min_ecc < ecc):
            height, width = cv.GetSize(cvimage)
            y0 = height-y0
            
            # line segment for orientation
            xmin = 0
            ymin = 0
            xmax = width-1
            ymax = height-1
    
            r = length/2
            x1 = x0 - r * N.cos(angle)
            y1 = y0 - r * N.sin(angle)
            x2 = x0 + r * N.cos(angle)
            y2 = y0 + r * N.sin(angle)
            # BUG: What happens if x,y are offscreen?
            
            
            #rospy.logwarn ('IP %s, %s, %s, %s, %s, %s, %s, %s' % (x1, width, xg, xo, x0, y0, angle, ecc))
            cv.Line(cvimage, 
                    (int(x1),int(y1)),
                    (int(x2),int(y2)), 
                    cv.CV_RGB(0,self.color_max,0))

    
    def AppendContour(self, cvseqContours):
        #seq = cv.FindContours(self.cvimageContour, cv.CreateMemStorage()) # Finds contours from imageContour, not imageForeground_binary.
        moments = cv.Moments(cvseqContours)
        (x, y, area, angle, ecc) = self.ContourFromMoments(moments)
        
        
        # Save contour info
        ptContour = PointStamped()
        ptContour.header.frame_id = "ImageRect"
        ptContour.point.x = x
        ptContour.point.y = y
        if x is not None:
            try:
                self.ptsOutput = self.tfrx.transformPoint(self.frameidOutput, ptContour)
                self.x0_list.append(self.ptsOutput.point.x)
                self.y0_list.append(self.ptsOutput.point.y)
                self.angle_list.append(angle)
                self.area_list.append(area)
                self.ecc_list.append(ecc)
                self.nContours += 1

            except tf.Exception, e:
                rospy.logwarn ('Exception transforming point to frame=%s from frame=%s: %s' % (self.frameidOutput, ptContour.header.frame_id, e))
                self.ptsOutput = PointStamped()
            except TypeError, e:
                rospy.logwarn ('Exception transforming point to frame=%s from frame=%s: %s' % (self.frameidOutput, ptContour.header.frame_id, e))
                
        


    def ContourinfoFromImage(self, cvimage):
        self.x0_list = []
        self.y0_list = []
        self.angle_list = []
        self.area_list = []
        self.ecc_list = []
        
        # Find contours
        sumImage = cv.Sum(cvimage)
        if self.minSumImage < sumImage[0]:
            cvseqContours = cv.FindContours(cvimage, self.storage, mode=cv.CV_RETR_CCOMP)
        else:
            cvseqContours = None
        
        # Process contours
        self.nContours = 0
        if cvseqContours is not None:
            while True:
                self.AppendContour(cvseqContours) # self.nContours++ gets incremented inside function.
                    
                if (cvseqContours.h_next()) and (self.nContours < self.nContours_max):
                    cvseqContours = cvseqContours.h_next()
                else:
                    break
        
            # Reset cvseqContours to the start.
            while cvseqContours.h_prev():
                cvseqContours = cvseqContours.h_prev()
            
        
        # display_text = str(self.nContours)
        # cv.PutText(self.im_display, display_text,(25,25), self.font, cv.CV_RGB(self.color_max,0,0))
        
        # Put contours into contourinfo.
        contourinfo = ContourInfo()
        contourinfo.header.stamp = self.header.stamp #rospy.Time.now()
        contourinfo.header.frame_id = self.frameidOutput # i.e. Camera
        
        if self.nContours != 0:
            contourinfo.x = self.x0_list
            contourinfo.y = self.y0_list
            contourinfo.angle = self.angle_list
            contourinfo.area = self.area_list
            contourinfo.ecc = self.ecc_list
        
        # Remove duplicates
        if self.nContours != 0:
            # Repackage the data.
            contours = []
            for iContour in range(self.nContours):
                contours.append([contourinfo.x[iContour], 
                                 contourinfo.y[iContour], 
                                 contourinfo.angle[iContour], 
                                 contourinfo.area[iContour], 
                                 contourinfo.ecc[iContour]])
            
            # Remove the dups.
            contours = sorted(tuple(contours))
            contours = [x for i, x in enumerate(contours) if (not i) or (N.linalg.norm(N.array(x[0:2])-N.array(contours[i-1][0:2]))>0.1)]
        
            
            # Repackage the cleaned data.
            self.nContours = len(contours)
            contourinfo.x = []
            contourinfo.y = []
            contourinfo.angle = []
            contourinfo.area = []
            contourinfo.ecc = []
            for iContour in range(self.nContours):
                contourinfo.x.append(contours[iContour][0])
                contourinfo.y.append(contours[iContour][1])
                contourinfo.angle.append(contours[iContour][2])
                contourinfo.area.append(contours[iContour][3])
                contourinfo.ecc.append(contours[iContour][4])
            
        return contourinfo, cvseqContours    
        

    def ImageRaw_callback(self, image):
        rospy.logwarn('ImageRaw_callback(now-prev=%s)' % (rospy.Time.now().to_sec()-self.timePrev))
        self.timePrev = rospy.Time.now().to_sec()


    def Image_callback(self, image):
#        rospy.logwarn('Image_callback(now-prev=%s)' % (rospy.Time.now().to_sec()-self.timePrev))
#        self.timePrev = rospy.Time.now().to_sec()

        if not self.preinit:
            return

        self.header = image.header
        
        # Convert ROS image to OpenCV image
        try:
            self.cvimage = cv.GetImage(self.cvbridge.imgmsg_to_cv(image, "passthrough"))
        except CvBridgeError, e:
          rospy.logwarn ('Exception %s' % e)
        
        if not self.initialized:
            self.InitializeImages()

        if self.initialized:        
            radiusMask = int(rospy.get_param("camera/mask/radius", 25)) 
            if radiusMask != self.radiusMask:
                self.radiusMask = radiusMask 
                cv.Zero(self.cvimageMask)
                cv.Circle(self.cvimageMask,
                          (int(self.ptsOriginMask.point.x),int(self.ptsOriginMask.point.y)),
                          int(self.radiusMask), 
                          self.color_max, 
                          cv.CV_FILLED)
            
            cv.SetImageROI(self.cvimage, self.rectImage)
    
            
            # Look for new diff_threshold value
            self.diff_threshold = int(rospy.get_param("camera/diff_threshold", 50))
            
            # Apply mask and Subtract background
            #q1 = cv.GetSize(self.cvimage)
            #q2 = cv.GetSize(self.cvimageMask)
            #rospy.logwarn ('%s' % [[q1[0],q1[1]],[q2[0],q2[1]]])
            cv.And(self.cvimage, self.cvimageMask, self.cvimage)
            cv.AbsDiff(self.cvimage, self.cvimageBackground, self.cvimageForeground)
            
            # Threshold
            cv.Threshold(self.cvimageForeground, 
                         self.cvimageForeground_binary, 
                         self.diff_threshold, 
                         self.max_8U, 
                         cv.CV_THRESH_BINARY)
            cv.Threshold(self.cvimageForeground, 
                         self.cvimageForeground, 
                         self.diff_threshold, 
                         self.max_8U, 
                         cv.CV_THRESH_TOZERO)
            # cv.Dilate(self.cvimageForeground,self.cvimageDilate)
            # cv.Erode(self.cvimageDilate,self.cvimageErode)
            # cv.ShowImage("Dilate Plus Erode Image", self.cvimageErode)
    
            
            # Get the ContourInfo.
            (self.contourinfo, self.cvseqContours) = self.ContourinfoFromImage(self.cvimageForeground_binary)
            #rospy.logwarn ('IP self.contourinfo=\n%s\n, self.cvseqContours\n=%s' % (self.contourinfo,self.cvseqContours))
            
            #if len(self.contourinfo.x)>0:
            self.pubContourInfo.publish(self.contourinfo)
            #rospy.logwarn('Published ContourInfo')
            
            # Convert to color for display image
            if self.pubImageProcessed.get_num_connections() > 0:
                cv.CvtColor(self.cvimage, self.cvimageProcessed, cv.CV_GRAY2RGB)
                
                # Draw contours on Processed image.
                if self.cvseqContours:
                    cv.DrawContours(self.cvimageProcessed, self.cvseqContours, cv.CV_RGB(0,0,self.color_max), cv.CV_RGB(0,self.color_max,0), 1, 1)
                
                
            
            # Publish processed image
            if self.pubImageProcessed.get_num_connections() > 0:
                try:
                    cv.Copy(self.cvimageProcessed, self.cvimageProcessed2)
                    image2 = self.cvbridge.cv_to_imgmsg(self.cvimageProcessed2, "passthrough")
                    image2.header = image.header
                    self.pubImageProcessed.publish(image2)
                    del image2
                except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                    rospy.logwarn ('Exception %s' % e)
            
            # Publish background image
            if self.pubImageBackground.get_num_connections() > 0:
                try:
                    image2 = self.cvbridge.cv_to_imgmsg(self.cvimageBackground, "passthrough")
                    image2.header.stamp = image.header.stamp
                    self.pubImageBackground.publish(image2)
                    del image2
                except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                    rospy.logwarn ('Exception %s' % e)
              
            # Publish foreground image
            if self.pubImageForeground.get_num_connections() > 0:
                try:
                    image2 = self.cvbridge.cv_to_imgmsg(self.cvimageForeground, "passthrough")
                    image2.header.stamp = image.header.stamp
                    self.pubImageForeground.publish(image2)
                    del image2
                except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                    rospy.logwarn ('Exception %s' % e)
              


    def Main(self):
        rospy.spin()
      

def main(args):
    rospy.init_node('ContourGenerator') #, anonymous=True)
    try:
        ip = ContourGenerator()
        ip.Main()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down")
    cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
  
