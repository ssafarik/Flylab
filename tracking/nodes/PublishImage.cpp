#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

//
// This ROS node takes a filename as a command-line argument, and publishes it to camera/image_rect.  
// One purpose, for example, is to send files to the Flylab tracking system, i.e. ContourGenerator.
// e.g.
// rosrun tracking publishimage ~/Pictures/0073_2180686.png
//

#define READGRAYSCALE


int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_publisher");
	
	ros::NodeHandle 				nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher 		pubImage 		= it.advertise("camera/image_rect", 1);
	ros::Publisher					pubCamerainfo	= nh.advertise<sensor_msgs::CameraInfo>("camera/camera_info", 1);
	
#ifdef READCOLOR
	cv::WImageBuffer3_b 	cvimage(cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR));
#else if READGRAYSCALE
	IplImage						*ipl = cvLoadImage(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
	
	// Invert the pixel intensities.
	uchar					*data = (uchar *)ipl->imageData;
	if (0)
		for(int i=0; i<ipl->height; i++)
			for(int j=0; j<ipl->width; j++)
				for(int k=0; k<ipl->nChannels; k++)  //loop to read for each channel
					data[i*ipl->widthStep+j*ipl->nChannels+k] = 255-data[i*ipl->widthStep+j*ipl->nChannels+k];    //inverting the image

	cv::WImageBuffer1_b 	cvimage(ipl);
#endif
	
	sensor_msgs::ImagePtr 	image = sensor_msgs::CvBridge::cvToImgMsg(cvimage.Ipl(), "mono8");
	sensor_msgs::CameraInfo	camerainfo;
	ros::Rate 				loop_rate(5);
	
	
	camerainfo.height = image->height;
	camerainfo.width = image->width;
	while (nh.ok()) 
	{
		pubImage.publish(image);
		pubCamerainfo.publish(camerainfo);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

