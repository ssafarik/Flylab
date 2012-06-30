#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include <cmath>
#include <sstream>
#include <boost/thread.hpp>
#include <NIDAQmx.h>


#ifndef NULL
#define NULL 0
#endif

#ifndef FALSE
#define FALSE 0
#define TRUE (!FALSE)
#endif

#define MAX(a,b) ((a)<(b) ? (b) : (a))

#define LASERON	TRUE
#define LASEROFF FALSE

#define NBUFFERCOPIES 2

int32 					g_nPointsBuffer=0;
int32 					g_nPointsBufferRegistered = -1;
int32 					g_nPointsBufferMax=0;
int32					g_nPointclouds = 0;

float64 			   *g_pBufferPoints=NULL;
int32 				   *g_pBufferBlanking=NULL;

boost::mutex 			g_mutex; // To prevent the various callbacks from colliding.
sensor_msgs::PointCloud g_pointcloud;

int						g_bStarted=FALSE;

float64					g_hzPoint = 16384.0;  // Delay of laser movement = onboardbufsize/g_hzPoint, e.g. 4095/16384 = 0.25 seconds.
float64					g_hzPointcloud = 0.0;
float64					g_hzUSB = 70.0;

int32   e=0;
char    errBuff[2048]={'\0'};

int32 CVICALLBACK OnEveryNSamples_callback (TaskHandle hTask, int32 eventType, uInt32 nSamples, void *callbackData);


// ************************************
// GalvoPointCloud_callback()
// Save the given pointcloud to g_pointcloud
//
void GalvoPointCloud_callback(const sensor_msgs::PointCloud::ConstPtr& pointcloud)
{
	//ROS_WARN("Received Pointcloud, len=%d.", pointcloud->points.size());
	
	boost::lock_guard<boost::mutex> lock(g_mutex); // Lock the global pointcloud.
	g_pointcloud = sensor_msgs::PointCloud(*pointcloud); // Copy the given pointcloud.
	
}


// ************************************
// ConvertPointsFromPointcloud()
// Take the points in the pointcloud structure, and put them into a float64 array.
//
void ConvertPointsFromPointcloud()
{
	int			i=0;
	int			iPoint=0;
	int			iPointcloud=0;
	int32		nPointsPerCloud=0;

	boost::lock_guard<boost::mutex> lock(g_mutex);

	nPointsPerCloud = MAX(1,g_pointcloud.points.size());
	g_hzPointcloud = g_hzPoint / (float64)nPointsPerCloud;
	
	
	g_nPointclouds = (int32)MAX(1.0, ceil(g_hzPointcloud/g_hzUSB) + 1.0);
	g_nPointsBuffer = g_nPointclouds * nPointsPerCloud;
	//ROS_WARN("nPointsPerCloud=%lu, g_hzPointcloud=%0.2f, g_hzUSB=%0.2f, g_nPointclouds=%lu, g_nPointsBuffer=%lu", 
	//	nPointsPerCloud,
	//	g_hzPointcloud, 
	//	g_hzUSB, 
	//	g_nPointclouds, 
	//	g_nPointsBuffer);
	
	// Realloc old memory if more points now.  This is one buffer containing multiple copies of pointcloud.  The DAQ will use this buffer NBUFFERCOPIES times.
	if (g_nPointsBufferMax < g_nPointsBuffer)
	{
		g_nPointsBufferMax = g_nPointsBuffer;

		if (g_pBufferPoints != NULL)
			delete g_pBufferPoints;
		g_pBufferPoints = new float64[g_nPointsBuffer * 2]; // Two values per point, i.e. (x,y).

		if (g_pBufferBlanking != NULL)
			delete g_pBufferBlanking;
		g_pBufferBlanking = new int32[g_nPointsBuffer];
	}

	
	// Copy the pointcloud points to global point buffer.
	iPoint = 0;		// The cumulative point in the buffer.
	for (iPointcloud=0; iPointcloud<g_nPointclouds; iPointcloud++)
	{
		if (g_pointcloud.points.size()>0)
			for (i=0; i<nPointsPerCloud; i++)
			{
				// Copy the (x,y) values.
				g_pBufferPoints[iPoint*2]   = (float64)g_pointcloud.points[i].x;
				g_pBufferPoints[iPoint*2+1] = (float64)g_pointcloud.points[i].y;
			
				// Copy the z-blanking value.
				g_pBufferBlanking[iPoint] = (g_pointcloud.points[i].z != 0.0) ? LASERON : LASEROFF;
		
				iPoint++;

				//ROS_WARN("(%0.4f, %0.4f)", g_pointcloud.points[i].x, g_pointcloud.points[i].y);
			}
		else
		{
			// Use (0,0).
			g_pBufferPoints[iPoint*2] = 0.0;
			g_pBufferPoints[iPoint*2+1] = 0.0;
			
			// Copy the z-blanking value.
			g_pBufferBlanking[iPoint] = LASERON;
	
			iPoint++;

			//ROS_WARN("(default point)");
		}
			
	}
}


// ************************************
// WritePoints()
// Write more points to the DAQ.  Uses the points already computed in g_pBufferPoints and g_pBufferBlanking.
void WritePoints(TaskHandle hTask)
{
	int32		nPointsWritten = 0;

	// Write the buffer.
	e=DAQmxWriteAnalogF64 (hTask, g_nPointsBuffer, FALSE, 0.0, DAQmx_Val_GroupByScanNumber, g_pBufferPoints, &nPointsWritten, NULL);
	if (DAQmxFailed(e))
	{
		DAQmxGetExtendedErrorInfo(errBuff,2048);
		ROS_WARN(errBuff);
	}

}



// ************************************
// RegisterCallback()
// Register (if necessary) a callback function to get called when the DAQ needs more data.
//
int RegisterCallback (TaskHandle hTask)
{
	int rv=FALSE;
	
	if (g_nPointsBuffer != g_nPointsBufferRegistered)
	{
		ROS_WARN("Resetting task, %d, pc.size=%d", g_nPointsBuffer, g_pointcloud.points.size());

		g_nPointsBufferRegistered = g_nPointsBuffer;
		if (g_bStarted)
		{
			e=DAQmxStopTask (hTask);
			if (DAQmxFailed(e))
			{
				DAQmxGetExtendedErrorInfo(errBuff,2048);
				ROS_WARN(errBuff);
			}
			else
				g_bStarted = FALSE;
		}

		// Unregister.
		e=DAQmxRegisterEveryNSamplesEvent (hTask, DAQmx_Val_Transferred_From_Buffer, g_nPointsBufferRegistered, 0, NULL,                     NULL);
		if (DAQmxFailed(e))
		{
			DAQmxGetExtendedErrorInfo(errBuff,2048);
			ROS_WARN(errBuff);
		}

		// Resize the output buffer.
		e=DAQmxCfgOutputBuffer (hTask, NBUFFERCOPIES*g_nPointsBufferRegistered);
		if (DAQmxFailed(e))
		{
			DAQmxGetExtendedErrorInfo(errBuff,2048);
			ROS_WARN(errBuff);
		}

		// Reregister.
		e=DAQmxRegisterEveryNSamplesEvent (hTask, DAQmx_Val_Transferred_From_Buffer, g_nPointsBufferRegistered, 0, OnEveryNSamples_callback, NULL);
		if (DAQmxFailed(e))
		{
			DAQmxGetExtendedErrorInfo(errBuff,2048);
			ROS_WARN(errBuff);
		}
		
		rv = TRUE;
	}
	
	return rv;
}


//*********************************************
// OnEveryNSamples_callback()
// Called when the DAQ wants more data.
//
int32 CVICALLBACK OnEveryNSamples_callback (TaskHandle hTask, int32 eventType, uInt32 nSamples, void *callbackData)
{
	if (hTask)
	{
		ConvertPointsFromPointcloud();

		if (RegisterCallback(hTask))
			for (int i=0; i<(NBUFFERCOPIES-1); i++)
				WritePoints(hTask);
		
		WritePoints(hTask);

		if (!g_bStarted)
		{
			e=DAQmxStartTask (hTask);
			if (!DAQmxFailed(e))
				g_bStarted = TRUE;
			else
			{
				DAQmxGetExtendedErrorInfo(errBuff,2048);
				ROS_WARN(errBuff);
			}
		}
	}
		
	return 0;	
}


//*********************************************
int main(int argc, char **argv)
{
	ros::init(argc, argv, "GalvoDriver");
	ros::NodeHandle node;
	TaskHandle 	 	hTask;

	char 		szTask[]="OutputPointList";
	char		szPhysicalChannel[]="Dev1/ao0:1";
	float64		points[]={0.0, 0.0, 0.0, 0.0};

	ros::Subscriber subGalvoPoints = node.subscribe("GalvoDriver/pointcloud", 2, GalvoPointCloud_callback);
	

	ROS_WARN ("Listening for points on ROS topic GalvoDriver/pointcloud...");

	e=DAQmxCreateTask (szTask, &hTask);
	e=DAQmxCreateAOVoltageChan (hTask, szPhysicalChannel, NULL, -10.0, +10.0, DAQmx_Val_Volts, NULL);
	e=DAQmxCfgSampClkTiming (hTask, "OnboardClock", g_hzPoint, DAQmx_Val_Rising, DAQmx_Val_ContSamps, g_nPointsBuffer);
	e=DAQmxSetWriteAttribute (hTask, DAQmx_Write_RegenMode, DAQmx_Val_DoNotAllowRegen);
	if (DAQmxFailed(e))
	{
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
		ROS_WARN(errBuff);
	}
	
	ConvertPointsFromPointcloud();
	e=DAQmxCfgOutputBuffer (hTask, NBUFFERCOPIES*g_nPointsBuffer);
	if (DAQmxFailed(e))
	{
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
		ROS_WARN(errBuff);
	}
	for (int i=0; i<NBUFFERCOPIES; i++)
		WritePoints(hTask);
	RegisterCallback(hTask);
	if (!g_bStarted)
	{
		e=DAQmxStartTask (hTask);
		if (!DAQmxFailed(e))
			g_bStarted = TRUE;
		else
		{
			DAQmxGetExtendedErrorInfo(errBuff,2048);
			ROS_WARN(errBuff);
		}
	}
	uInt32 data;
	DAQmxSetBufOutputOnbrdBufSize(hTask, 2048);
	DAQmxGetBufOutputOnbrdBufSize(hTask, &data);
	ROS_WARN("Onboard bufsize=%u", data);
	ros::spin();
	
	e=DAQmxStopTask (hTask);
	e=DAQmxClearTask (hTask);
	e=DAQmxResetDevice ("Dev1");


	return 0;
}
