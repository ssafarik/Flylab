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


uInt64 					g_nPointsBuffer=0;
uInt64 					g_nPointsBufferRegistered = -1;
uInt64 					g_nPointsBufferMax=0;
float64 			   *g_pBufferPoints=NULL;
int32 				   *g_pBufferBlanking=NULL;
boost::mutex 			g_mutex;
sensor_msgs::PointCloud g_pointcloud;
int						g_bStarted=FALSE;

float64					g_hzPoint = 10000.0;
int						g_nPointclouds = 1;
float64					g_hzPointcloud = 20.0;
float64					g_hzUSB = 20.0;

int32   e=0;
char    errBuff[2048]={'\0'};

int32 CVICALLBACK OnEveryNSamples_callback (TaskHandle hTask, int32 eventType, uInt32 nSamples, void *callbackData);


// ************************************
// GalvoPointCloud_callback()
// Save the given pointcloud to g_pointcloud
//
void GalvoPointCloud_callback(const sensor_msgs::PointCloud::ConstPtr& pointcloud)
{
	ROS_WARN("Received Pointcloud.");
	boost::lock_guard<boost::mutex> lock(g_mutex);
	g_pointcloud = sensor_msgs::PointCloud(*pointcloud);
	
}


// ************************************
// ConvertPointsFromPointcloud()
// Take the points in the pointcloud structure, and put them into a float64 array.
//
void ConvertPointsFromPointcloud()
{
	int			i=0;
	int			iPoint=0;
	int			iBlanking=0;
	int			iPointcloud=0;
	int			nPoints=0;

	
	boost::lock_guard<boost::mutex> lock(g_mutex);

	nPoints = MAX(1,g_pointcloud.points.size());
	g_hzPointcloud = g_hzPoint / (float64)nPoints;
	
	
	g_nPointclouds = (int)MAX(1.0, ceil(g_hzPointcloud/g_hzUSB) + 1.0);
	g_nPointsBuffer = g_nPointclouds * nPoints;
	
	// Realloc old memory if more points now.  This is one buffer containing multiple pointclouds.  The DAQ will use this buffer twice.
	if (g_nPointsBufferMax < g_nPointsBuffer)
	{
		g_nPointsBufferMax = g_nPointsBuffer;

		if (g_pBufferPoints != NULL)
			delete g_pBufferPoints;
		g_pBufferPoints = new float64[g_nPointsBuffer * 2]; // Two values per point (x,y).

		if (g_pBufferBlanking != NULL)
			delete g_pBufferBlanking;
		g_pBufferBlanking = new int32[g_nPointsBuffer];
	}

	
	// Copy the pointcloud points to global point buffer.
	iPoint = 0;		// The cumulative point in the buffer.
	for (iPointcloud=0; iPointcloud<g_nPointclouds; iPointcloud++)
	{
		for (i=0; i<nPoints; i++)
		{
			if (nPoints>1)
			{
				// Copy the (x,y) values.
				g_pBufferPoints[iPoint*2] = g_pointcloud.points[i].x;
				g_pBufferPoints[iPoint*2+1] = g_pointcloud.points[i].y;
			
				// Copy the z-blanking value.
				g_pBufferBlanking[iPoint] = (g_pointcloud.points[i].z != 0.0) ? LASERON : LASEROFF;
			}
			else
			{
				// Copy the (x,y) values.
				g_pBufferPoints[iPoint*2] = 0.0;
				g_pBufferPoints[iPoint*2+1] = 0.0;
			
				// Copy the z-blanking value.
				g_pBufferBlanking[iPoint] = LASERON;
			}
			
			iPoint++;
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
	e=DAQmxWriteAnalogF64 (hTask, g_nPointsBuffer, FALSE, 10.0, DAQmx_Val_GroupByScanNumber, g_pBufferPoints, &nPointsWritten, NULL);
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
		e=DAQmxCfgOutputBuffer (hTask, 2*g_nPointsBufferRegistered);
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

	ros::Subscriber subGalvoPoints = node.subscribe("GalvoDriver/pointcloud", 1000, GalvoPointCloud_callback);
	//ros::Rate rateUpdate(10);

	char 		szTask[]="OutputPointList";
	char		szPhysicalChannel[]="Dev1/ao0:1";
	float64		points[]={0.0, 0.0, 0.0, 0.0};


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
	e=DAQmxCfgOutputBuffer (hTask, 2*g_nPointsBuffer);
	if (DAQmxFailed(e))
	{
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
		ROS_WARN(errBuff);
	}
	WritePoints (hTask);
	WritePoints (hTask);
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
	
	ros::spin();
	
	e=DAQmxStopTask (hTask);
	e=DAQmxClearTask (hTask);
	e=DAQmxResetDevice ("Dev1");


	return 0;
}
