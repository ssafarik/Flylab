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

#define DEVICE			"Dev1"
#define CHANNELS		"ao0:1"

#define NCOPIES_POINTCLOUDEX 2

#define LENERR	2048
char    				szErr[LENERR]={'\0'};
int32   				e=0;

TaskHandle 	 			g_hTask=0;
int32 					g_nPointsPointcloudEx=0;
int32 					g_nPointsBufferDaqRegistered = -1;
int32 					g_nPointsPointcloudExMax=0;
int32					g_nPointsBufferDaq=0;
int32					g_nDuplicates = 0;
float64 			   *g_pPointcloudExPoints=NULL;
int32 				   *g_pPointcloudExIntensity=NULL;

boost::mutex 			g_lockPointcloud; 	// To prevent the various callbacks from colliding.
boost::mutex 			g_lockDAQ; 			// To prevent the various callbacks from colliding.
//int						g_lockPointcloud = FALSE;
sensor_msgs::PointCloud g_pointcloud;

int						g_bStarted=FALSE;

double					g_hzPoint = 10.0; // The point rate effects the delay of laser movement = onboardbufsize/g_hzPoint, e.g. 4095/16384 = 0.25 seconds.
double					g_hzPointcloud = 0.0;
double					g_hzPointcloudEx = 0.0;
double					g_hzUSB = 70.0;

int 					g_bError=FALSE;	// Flag any errors in DAQmx function calls.


// ************************************
// Function Prototypes.
void 				GalvoPointCloud_callback (const sensor_msgs::PointCloud::ConstPtr& pointcloud);
void 				UpdatePointsFromPointcloud (void);
void 				HandleDAQError (int32 e);
void 				WritePoints (TaskHandle hTask);
int 				RegisterCallbackDAQBuffer (TaskHandle hTask);
int32 CVICALLBACK 	OnEveryNSamples_callback (TaskHandle hTask, int32 eventType, uInt32 nSamples, void *callbackData);
void 				ResetDAQ (void);



// ************************************
// GalvoPointCloud_callback()
// Save the given pointcloud to g_pointcloud
//
void GalvoPointCloud_callback(const sensor_msgs::PointCloud::ConstPtr& pointcloud)
{
	//ROS_WARN("GalvoPointCloud_callback()");
	boost::lock_guard<boost::mutex> lock(g_lockPointcloud);

	// Copy the given pointcloud, if it contains points.
	if (pointcloud->points.size()>0)
		g_pointcloud = sensor_msgs::PointCloud(*pointcloud); 

	ros::param::get("galvodriver/hzPoint", g_hzPoint);

	//ROS_WARN("GalvoPointCloud_callback() Done");
}


//*********************************************
// OnEveryNSamples_callback()
// Called when the DAQ wants more data.
//
int32 CVICALLBACK OnEveryNSamples_callback (TaskHandle hTask, int32 eventType, uInt32 nSamples, void *callbackData)
{
	//ROS_WARN("OnEveryNSamples_callback()");
	boost::lock_guard<boost::mutex> lock1(g_lockDAQ);
	boost::lock_guard<boost::mutex> lock2(g_lockPointcloud);
	
	if (hTask)
	{
		UpdatePointsFromPointcloud();

		if (!RegisterCallbackDAQBuffer(hTask))
			WritePoints(hTask);

		if (!g_bStarted)
		{
			e=DAQmxStartTask (hTask);
			HandleDAQError(e);
			if (!DAQmxFailed(e))
				g_bStarted = TRUE;
		}
	}
		
	// If there was an error in a prior DAQmx call, then reset the device.
	if (g_bError)
		ResetDAQ();

	//ROS_WARN("OnEveryNSamples_callback() Done");

	return 0;	
}


// ************************************
// UpdatePointsFromPointcloud()
// Take the points in the pointcloud structure, and put them into the global points array.
//
void UpdatePointsFromPointcloud(void)
{
	int			i=0;
	int			iPoint=0;
	int			iPointcloud=0;
	int			iDuplicate=0;
	int32		nPointsPerCloud=0;

	//ROS_WARN("UpdatePointsFromPointcloud()");

	nPointsPerCloud = MAX(1,g_pointcloud.points.size());
	g_hzPointcloud = g_hzPoint / (double)nPointsPerCloud;					// Output rate of the given pointcloud.
	g_nDuplicates = (int32)MAX(1.0, ceil(g_hzPointcloud/g_hzUSB) + 1.0);	// Number of point (or pointcloud) copies needed to stay within the USB update rate.
	g_hzPointcloudEx = g_hzPointcloud / (double)g_nDuplicates;				// Output rate of the expanded pointcloud.
	g_nPointsPointcloudEx = nPointsPerCloud * g_nDuplicates;				// Number of points in the "expanded" pointcloud.
	g_nPointsBufferDaq = NCOPIES_POINTCLOUDEX * g_nPointsPointcloudEx; 		// Number of points in the PC buffer.


	// Realloc memory if size grew.  This is one buffer containing multiple copies of pointcloud, i.e. one pointcloudEx.  The DAQ will use this buffer NCOPIES_POINTCLOUDEX times.
	if (g_nPointsPointcloudExMax < g_nPointsPointcloudEx)
	{
		g_nPointsPointcloudExMax = g_nPointsPointcloudEx;

		if (g_pPointcloudExPoints != NULL)
			delete g_pPointcloudExPoints;
		g_pPointcloudExPoints = new float64[g_nPointsPointcloudEx * 2]; // Two values per point, i.e. (x,y).

		if (g_pPointcloudExIntensity != NULL)
			delete g_pPointcloudExIntensity;
		g_pPointcloudExIntensity = new int32[g_nPointsPointcloudEx];
	}

	
	// Copy the pointcloud points to global point buffer.
	iPoint = 0;		// The cumulative point in the buffer.
	if (g_pointcloud.points.size()>0)
		for (i=0; i<nPointsPerCloud; i++)
		{
			for (iDuplicate=0; iDuplicate<g_nDuplicates; iDuplicate++)
			{
				// Copy the (x,y) values.
				g_pPointcloudExPoints[iPoint*2]   = (float64)g_pointcloud.points[i].x;
				g_pPointcloudExPoints[iPoint*2+1] = (float64)g_pointcloud.points[i].y;
			
				// Copy the z-blanking value.
				g_pPointcloudExIntensity[iPoint] = (g_pointcloud.points[i].z != 0.0) ? LASERON : LASEROFF;
		
				iPoint++;
			}
		}
	else
	{
		for (i=0; i<nPointsPerCloud; i++)
		{
			for (iDuplicate=0; iDuplicate<g_nDuplicates; iDuplicate++)
			{
				// Copy the (x,y) values.
				g_pPointcloudExPoints[iPoint*2] = 0.0;
				g_pPointcloudExPoints[iPoint*2+1] = 0.0;
			
				// Copy the z-blanking value.
				g_pPointcloudExIntensity[iPoint] = LASERON;
		
				iPoint++;
			}
		}
	}
	//ROS_WARN("UpdatePointsFromPointcloud() Done");
}


// ************************************
void HandleDAQError(int32 e)
{
	if (DAQmxFailed(e))
	{
		g_bError = TRUE;
		DAQmxGetExtendedErrorInfo(szErr,LENERR);
		ROS_WARN(szErr);
	}
}


// ************************************
// WritePoints()
// Write more points to the DAQ.  Uses the points already computed in g_pPointcloudExPoints and g_pPointcloudExIntensity.
void WritePoints(TaskHandle hTask)
{
	int32		nPointsWritten = 0;
	
	//ROS_WARN("WritePoints()");
	// Write to the (offboard) buffer.
	e=DAQmxWriteAnalogF64 (hTask, g_nPointsPointcloudEx, FALSE, 0.0, DAQmx_Val_GroupByScanNumber, g_pPointcloudExPoints, &nPointsWritten, NULL);
	HandleDAQError(e);
	//ROS_WARN("WritePoints() Done");
}



// ************************************
// RegisterCallbackDAQBuffer()
// Register (if necessary) a callback function to get called upon the DAQ needing more data.
//
int RegisterCallbackDAQBuffer (TaskHandle hTask)
{
	int rv=FALSE;
	
	//ROS_WARN("RegisterCallbackDAQBuffer()");

	if (g_nPointsBufferDaq != g_nPointsBufferDaqRegistered)
	{
		g_nPointsBufferDaqRegistered = g_nPointsBufferDaq;

		if (g_bStarted)
		{
			e=DAQmxStopTask (hTask);
			HandleDAQError(e);
			if (!DAQmxFailed(e))
				g_bStarted = FALSE;
		}

		// Unregister.
		e=DAQmxRegisterEveryNSamplesEvent (hTask, DAQmx_Val_Transferred_From_Buffer, g_nPointsPointcloudEx, 0, NULL,                     NULL);
		HandleDAQError(e);

		// Resize the output buffer.
		e=DAQmxCfgOutputBuffer (hTask, g_nPointsBufferDaqRegistered);
		HandleDAQError(e);

		// Fill the buffer.
		for (int i=0; i<NCOPIES_POINTCLOUDEX; i++)
			WritePoints(hTask);

		// Reregister.
		e=DAQmxRegisterEveryNSamplesEvent (hTask, DAQmx_Val_Transferred_From_Buffer, g_nPointsPointcloudEx, 0, OnEveryNSamples_callback, NULL);
		HandleDAQError(e);
		
		ROS_WARN("New buffer size:  nPointsPerCloud=%lu (hz=%0.2f), nPointsPerCloudEx=%lu (hz=%0.2f), nDuplicates=%lu", 
			MAX(1,g_pointcloud.points.size()),
			g_hzPointcloud, 
			g_nPointsPointcloudEx,
			g_hzPointcloudEx, 
			g_nDuplicates);
	
		rv = TRUE;
	}
	
	//ROS_WARN("RegisterCallbackDAQBuffer() Done");
	return rv;
}


//*********************************************
void ResetDAQ(void)
{
	char 				szTask[]="OutputPointList";
	char				szPhysicalChannel[]=DEVICE "/" CHANNELS;

	//ROS_WARN("ResetDAQ()");
	
	g_bError = FALSE;
	g_nPointsBufferDaqRegistered = -1;
	
	if (g_hTask)
	{
		e=DAQmxStopTask (g_hTask);
		HandleDAQError(e);
		
		e=DAQmxClearTask (g_hTask);
		HandleDAQError(e);
	}
	
	e=DAQmxResetDevice(DEVICE);
	HandleDAQError(e);
	if (!DAQmxFailed(e))
	{
		e=DAQmxCreateTask (szTask, &g_hTask);
		HandleDAQError(e);
		
		// Set up the output channels.
		e=DAQmxCreateAOVoltageChan (g_hTask, szPhysicalChannel, NULL, -10.0, +10.0, DAQmx_Val_Volts, NULL);
		HandleDAQError(e);
		
		e=DAQmxCfgSampClkTiming (g_hTask, "OnboardClock", (float64)g_hzPoint, DAQmx_Val_Rising, DAQmx_Val_ContSamps, g_nPointsPointcloudEx);
		HandleDAQError(e);
		
		e=DAQmxSetWriteAttribute (g_hTask, DAQmx_Write_RegenMode, DAQmx_Val_DoNotAllowRegen);
		HandleDAQError(e);
		
		
		// Set/Get the onboard DAQ buffer size.if (
		//e=DAQmxSetBufOutputOnbrdBufSize(g_hTask, 4095);  // Only allowed value for NI USB-6211 is 4095.
		//HandleDAQError(e);
		
		
		// Initialize the offboard buffer.
		UpdatePointsFromPointcloud();
		e=DAQmxCfgOutputBuffer (g_hTask, g_nPointsBufferDaq);
		HandleDAQError(e);
		
		RegisterCallbackDAQBuffer(g_hTask);
		
		
		// Start the DAQ output.
		if (!g_bStarted)
		{
			e=DAQmxStartTask (g_hTask);
			HandleDAQError(e);
			if (!DAQmxFailed(e))
				g_bStarted = TRUE;
		}
		ROS_WARN("Reset DAQ.");
	}
	else
		ROS_WARN("Failed to reset DAQ.");

	//ROS_WARN("ResetDAQ() Done");
}


//*********************************************
int main(int argc, char **argv)
{
	ros::init(argc, argv, "GalvoDriver");
	
	ros::NodeHandle 	node;
	uInt32 				data;

	
	node.getParam("galvodriver/hzPoint", g_hzPoint);
	node.getParam("galvodriver/hzUSB", g_hzUSB);
	
	ros::Subscriber subGalvoPoints = node.subscribe("GalvoDriver/pointcloud", 2, GalvoPointCloud_callback);
	ROS_WARN ("Listening on GalvoDriver/pointcloud, hzPoint=%0.2f", g_hzPoint);

	ResetDAQ();

	e=DAQmxGetBufOutputOnbrdBufSize(g_hTask, &data);
	HandleDAQError(e);
	ROS_WARN("Onboard bufsize=%u", data);
	
	ros::spin();
	
	e=DAQmxStopTask (g_hTask);
	e=DAQmxClearTask (g_hTask);
	e=DAQmxResetDevice (DEVICE);


	return 0;
}
