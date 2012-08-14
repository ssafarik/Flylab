#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <boost/thread.hpp>
#include <NIDAQmx.h>

using namespace std;

#ifndef NULL
#define NULL 			0
#endif

#ifndef FALSE
#define FALSE 			0
#define TRUE 			(!FALSE)
#endif

#define MAX(a,b) 		((a)<(b) ? (b) : (a))

#define LASERON			TRUE
#define LASEROFF 		FALSE

#define DEVICE			"Dev1"
#define CHANNELS		"ao0:1"

#define NCOPIES_POINTCLOUDEX 2

#define LENERR			2048
char    				szErr[LENERR]={'\0'};
int32   				e=0;

TaskHandle 	 			g_hTask=NULL;
int32					g_nPointsPerCloud=0;
int32					g_nPointsBufferDaq=0;
int32					g_nPointsBufferDaqRegistered = -1;
int32					g_nPointsPointcloudEx=0;
int32					g_nPointsPointcloudExMax=0;
int32					g_nPointsPointcloudExRegistered=0;
int32					g_nDuplicates = 0;
float64 			   *g_pPointcloudExPoints=NULL;
int32 				   *g_pPointcloudExIntensity=NULL;

boost::mutex 			g_lockPointcloud; 	// To prevent the various callbacks from colliding.
boost::mutex 			g_lockDAQ; 			// To prevent the various callbacks from colliding.
sensor_msgs::PointCloud g_pointcloud;

int						g_bStarted=FALSE;

double					g_hzPoint = 10.0; // The point rate effects the lag in the pointcloud update = onboardbufsize/g_hzPoint, e.g. 4095/16384 = 0.25 seconds.
double					g_hzPointcloud = 0.0;
double					g_hzPointcloudEx = 0.0;
double					g_hzUSB = 40.0;

double					g_timeHeartbeat=0.0;

int 					g_bNeedToReset=FALSE;	// If true, then reset the DAQ.  Checked periodically.


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
void HandleDAQError(int32 e)
{
	if (DAQmxFailed(e))
	{
		g_bNeedToReset = TRUE;
		DAQmxGetExtendedErrorInfo(szErr,LENERR);
		ROS_WARN(szErr);
	}
}


// ************************************
// GalvoPointCloud_callback()
// Save the given pointcloud to g_pointcloud
//
void GalvoPointCloud_callback(const sensor_msgs::PointCloud::ConstPtr& pointcloud)
{
	double	hzPoint = 0.0;
	
	boost::lock_guard<boost::mutex> lock(g_lockPointcloud);

	// Copy the given pointcloud, if it contains points.
	if (pointcloud->points.size()>0)
		g_pointcloud = sensor_msgs::PointCloud(*pointcloud); 

	ros::param::get("galvodriver/hzPoint", hzPoint);
	if (g_hzPoint != hzPoint)
	{
		g_hzPoint = hzPoint;
		g_bNeedToReset = TRUE;
	}
	//cout << g_nPointsBufferDaq << " ";

}


//*********************************************
// OnEveryNSamples_callback()
// Called when the DAQ wants more data.
//
int32 CVICALLBACK OnEveryNSamples_callback (TaskHandle hTask, int32 eventType, uInt32 nSamples, void *callbackData)
{
	boost::lock_guard<boost::mutex> lock1(g_lockDAQ);
	boost::lock_guard<boost::mutex> lock2(g_lockPointcloud);
	
	g_timeHeartbeat = ros::Time::now().toSec();
	
	
	//cout << "CB";
	// If there was an error in a prior DAQmx call, then reset the device.
	if (g_bNeedToReset)
		ResetDAQ();
	else
	{
		if (hTask)
		{
			UpdatePointsFromPointcloud();
	
			if (!RegisterCallbackDAQBuffer(hTask))
				WritePoints(hTask);
	
			if (!g_bStarted)
			{
				e = DAQmxStartTask (hTask);
				HandleDAQError(e);
				if (!DAQmxFailed(e))
					g_bStarted = TRUE;
			}
		}
	}

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
	

	// If a multi-point cloud, then duplicate its points to bring it to down the USB rate, etc.
	if (g_pointcloud.points.size()>1 || g_nPointsPerCloud == 0)
	{
		g_nPointsPerCloud = MAX(1,g_pointcloud.points.size());
		g_hzPointcloud = g_hzPoint / (double)g_nPointsPerCloud;						// Output rate of the given pointcloud.
		g_nDuplicates = (int32)MAX(1.0, ceil(g_hzPointcloud/g_hzUSB));          	// Number of point (or pointcloud) copies needed to stay below the USB update rate.
		g_hzPointcloudEx = g_hzPointcloud / (double)g_nDuplicates;					// Output rate of the expanded pointcloud.
		g_nPointsPointcloudEx = g_nPointsPerCloud * g_nDuplicates;					// Number of points in the "expanded" pointcloud.
		g_nPointsBufferDaq = NCOPIES_POINTCLOUDEX * g_nPointsPointcloudEx; 			// Number of points in the PC buffer -- multiple pointcloudex's.
	}
	else // if a single-point cloud, make it the same size as the prior multi-point cloud so we don't have to reregister the DAQ buffer, etc.
	{
		g_nDuplicates = g_nPointsPointcloudEx;										// Make same size as prior cloud, but by way of using duplicates instead of unique points.
		g_nPointsPerCloud = 1;
		g_hzPointcloud = g_hzPoint / (double)g_nPointsPerCloud;						// Output rate of the given pointcloud.
		g_hzPointcloudEx = g_hzPointcloud / (double)g_nDuplicates;					// Output rate of the expanded pointcloud.
		g_nPointsPointcloudEx = g_nPointsPerCloud * g_nDuplicates;					// Number of points in the "expanded" pointcloud.
		g_nPointsBufferDaq = NCOPIES_POINTCLOUDEX * g_nPointsPointcloudEx; 			// Number of points in the PC buffer -- multiple pointcloudex's.
	}

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
		for (i=0; i<g_nPointsPerCloud; i++)
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
	else // Empty pointcloud -- use point (0,0)
	{
		for (i=0; i<g_nPointsPerCloud; i++)
		{
			for (iDuplicate=0; iDuplicate<g_nDuplicates; iDuplicate++)
			{
				// Copy the (0,0) values.
				g_pPointcloudExPoints[iPoint*2] = 0.0;
				g_pPointcloudExPoints[iPoint*2+1] = 0.0;
			
				// Copy the z-blanking value.
				g_pPointcloudExIntensity[iPoint] = LASERON;
		
				iPoint++;
			}
		}
	}
} // UpdatePointsFromPointcloud()


// ************************************
// WritePoints()
// Write pre-computed points to the DAQ.  Uses the points already computed in g_pPointcloudExPoints and g_pPointcloudExIntensity.
void WritePoints(TaskHandle hTask)
{
	int32		nPointsWritten = 0;
	
	// Write to the (offboard) buffer.
	e = DAQmxWriteAnalogF64 (hTask, g_nPointsPointcloudEx, FALSE, 0.0, DAQmx_Val_GroupByScanNumber, g_pPointcloudExPoints, &nPointsWritten, NULL);
	HandleDAQError(e);
} // WritePoints()



// ************************************
// RegisterCallbackDAQBuffer()
// Register (if necessary) a callback function to get called upon the DAQ needing more data.
//
int RegisterCallbackDAQBuffer (TaskHandle hTask)
{
	int rv=FALSE;
	
	if (g_nPointsBufferDaq != g_nPointsBufferDaqRegistered)
	//if (g_nPointsBufferDaqRegistered < g_nPointsBufferDaq)
	{
		if (g_bStarted)
		{
			e = DAQmxStopTask (hTask);
			HandleDAQError(e);
			if (!DAQmxFailed(e))
				g_bStarted = FALSE;
		}

		// Unregister.
		e = DAQmxRegisterEveryNSamplesEvent (hTask, DAQmx_Val_Transferred_From_Buffer, g_nPointsPointcloudExRegistered, 0, NULL,                     NULL);
		HandleDAQError(e);

		// Resize the output buffer.
		e = DAQmxCfgOutputBuffer (hTask, g_nPointsBufferDaq);
		HandleDAQError(e);

		// Fill the buffer.
		for (int i=0; i<NCOPIES_POINTCLOUDEX; i++)
			WritePoints(hTask);

		// Reregister.
		e = DAQmxRegisterEveryNSamplesEvent (hTask, DAQmx_Val_Transferred_From_Buffer, g_nPointsPointcloudEx, 0, OnEveryNSamples_callback, NULL);
		HandleDAQError(e);
		
		ROS_WARN("hzPre=%9.2f, hzPost=%6.2f, offbdBufsz=%6lu, nPtsPerCloud=%6lu, nDups=%6lu", 
				g_hzPointcloud, 
				g_hzPointcloudEx, 
				g_nPointsPointcloudEx,
				MAX(1,g_pointcloud.points.size()),
				g_nDuplicates);

		if (!g_bNeedToReset)
		{
			g_nPointsBufferDaqRegistered = g_nPointsBufferDaq;
			g_nPointsPointcloudExRegistered = g_nPointsPointcloudEx;
		}

		rv = TRUE;
	}
	
	return rv;
} // RegisterCallbackDAQBuffer()


//*********************************************
void ResetDAQ(void)
{
	char 				szTask[]="OutputPointList";
	char				szPhysicalChannel[]=DEVICE "/" CHANNELS;

	
	g_bNeedToReset = FALSE;
	g_nPointsBufferDaqRegistered = -1;
	
	if (g_hTask)
	{
		e = DAQmxStopTask (g_hTask);
		HandleDAQError(e);
		
		e = DAQmxClearTask (g_hTask);
		HandleDAQError(e);
		
		g_hTask = NULL;
	}
	
	e = DAQmxResetDevice(DEVICE);
	HandleDAQError(e);
	if (!DAQmxFailed(e))
	{
		e = DAQmxCreateTask (szTask, &g_hTask);
		HandleDAQError(e);
		
		// Set up the output channels.
		e = DAQmxCreateAOVoltageChan (g_hTask, szPhysicalChannel, NULL, -10.0, +10.0, DAQmx_Val_Volts, NULL);
		HandleDAQError(e);
		
		e = DAQmxCfgSampClkTiming (g_hTask, "OnboardClock", (float64)g_hzPoint, DAQmx_Val_Rising, DAQmx_Val_ContSamps, g_nPointsPointcloudEx);
		HandleDAQError(e);
		
		e = DAQmxSetWriteAttribute (g_hTask, DAQmx_Write_RegenMode, DAQmx_Val_DoNotAllowRegen);
		HandleDAQError(e);
		
		
		// Set/Get the onboard DAQ buffer size.
		//e = DAQmxSetBufOutputOnbrdBufSize(g_hTask, 4095);  // The only allowed value for NI USB-6211 is 4095.
		//HandleDAQError(e);
		
		
		// Initialize the offboard buffer.
		UpdatePointsFromPointcloud();
		e = DAQmxCfgOutputBuffer (g_hTask, g_nPointsBufferDaq);
		HandleDAQError(e);
		
		RegisterCallbackDAQBuffer(g_hTask);
		
		
		// Start the DAQ output.
		if (!g_bStarted)
		{
			e = DAQmxStartTask (g_hTask);
			HandleDAQError(e);
			if (!DAQmxFailed(e))
				g_bStarted = TRUE;
		}
	}
	
	
	if (!g_bNeedToReset)
		ROS_WARN("Reset DAQ Succeeded.");
	else
		ROS_WARN("Reset DAQ Failed.");

	
	g_timeHeartbeat = ros::Time::now().toSec();

} // ResetDAQ()


//*********************************************
int main(int argc, char **argv)
{
	ros::init(argc, argv, "GalvoDriver");
	
	ros::NodeHandle 	node;
	uInt32 				data;

	
	node.getParam("galvodriver/hzPoint", g_hzPoint);
	node.getParam("galvodriver/hzUSB", g_hzUSB);
	
	ros::Rate           rosRate(2.0 * g_hzUSB);
	
	ros::Subscriber subGalvoPoints = node.subscribe("GalvoDriver/pointcloud", 2, GalvoPointCloud_callback);
	ROS_WARN ("Listening on GalvoDriver/pointcloud, hzPoint=%0.2f", g_hzPoint);

	ResetDAQ();

	e = DAQmxGetBufOutputOnbrdBufSize(g_hTask, &data);
	HandleDAQError(e);
	ROS_WARN("Onboard buffer size=%u", data);
	
	//ros::spin();
	double timeAllowed = ros::Duration(1.0).toSec();
	double timeTaken;
	while (ros::ok())
	{
		ros::spinOnce();
		rosRate.sleep();
		timeTaken = ros::Time::now().toSec()-g_timeHeartbeat;
		if (timeTaken > timeAllowed)
		{
			ROS_WARN("Resetting DAQ due to heartbeat irregularity.");
			ResetDAQ();
		}
	}
	
	ResetDAQ();
	g_nPointsPointcloudEx = 2; // 2 for two points.
	g_pPointcloudExPoints = new float64[g_nPointsPointcloudEx * 2]; // 2 for (x,y)
	g_pPointcloudExPoints[0] =   0.0;
	g_pPointcloudExPoints[1] = -10.0;
	g_pPointcloudExPoints[2] =   0.0;
	g_pPointcloudExPoints[3] = -10.0;
	WritePoints(g_hTask);
	
	//e = DAQmxStopTask (g_hTask);
	//e = DAQmxClearTask (g_hTask);
	//e = DAQmxResetDevice (DEVICE);


	return 0;
}
