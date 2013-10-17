#include "ros/ros.h"
#include "ros/node_handle.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Time.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <boost/thread.hpp>
#include <NIDAQmx.h>

#include "std_msgs/Int32.h"

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

#define LENERR			4096
char                   *g_pszError=NULL;
int32   				e=0;

TaskHandle 	 			g_hTask=NULL;
int32					g_nPointsPerCloud=0;
int32					g_nPointsBufferDaq=0;
int32					g_nPointsBufferDaqRegistered = -1;
int32					g_nPointsPointcloudEx=0;
int32					g_nPointsPointcloudExMax=0;
int32					g_nPointsPointcloudExRegistered=0;
int32					g_nDupsPointcloudDup = 0;
int32					g_nDupsPoint = 0;
float64 			   *g_pPointcloudExPoints=NULL;
int32 				   *g_pPointcloudExIntensity=NULL;

boost::mutex 			g_lockPointcloud; 	// To prevent the various callbacks from colliding.
boost::mutex 			g_lockDAQ; 			// To prevent the various callbacks from colliding.
sensor_msgs::PointCloud g_pointcloud;

int						g_bStarted=FALSE;

double					g_hzDaqClock = 1000.0; // The point rate effects the lag in the pointcloud update = onboardbufsize/g_hzDaqClock, e.g. 4095/16384 = 0.25 seconds.
double					g_hzGalvoRate = 10.0; // The rate the galvos can move from point to point, with reasonable quality.

double					g_hzPointcloud = 0.0;
double					g_hzPointcloudEx = 0.0;
double					g_hzUSB = 40.0;
double					g_xBeamsink = 0.0;
double					g_yBeamsink = 0.0;

double					g_timeHeartbeat=0.0;	// This time should get updated at regular intervals.  Checked in main().

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
		if (g_pszError)
		{
			DAQmxGetExtendedErrorInfo(g_pszError,LENERR);
			ROS_WARN("DAQ Error.  g_hTask=0x%0X: %s", g_hTask, g_pszError);
		}
		else
			ROS_WARN("Error while initializing.  g_pszError not initialized.");
	}
} // HandleDAQError()


// ************************************
// GalvoPointCloud_callback()
// Save the given pointcloud to g_pointcloud
//
void GalvoPointCloud_callback(const sensor_msgs::PointCloud::ConstPtr& pointcloud)
{
	double	hzDaqClock = 0.0;
	

	// Copy the given pointcloud, if it contains points.
	//g_lockPointcloud.lock();
	if (pointcloud->points.size()>0)
		g_pointcloud = sensor_msgs::PointCloud(*pointcloud); 

	//g_lockPointcloud.unlock();

//	ros::param::get("galvodriver/hzDaqClock", hzDaqClock);
//	if (g_hzDaqClock != hzDaqClock)
//	{
//		g_hzDaqClock = hzDaqClock;
//		g_bNeedToReset = TRUE;
//	}
	//cout << g_nPointsBufferDaq << " ";

} // GalvoPointCloud_callback()


//*********************************************
// OnEveryNSamples_callback()
// Called when the DAQ wants more data.
//
int32 CVICALLBACK OnEveryNSamples_callback (TaskHandle hTask, int32 eventType, uInt32 nSamples, void *callbackData)
{
	g_timeHeartbeat = ros::Time::now().toSec();
	
	
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
				//ROS_WARN("StartTask");
				HandleDAQError(e);
				if (!DAQmxFailed(e))
					g_bStarted = TRUE;
			}
		}
	}

	return 0;	
} // OnEveryNSamples_callback()


// ************************************
// UpdatePointsFromPointcloud()
// Take the points in the pointcloud structure, and put them into the global points array.
//
void UpdatePointsFromPointcloud(void)
{
	int			i=0;
	int			iPoint=0;
	int			iDupPointcloud=0;
	int			iDupPoint=0;
	double		hzPointcloudDup = 0.0;
	

	//g_lockPointcloud.lock();
	
	// If a multi-point cloud, then duplicate its points to bring it to down the USB rate, etc.
	if (g_pointcloud.points.size()>1 || g_nPointsPerCloud == 0)
	{
		g_nPointsPerCloud = MAX(1,g_pointcloud.points.size());								// The number of points in the given cloud.
		g_hzPointcloud = g_hzDaqClock / (double)g_nPointsPerCloud;							// Output rate of the given pointcloud, if we just blasted it out.

		g_nDupsPoint = (int32)MAX(1.0, ceil(g_hzDaqClock / g_hzGalvoRate));					// Number of point copies needed to stay below the max galvo rate.
		hzPointcloudDup =  g_hzPointcloud / (double)g_nDupsPoint;							// Rate of a pointcloud with duplicated points.
		g_nDupsPointcloudDup = (int32)MAX(1.0, ceil(hzPointcloudDup / g_hzUSB));			// Number of pointcloudDup copies needed to stay below the USB update rate.

		g_hzPointcloudEx = g_hzPointcloud / (double)(g_nDupsPointcloudDup * g_nDupsPoint);	// Output rate of the expanded pointcloud.
		g_nPointsPointcloudEx = g_nPointsPerCloud * (g_nDupsPointcloudDup * g_nDupsPoint);	// Number of points in the "expanded" pointcloud.
		g_nPointsBufferDaq = NCOPIES_POINTCLOUDEX * g_nPointsPointcloudEx; 					// Number of points in the PC buffer -- multiple pointcloudex's.
	}
	else // if a single-point cloud, make it the same size as the prior multi-point cloud so we don't have to reregister the DAQ buffer, etc.
	{
		g_nPointsPerCloud = 1;
		g_hzPointcloud = g_hzDaqClock / (double)g_nPointsPerCloud;							// Output rate of the given pointcloud.

		g_nDupsPoint = g_nPointsPointcloudEx;												// Make same size as prior cloud, but by way of using duplicates instead of unique points.
		g_nDupsPointcloudDup = 1;															// Make same size as prior cloud, but by way of using duplicates instead of unique points.

		g_hzPointcloudEx = g_hzPointcloud / (double)(g_nDupsPointcloudDup * g_nDupsPoint);	// Output rate of the expanded pointcloud.
		g_nPointsPointcloudEx = g_nPointsPerCloud * (g_nDupsPointcloudDup * g_nDupsPoint);	// Number of points in the "expanded" pointcloud.
		g_nPointsBufferDaq = NCOPIES_POINTCLOUDEX * g_nPointsPointcloudEx; 				// Number of points in the PC buffer -- multiple pointcloudex's.
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
	{
		//ROS_WARN("Nonempty pointcloud.");
		for (iDupPointcloud=0; iDupPointcloud<g_nDupsPointcloudDup; iDupPointcloud++)
		{
			for (i=0; i<g_nPointsPerCloud; i++)
			{
				for (iDupPoint=0; iDupPoint<g_nDupsPoint; iDupPoint++)
				{
					// Copy the (x,y) values.
					g_pPointcloudExPoints[iPoint*2]   = (float64)g_pointcloud.points[i].x;
					g_pPointcloudExPoints[iPoint*2+1] = (float64)g_pointcloud.points[i].y;

					// Copy the z-blanking value.
					g_pPointcloudExIntensity[iPoint] = (g_pointcloud.points[i].z != 0.0) ? LASERON : LASEROFF;
			
					iPoint++;
				}
			}
		}
	}
	else // Empty pointcloud -- use beamsink point.
	{
		//ROS_WARN("Empty pointcloud.");
		for (iDupPointcloud=0; iDupPointcloud<g_nDupsPointcloudDup; iDupPointcloud++)
		{
			for (i=0; i<g_nPointsPerCloud; i++)
			{
				for (iDupPoint=0; iDupPoint<g_nDupsPoint; iDupPoint++)
				{
					// Copy the beamsink values.
					g_pPointcloudExPoints[iPoint*2]   = g_xBeamsink;
					g_pPointcloudExPoints[iPoint*2+1] = g_yBeamsink;

					// Copy the z-blanking value.
					g_pPointcloudExIntensity[iPoint] = LASERON;
			
					iPoint++;
				}
			}
		}
	}
	//g_lockPointcloud.unlock();

} // UpdatePointsFromPointcloud()


// ************************************
// WritePoints()
// Write pre-computed points to the DAQ.  Uses the points already computed in g_pPointcloudExPoints and g_pPointcloudExIntensity.
void WritePoints(TaskHandle hTask)
{
	int32		nPointsWritten = 0;
	
	// Write to the (offboard) buffer.
	e = DAQmxWriteAnalogF64 (hTask, g_nPointsPointcloudEx, FALSE, 0.0, DAQmx_Val_GroupByScanNumber, g_pPointcloudExPoints, &nPointsWritten, NULL);
	//ROS_WARN("WriteAnalogF64 wrote %d points.", nPointsWritten);
	HandleDAQError(e);
} // WritePoints()



// ************************************
// RegisterCallbackDAQBuffer()
// Register (if necessary) a callback function to get called upon the DAQ needing more data.
//
int RegisterCallbackDAQBuffer (TaskHandle hTask)
{
	int		rv=FALSE;
	bool32	bTaskDone;
	
	if (g_nPointsBufferDaq != g_nPointsBufferDaqRegistered)
	//if (g_nPointsBufferDaqRegistered < g_nPointsBufferDaq)
	{
		if (g_bStarted)
		{
			e = DAQmxIsTaskDone (g_hTask, &bTaskDone);
			//ROS_WARN("IsTaskDone A");
			while (!bTaskDone)
			{
				e = DAQmxIsTaskDone (g_hTask, &bTaskDone);
				HandleDAQError(e);
				//ROS_WARN("IsTaskDone A");
			}

			e = DAQmxStopTask (hTask);
			//ROS_WARN("StopTask A");
			HandleDAQError(e);

			if (!g_bNeedToReset)
				g_bStarted = FALSE;
		}

		// Unregister.
		e = DAQmxRegisterEveryNSamplesEvent (hTask, DAQmx_Val_Transferred_From_Buffer, g_nPointsPointcloudExRegistered, 0, NULL,                     NULL);
		//ROS_WARN("RegisterEveryNSamplesEvent1");
		HandleDAQError(e);
		g_nPointsPointcloudExRegistered = 0;

		// Configure the output buffer.
		e = DAQmxCfgOutputBuffer (hTask, g_nPointsBufferDaq);
		//ROS_WARN("CfgOutputBuffer");
		HandleDAQError(e);

		// Fill the buffer.
		for (int i=0; i<NCOPIES_POINTCLOUDEX; i++)
			WritePoints(hTask);

		// Reregister.
		e = DAQmxRegisterEveryNSamplesEvent (hTask, DAQmx_Val_Transferred_From_Buffer, g_nPointsPointcloudEx, 0, OnEveryNSamples_callback, NULL);
		//ROS_WARN("RegisterEveryNSamplesEvent2");
		HandleDAQError(e);
		
		if (!g_bNeedToReset)
		{
			g_nPointsBufferDaqRegistered = g_nPointsBufferDaq;
			g_nPointsPointcloudExRegistered = g_nPointsPointcloudEx;
		}

		//g_lockPointcloud.lock();
		ROS_WARN("hzPre=%9.2f, hzPost=%6.2f, offbdBufsz=%6lu, nPtsPerCloud=%6lu, nDupsPointcloud=%6lu, nDupsPoint=%6lu",
				g_hzPointcloud, 
				g_hzPointcloudEx, 
				g_nPointsPointcloudEx,
				MAX(1,g_pointcloud.points.size()),
				g_nDupsPointcloudDup,
				g_nDupsPoint);
		//g_lockPointcloud.lock();

		rv = TRUE;
	}
	
	return rv;
} // RegisterCallbackDAQBuffer()


//*********************************************
void ResetDAQ(void)
{
	char 				szTask[]="OutputPointList";
	char				szPhysicalChannel[]=DEVICE "/" CHANNELS;
	bool32				bTaskDone;

	ROS_WARN("****************Resetting...");
	g_lockDAQ.lock();
	g_bNeedToReset = FALSE;
	g_nPointsBufferDaqRegistered = -1;
	
	if (g_hTask)
	{
		if (g_bStarted)
		{
			e = DAQmxIsTaskDone (g_hTask, &bTaskDone);
			//ROS_WARN("IsTaskDone B");
			while (!bTaskDone)
			{
				e = DAQmxIsTaskDone (g_hTask, &bTaskDone);
				HandleDAQError(e);
				//ROS_WARN("IsTaskDone B");
			}

			e = DAQmxStopTask (g_hTask);
			//ROS_WARN("StopTask B");
			HandleDAQError(e);

			if (!g_bNeedToReset)
				g_bStarted = FALSE;
		}
		
		// Unregister.
		e = DAQmxRegisterEveryNSamplesEvent (g_hTask, DAQmx_Val_Transferred_From_Buffer, g_nPointsPointcloudExRegistered, 0, NULL, NULL);
		//ROS_WARN("RegisterEveryNSamplesEvent3");
		HandleDAQError(e);
		
		e = DAQmxClearTask (g_hTask);
		//ROS_WARN("ClearTask");
		HandleDAQError(e);
		
		g_hTask = NULL;
	}
	
	e = DAQmxResetDevice(DEVICE);
	//ROS_WARN("ResetDevice");
	HandleDAQError(e);
	if (!DAQmxFailed(e))
	{
		e = DAQmxCreateTask (szTask, &g_hTask);
		//ROS_WARN("CreateTask");
		HandleDAQError(e);
		
		if (g_hTask)
		{
			// Set up the output channels.
			e = DAQmxCreateAOVoltageChan (g_hTask, szPhysicalChannel, NULL, -10.0, +10.0, DAQmx_Val_Volts, NULL);
			//ROS_WARN("CreateAOVoltageChan");
			HandleDAQError(e);
			
			e = DAQmxCfgSampClkTiming (g_hTask, "OnboardClock", (float64)g_hzDaqClock, DAQmx_Val_Rising, DAQmx_Val_ContSamps, g_nPointsPointcloudEx);
			//ROS_WARN("CfgSampClkTiming");
			HandleDAQError(e);
			
			e = DAQmxSetWriteAttribute (g_hTask, DAQmx_Write_RegenMode, DAQmx_Val_DoNotAllowRegen);
			//ROS_WARN("SetWriteAttribute");
			HandleDAQError(e);
			
			
			// Set/Get the onboard DAQ buffer size.
			//e = DAQmxSetBufOutputOnbrdBufSize(g_hTask, 4095);  // The only allowed value for NI USB-6211 is 4095.
			//HandleDAQError(e);
			
			
			// Initialize the offboard buffer.
			UpdatePointsFromPointcloud();
			e = DAQmxCfgOutputBuffer (g_hTask, g_nPointsBufferDaq);
			//ROS_WARN("CfgOutputBuffer");
			HandleDAQError(e);

			// Fill the buffer.
			for (int i=0; i<NCOPIES_POINTCLOUDEX; i++)
				WritePoints(g_hTask);
			
			// Reregister.
			//RegisterCallbackDAQBuffer(g_hTask);
			e = DAQmxRegisterEveryNSamplesEvent (g_hTask, DAQmx_Val_Transferred_From_Buffer, g_nPointsPointcloudEx, 0, OnEveryNSamples_callback, NULL);
			//ROS_WARN("RegisterEveryNSamplesEvent4");
			HandleDAQError(e);
			if (!g_bNeedToReset)
			{
				g_nPointsBufferDaqRegistered = g_nPointsBufferDaq;
				g_nPointsPointcloudExRegistered = g_nPointsPointcloudEx;
			}
			
			
			// Start the DAQ output.
			if (!g_bStarted)
			{
				e = DAQmxStartTask (g_hTask);
				//ROS_WARN("StartTask");
				HandleDAQError(e);
				if (!DAQmxFailed(e))
					g_bStarted = TRUE;
			}
		}
	}
	
	
	if (!g_bNeedToReset)
		ROS_WARN("****************Reset DAQ Succeeded.");
	else
		ROS_WARN("****************Reset DAQ Failed.");

	
	g_timeHeartbeat = ros::Time::now().toSec();
	g_lockDAQ.unlock();

} // ResetDAQ()


//*********************************************
int main(int argc, char **argv)
{
	ros::init(argc, argv, "GalvoDriver");
	
	ros::NodeHandle 	node;
	uInt32 				udata;
	int32 				data;
	float64				fdata;
	char 				szTask[]="OutputPointList";
	char				szPhysicalChannel[]=DEVICE "/" CHANNELS;
	double				hzSpin;
	
	g_pszError = new char[LENERR];
	
	node.getParam("galvodriver/hzDaqClock", g_hzDaqClock);
	node.getParam("galvodriver/hzGalvoRate", g_hzGalvoRate);
	node.getParam("galvodriver/hzUSB", g_hzUSB);
	node.getParam("galvodirector/xBeamsink", g_xBeamsink);
	node.getParam("galvodirector/yBeamsink", g_yBeamsink);

	
	hzSpin = 2.0 * g_hzUSB;
	ros::Rate           rosRate(hzSpin);
	
	ros::Subscriber subGalvoPoints = node.subscribe("GalvoDriver/pointcloud", 2, GalvoPointCloud_callback);
	ROS_WARN ("Listening on GalvoDriver/pointcloud, hzDaqClock=%0.2f", g_hzDaqClock);
	ros::Publisher	pubHeartbeat = node.advertise<std_msgs::Time>("galvodriver/heartbeat", 10);

	ResetDAQ();

	e = DAQmxGetDevAOMaxRate(DEVICE, &fdata);
	HandleDAQError(e);
	ROS_WARN("Max output rate=%f", fdata);
	e = DAQmxGetBufOutputOnbrdBufSize(g_hTask, &udata);
	HandleDAQError(e);
	ROS_WARN("Onboard buffer size=%u", udata);
	e = DAQmxGetAOUseOnlyOnBrdMem(g_hTask, szPhysicalChannel, &udata);
	HandleDAQError(e);
	ROS_WARN("Use only onboard memory=%u", udata);
	e = DAQmxGetAODataXferMech(g_hTask, szPhysicalChannel, &data);
	HandleDAQError(e);
	ROS_WARN("Data transfer mechanism=%d", data);
	e = DAQmxGetAODataXferReqCond(g_hTask, szPhysicalChannel, &data);
	HandleDAQError(e);
	ROS_WARN("Data transfer request condition=%d", data);
	e = DAQmxGetAOUsbXferReqSize(g_hTask, szPhysicalChannel, &udata);
	HandleDAQError(e);
	ROS_WARN("USB transfer request size=%u", udata);
	e = DAQmxGetAOUsbXferReqCount(g_hTask, szPhysicalChannel, &udata);
	HandleDAQError(e);
	ROS_WARN("USB transfer request count=%u", udata);
	
	//ros::spin();
	double 			timeAllowed = ros::Duration(2.0).toSec();
	double 			timeTaken;
	std_msgs::Time	heartbeat;
	int				iCount=0;
	int				nSpin=(int)hzSpin;
	
	
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
		if (!(iCount % nSpin))
		{
			heartbeat.data = ros::Time::now();
			pubHeartbeat.publish(heartbeat);
		}
		
		iCount++;
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
