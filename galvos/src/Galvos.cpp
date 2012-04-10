#include "ros/ros.h"
#include "std_msgs/String.h"
#include "NIDAQmxBase.h"
#include <sstream>


TaskHandle 	 ghTask;
uInt64 gnPoints=0;


void WriteFlyLogo(void)
{
	int32		 nPointsWritten = 0;

	gnPoints = 52;
	float64 dataPoints[] = // The fly logo.
	{
			 -4.0,  14.0,
			 -3.0,  11.0,
			 -7.0,  7.0,
			 -6.0,   5.0,
			 -7.0,   4.0,
			 -9.0,   4.0,
			-12.0,   6.0,
			 -9.0,  4.0,
			-13.0,   3.0,
			-14.0,   0.0,
			-11.0,   1.5,
			 -9.0,   4.0,
			-11.0,  0.0,
			 -9.0,  -4.0,
			-11.0,  -1.5,
			-14.0,   0.0,
			-13.0,  -3.0,
			 -9.0, -4.0,
			-12.0,  -6.0,
			 -9.0,  -4.0,
			 -7.0,  -4.0,
			 -6.0,  -5.0,
			 -7.0, -7.0,
			 -3.0, -11.0,
			 -4.0, -14.0,
			 -3.0, -11.0,
			 -7.0,  -7.0,
			 -6.0, -5.0,
			 -3.0,  -7.0,
			  3.0, -13.0,
			 13.0, -15.0,
			 16.0, -12.0,
			 12.0, -6.0,
			  4.0,  -2.0,
			  0.0,  -1.0,
			 -5.0,  -4.0,
			 -2.0,   0.0,
			 -5.0,  4.0,
			  0.0,   1.0,
			  4.0,   2.0,
			  4.0,  -2.0,
			  7.0,   0.0,
			  4.0,  2.0,
			 12.0,   6.0,
			 16.0,  12.0,
			 13.0,  15.0,
			  3.0,  13.0,
			 -3.0,  7.0,
			 -6.0,   5.0,
			 -7.0,   7.0,
			 -3.0,  11.0,
			 -4.0,  14.0
	};

	// Scale to [-10,+10].
	for (int k=0; k<104; k++)
		dataPoints[k] /= 1.60; // =10*x/xmax=10*x/16

	//Compute the desired waveform, using the buffer size and DC value.
	DAQmxBaseWriteAnalogF64 (ghTask, (int32)gnPoints, TRUE, 1.0, DAQmx_Val_GroupByScanNumber, dataPoints, &nPointsWritten, NULL);

}

void GalvoPointListCallback(const std_msgs::String::ConstPtr& msg)
{

}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "GalvoScanner");
	ros::NodeHandle n;

	//ros::Publisher pubGalvoScanner = n.advertise<std_msgs::String>("GalvoScanner", 1000);
	ros::Subscriber subGalvoScanner = n.subscribe("ScannerPoints", 1000, GalvoPointListCallback);
	ros::Rate loop_rate(10);

	char 		szTask[]="OutputPointList";
	char		szPhysicalChannel[]="Dev1/ao0:1";
	float64		 ratePoint = 100.0; //rateFrame * gnPoints;
	float64		 ratePointActual = 0.0;


	//DAQmxCreateTask (szTask, &ghTask);
	//DAQmxCreateAOVoltageChan (ghTask, szPhysicalChannel, NULL, -10.0, +10.0, DAQmx_Val_Volts, NULL);
	//DAQmxSetWriteRegenMode(ghTask, DAQmx_Val_AllowRegen);
	//DAQmxCfgSampClkTiming (ghTask, "OnboardClock", ratePoint, DAQmx_Val_Rising, DAQmx_Val_ContSamps, gnPoints);
	//DAQmxGetSampClkRate(ghTask, &ratePointActual);
	//WriteFlyLogo();
	//DAQmxStartTask (ghTask);
	//ros::spin();
	//DAQmxClearTask (ghTask);

	DAQmxBaseCreateTask (szTask, &ghTask);
	DAQmxBaseCreateAOVoltageChan (ghTask, szPhysicalChannel, NULL, -10.0, +10.0, DAQmx_Val_Volts, NULL);
	DAQmxBaseCfgSampClkTiming (ghTask, "OnboardClock", ratePoint, DAQmx_Val_Rising, DAQmx_Val_ContSamps, gnPoints);
	WriteFlyLogo();
	DAQmxBaseStartTask (ghTask);
	ros::spin();
	DAQmxBaseClearTask (ghTask);

	//while (ros::ok())
	//{
	//	ros::spinOnce();
	//	loop_rate.sleep();
	//}


	return 0;
}
