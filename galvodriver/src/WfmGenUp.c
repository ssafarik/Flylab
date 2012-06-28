/*********************************************************************
*
* ANSI C Example program:
*    ContGen-IntClk.c
*
* Example Category:
*    AO
*
* Description:
*    This example demonstrates how to output two continuous periodic
*    waveforms with a frequency sweep using an internal sample clock.
*
* Instructions for Running:
*    1. Select the Physical Channel to correspond to where your
*       signal is output on the DAQ device.
*    2. Enter the Minimum and Maximum Voltage Ranges.
*    3. Enter the desired rate for the generation. The onboard sample
*       clock will operate at this rate.
*	 4. Enter the desired frequency for the periodic waveforms to generate.
*
* Steps:
*    1. Create a task.
*    2. Create two Analog Output Voltage channels.
*    3. Define the update Rate for the Voltage generation.
*       Additionally, define the sample mode to be continuous.
*	 4. Disable de regenaration mode.
* 	 5. Register a callback on Every N Samples generated.
*	 6. Double the size of the buffer.
*    7. Write the waveform to the output buffer (twice the size of SAMPLESNB).
*    8. Call the Start function.
*    9. Wait for the ENTER key.
*	 10. Call the Stop Task function to stop the Task.
* 	 11. Call the Clear Task function to clear the Task.
*    12. Display an error if any.
*
* I/O Connections Overview:
*    Make sure your signal output terminal matches the Physical
*    Channel I/O Control. For further connection information, refer
*    to your hardware reference manual.
*
*********************************************************************/

#include <NIDAQmx.h>
#include <stdio.h>
#include <math.h>

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

#define PI			3.1415926535
#define SAMPLESNB	50000

int32 CVICALLBACK OnEveryNSamplesEvent (TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void *callbackData);
void 	GenerateBuffer (void);

TaskHandle  genTaskHandle=0;
float64     data[2*SAMPLESNB];
float64		rate = 1000000.0;
float64 	frequency1 = 1;				// frequency of the periodic waveform for channel 1
float64 	frequency2 = 0.1;			// frequency of the periodic waveform for channel 2
char		channels[256]="Dev2/ao0:1";


/*********************************************/
// MAIN
/*********************************************/

int main(void)
{
	static int32   error=0;
	static char    errBuff[2048]={'\0'};

	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	DAQmxErrChk (DAQmxCreateTask("",&genTaskHandle));
	DAQmxErrChk (DAQmxCreateAOVoltageChan (genTaskHandle, channels, "", -10.0, 10.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk (DAQmxCfgSampClkTiming (genTaskHandle, "", rate, DAQmx_Val_Rising, DAQmx_Val_ContSamps,
										SAMPLESNB));
	// Disable generation to be warned if the output buffer is not refreshed fast enough
	// It prevents from generating discontinuity on the signal
	DAQmxErrChk (DAQmxSetWriteAttribute (genTaskHandle, DAQmx_Write_RegenMode, DAQmx_Val_DoNotAllowRegen));
	// Register a callback on Every N Samples generated so that we can be trigged to refresh the output buffer.
	DAQmxErrChk (DAQmxRegisterEveryNSamplesEvent (genTaskHandle, DAQmx_Val_Transferred_From_Buffer, SAMPLESNB, 0, OnEveryNSamplesEvent, NULL));
	// Double the size of the output buffer to provide enough sample at the generation start
	// (usefull for high an update rate)
	DAQmxCfgOutputBuffer (genTaskHandle, 2*SAMPLESNB);
	
	/*********************************************/
	// DAQmx Write Code
	/*********************************************/
	GenerateBuffer();
	DAQmxErrChk (DAQmxWriteAnalogF64 (genTaskHandle, SAMPLESNB, 0, 10.0, DAQmx_Val_GroupByScanNumber, data, NULL,
									  NULL));
	GenerateBuffer();
	DAQmxErrChk (DAQmxWriteAnalogF64 (genTaskHandle, SAMPLESNB, 0, 10.0, DAQmx_Val_GroupByScanNumber, data, NULL,
									  NULL));

	/*********************************************/
	// DAQmx Start Code
	/*********************************************/
	DAQmxErrChk (DAQmxStartTask(genTaskHandle));
	
	printf("Generating voltage continuously. Press Enter to interrupt\n"); 
	getchar();

Error:
	if( DAQmxFailed(error) )
		DAQmxGetExtendedErrorInfo(errBuff,2048);
	if( genTaskHandle!=0 ) {
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(genTaskHandle);
		DAQmxClearTask(genTaskHandle);
		genTaskHandle=0;
	}
	if( DAQmxFailed(error) )
		printf("DAQmx Error: %s\n",errBuff);
	printf("End of program, press Enter key to quit\n");
	getchar();
	return 0;
}

/*********************************************/
// OnEveryNSamplesEvent
/*********************************************/

int32 CVICALLBACK OnEveryNSamplesEvent (TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void *callbackData)
{
	static int32   	error=0;
	static char    	errBuff[2048]={'\0'};
	static int		NbOfSamplesWritten=0, totalNbOfSamplesWritten=0;

	/*********************************************/
	// DAQmx Write Code
	/*********************************************/
	if( taskHandle!=0 )
	{
		GenerateBuffer();	
		  
		DAQmxErrChk (DAQmxWriteAnalogF64 (taskHandle, SAMPLESNB, 0, 10.0, DAQmx_Val_GroupByScanNumber, data,
										  &NbOfSamplesWritten, NULL));
		totalNbOfSamplesWritten += NbOfSamplesWritten;
		printf("%d samples written per channel.\n", totalNbOfSamplesWritten);
	}
	
Error:
	if( DAQmxFailed(error) )
	{
		DAQmxGetExtendedErrorInfo(errBuff,2048);
		printf("DAQmx Error: %s\n",errBuff); 
		DAQmxClearTask(taskHandle);
		genTaskHandle=0;
	}
	return 0;	
}

/*********************************************/
// Modulo function
/*********************************************/

double ModuloDouble(double value, double modulo)
{
	double retval;
	retval = value;
	if(modulo < 0) // YG must be positive
		modulo = -modulo;
	if(retval > 0) 
		while(retval >= modulo)
			retval -= modulo;
	else if(retval < 0)
		while(retval <= modulo)
			retval += modulo;
	return retval;
}

/*********************************************/
// Waveform generation
/*********************************************/

void GenerateBuffer (void)
{
	static int i;
	static double phase1=0;
	static double phase2=0;
	
	frequency1 /= 1.001;
	frequency2 *= 1.001;
	
	for(i=0;i<SAMPLESNB;i++)
	{
		data[2*i] 	= 9.95*sin(phase1 + (double)i*2.0*PI/(rate/frequency1));
		data[2*i+1] = 9.95*sin(phase2 + (double)i*2.0*PI/(rate/frequency2));
	}
		
	phase1 += (2*PI*(frequency1*SAMPLESNB)/rate);
	phase1 = ModuloDouble(phase1, 2*PI);
	phase2 += (2*PI*(frequency2*SAMPLESNB)/rate);
	phase2 = ModuloDouble(phase2, 2*PI);
}
