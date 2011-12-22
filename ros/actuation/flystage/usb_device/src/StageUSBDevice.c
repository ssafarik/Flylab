/*
  LUFA Library
  Copyright (C) Dean Camera, 2009.

  dean [at] fourwalledcubicle [dot] com
  www.fourwalledcubicle.com
*/

/*
  Copyright 2009  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, and distribute this software
  and its documentation for any purpose and without fee is hereby
  granted, provided that the above copyright notice appear in all
  copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the GenericHID demo. This file contains the main tasks of the demo and
 *  is responsible for the initial application hardware configuration.
 */

#define INCLUDE_FROM_STAGEUSBDEVICE_C
#include "StageUSBDevice.h"

/* Scheduler Task List */
TASK_LIST
{
  { .Task = USB_USBTask          , .TaskStatus = TASK_STOP },
    { .Task = USB_ProcessPacket    , .TaskStatus = TASK_STOP },
      };

/* DFU Bootloader Declarations */
uint32_t  boot_key __attribute__ ((section (".noinit")));
typedef void (*AppPtr_t)(void) __attribute__ ((noreturn));
AppPtr_t Bootloader = (AppPtr_t)0xf000;

/** Main program entry point. This routine configures the hardware required by the application, then
 *  starts the scheduler to run the USB management task.
 */
int main(void)
{
  /* After reset start bootloader? */
  if ((AVR_IS_WDT_RESET()) && (boot_key == DFU_BOOT_KEY_VAL))
    {
      boot_key = 0;
      Bootloader();
    }

  /* Disable watchdog if enabled by bootloader/fuses */
  MCUSR &= ~(1 << WDRF);
  wdt_disable();

  /* Disable clock division */
  clock_prescale_set(clock_div_1);

  /* Hardware Initialization */
  LEDs_Init();

  /* Indicate USB not ready */
  UpdateStatus(Status_USBNotReady);

  /* Initialize Scheduler so that it can be used */
  Scheduler_Init();

  /* Initialize USB Subsystem */
  USB_Init();

  /* Initialize I/O lines */
  IO_Init();

  /* Initialize Timers */
  Timer_Init();

  /* Initialize Motors */
  Motor_Init();

  /* Initialize Software Interrupt */
  Interrupt_Init();

  /* Scheduling - routine never returns, so put this last in the main function */
  Scheduler_Start();
}

/** Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Connect(void)
{
  /* Start USB management task */
  Scheduler_SetTaskMode(USB_USBTask, TASK_RUN);

  /* Indicate USB enumerating */
  UpdateStatus(Status_USBEnumerating);

}

/** Event handler for the USB_Disconnect event. This indicates that the device is no longer connected to a host via
 *  the status LEDs and stops the USB management task.
 */
void EVENT_USB_Disconnect(void)
{
  /* Stop running HID reporting and USB management tasks */
  Scheduler_SetTaskMode(USB_ProcessPacket, TASK_STOP);
  Scheduler_SetTaskMode(USB_USBTask, TASK_STOP);

  /* Indicate USB not ready */
  UpdateStatus(Status_USBNotReady);
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host sets the current configuration
 *  of the USB device after enumeration, and configures the generic HID device endpoints.
 */
void EVENT_USB_ConfigurationChanged(void)
{
  /* Setup USB In and Out Endpoints */
  Endpoint_ConfigureEndpoint(OUT_EPNUM, EP_TYPE_BULK,
                             ENDPOINT_DIR_OUT, OUT_EPSIZE,
                             ENDPOINT_BANK_SINGLE);

  Endpoint_ConfigureEndpoint(IN_EPNUM, EP_TYPE_BULK,
                             ENDPOINT_DIR_IN, IN_EPSIZE,
                             ENDPOINT_BANK_SINGLE);

  /* Home Motors */
  if (!MotorsHomed)
    {
      AllMotorsInPosition = FALSE;
      LookupTableMoveInProgress = FALSE;
      HomeInProgress = TRUE;
      MotorsHomed = FALSE;
      Motor_Home();
    }

  /* Indicate USB connected and ready */
  UpdateStatus(Status_USBReady);

  /* Start ProcessPacket task */
  Scheduler_SetTaskMode(USB_ProcessPacket, TASK_RUN);
}

/** Function to manage status updates to the user. This is done via LEDs on the given board, if available, but may be changed to
 *  log to a serial port, or anything else that is suitable for status updates.
 */
void UpdateStatus(uint8_t CurrentStatus)
{
  uint8_t LEDMask = LEDS_NO_LEDS;

  /* Set the LED mask to the appropriate LED mask based on the given status code */
  switch (CurrentStatus)
    {
    case Status_USBNotReady:
      LEDMask = (LEDS_LED1);
      break;
    case Status_USBEnumerating:
      LEDMask = (LEDS_LED1 | LEDS_LED2);
      break;
    case Status_USBReady:
      LEDMask = (LEDS_LED2 | LEDS_LED4);
      break;
    case Status_ProcessingPacket:
      LEDMask = (LEDS_LED1 | LEDS_LED2);
      break;
    }

  /* Set the board LEDs to the new LED mask */
  LEDs_SetAllLEDs(LEDMask);
}

TASK(USB_ProcessPacket)
{
  /* Check if the USB System is connected to a Host */
  if (USB_IsConnected)
    {
      /* Select the Data Out Endpoint */
      Endpoint_SelectEndpoint(OUT_EPNUM);

      /* Check if OUT Endpoint contains a packet */
      if (Endpoint_IsOUTReceived())
        {
          /* Check to see if a command from the host has been issued */
          if (Endpoint_IsReadWriteAllowed())
            {
              /* Indicate busy */
              UpdateStatus(Status_ProcessingPacket);

              /* Read USB packet from the host */
              USBPacket_Read();

              /* Return the same CommandID that was received */
              USBPacketIn.CommandID = USBPacketOut.CommandID;

              /* Process USB packet */
              switch (USBPacketOut.CommandID)
                {
                case USB_CMD_AVR_RESET:
                  {
                    Write_Return_USBPacket();
                    AVR_RESET();
                  }
                  break;
                case USB_CMD_AVR_DFU_MODE:
                  {
                    Write_Return_USBPacket();
                    boot_key = DFU_BOOT_KEY_VAL;
                    AVR_RESET();
                  }
                  break;
                case USB_CMD_GET_STATE:
                  {
                    Write_Return_USBPacket();
                  }
                  break;
                case USB_CMD_SET_STATE:
                  {
                    if (!IO_Enabled)
                      {
                        IO_Init();
                      }
                    MotorUpdateBits = USBPacketOut.MotorUpdate;
                    LookupTablePosMove = FALSE;
                    LookupTableVelMove = FALSE;
                    AllMotorsInPosition = FALSE;
                    LookupTableMoveInProgress = FALSE;
                    Write_Return_USBPacket();
                    Motor_Set_Values_All(USBPacketOut.Setpoint[0]);
                    Motor_Update_All();
                  }
                  break;
                case USB_CMD_HOME:
                  {
                    if (!IO_Enabled)
                      {
                        IO_Init();
                      }
                    MotorUpdateBits = USBPacketOut.MotorUpdate;
                    AllMotorsInPosition = FALSE;
                    LookupTableMoveInProgress = FALSE;
                    HomeInProgress = TRUE;
                    MotorsHomed = FALSE;
                    Write_Return_USBPacket();
                    Motor_Home();
                  }
                  break;
                case USB_CMD_LOOKUP_TABLE_FILL:
                  {
                    LookupTablePosMove = FALSE;
                    LookupTableVelMove = FALSE;
                    AllMotorsInPosition = FALSE;
                    LookupTableMoveInProgress = TRUE;
                    Write_Return_USBPacket();
                    if (USBPacketOut.EntryLocation < LOOKUP_TABLE_SIZE)
                      {
                        Lookup_Table_Fill(USBPacketOut.Setpoint,USBPacketOut.EntryCount,USBPacketOut.EntryLocation);
                      }
                  }
                  break;
                case USB_CMD_LOOKUP_TABLE_POS_MOVE:
                  {
                    if (!IO_Enabled)
                      {
                        IO_Init();
                      }
                    MotorUpdateBits = USBPacketOut.MotorUpdate;
                    LookupTablePosMove = TRUE;
                    AllMotorsInPosition = FALSE;
                    LookupTableMoveInProgress = TRUE;
                    Write_Return_USBPacket();
                    TableEntry = 0;
                    if (TableEntry < TableEnd)
                      {
                        TableEntry++;
                        Motor_Set_Values_All(LookupTable[TableEntry-1]);
                        Motor_Update_All();
                      }
                    else
                      {
                        LookupTablePosMove = FALSE;
                      }
                  }
                  break;
                case USB_CMD_LOOKUP_TABLE_VEL_MOVE:
                  {
                    if (!IO_Enabled)
                      {
                        IO_Init();
                      }
                    MotorUpdateBits = USBPacketOut.MotorUpdate;
                    LookupTableVelMove = TRUE;
                    AllMotorsInPosition = FALSE;
                    LookupTableMoveInProgress = TRUE;
                    Write_Return_USBPacket();
                    TableEntry = 0;
                    Timer_On(0);
                  }
                  break;
                default:
                  {
                    Write_Return_USBPacket();
                  }
                }

              /* Indicate ready */
              LEDs_SetAllLEDs(LEDS_LED2 | LEDS_LED4);
            }
        }
    }
}

/* Write the return USB packet */
static void Write_Return_USBPacket(void)
{
  for ( uint8_t Motor_N=0; Motor_N<MOTOR_NUM; Motor_N++ )
    {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
      {
        USBPacketIn.MotorStatus[Motor_N].Frequency = Motor[Motor_N].Frequency;
        USBPacketIn.MotorStatus[Motor_N].Position = Motor[Motor_N].Position;
      }
    }
  USBPacketIn.AllMotorsInPosition = AllMotorsInPosition;
  USBPacketIn.LookupTableMoveInProgress = LookupTableMoveInProgress;
  USBPacketIn.HomeInProgress = HomeInProgress;
  USBPacketIn.MotorsHomed = MotorsHomed;
  USBPacket_Write();
}

static void USBPacket_Read(void)
{
  uint8_t* USBPacketOutPtr = (uint8_t*)&USBPacketOut;

  /* Select the Data Out endpoint */
  Endpoint_SelectEndpoint(OUT_EPNUM);

  /* Read in USB packet header */
  Endpoint_Read_Stream_LE(USBPacketOutPtr, sizeof(USBPacketOut));

  /* Finalize the stream transfer to send the last packet */
  Endpoint_ClearOUT();
}

static void USBPacket_Write(void)
{
  uint8_t* USBPacketInPtr = (uint8_t*)&USBPacketIn;

  /* Select the Data In endpoint */
  Endpoint_SelectEndpoint(IN_EPNUM);

  /* Wait until read/write to IN data endpoint allowed */
  while (!(Endpoint_IsReadWriteAllowed() && Endpoint_IsINReady()));

  /* Write the return data to the endpoint */
  Endpoint_Write_Stream_LE(USBPacketInPtr, sizeof(USBPacketIn));

  /* Finalize the stream transfer to send the last packet */
  Endpoint_ClearIN();
}

static void IO_Init(void)
{
  /* Input lines initialization */

  /* Set data direction of pins 0:2 on PORTD to input for HOME lines */
  DDRD &= ~((1<<DDD0) | (1<<DDD1) | (1<<DDD2));

  /* Enable pull-up resistors on pins 0:2 on PORTD */
  PORTD |= ((1<<PD0) | (1<<PD1) | (1<<PD2));

  /* Disable Pull-up disable */
  MCUCR &= ~(1<<PUD);

  /* Output lines initialization */

  /* Set data direction of Timer 0 PWM output A to output (PORTB pin 7) */
  DDRB |= (1<<DDB7);

  /* Set data direction of Timer 1 PWM output A to output (PORTB pin 5) */
  DDRB |= (1<<DDB5);

  /* Set data direction of Timer 2 PWM output A to output (PORTB pin 4) */
  DDRB |= (1<<DDB4);

  /* Set data direction of Timer 3 PWM output A to output (PORTC pin 6) */
  DDRC |= (1<<DDC6);

  /* Set data direction of Direction pins to output (PORTC pin 0:2) */
  DDRC |= ((1<<DDC0) | (1<<DDC1) | (1<<DDC2));

  /* Set Timer PWM and Direction pins low to start (PORTB pins 4,5,7 PORTC pin 0:2,6) */
  PORTB &= ~((1<<PB4) | (1<<PB5) | (1<<PB7));
  PORTC &= ~((1<<PC0) | (1<<PC1) | (1<<PC2) | (1<<PC6));

  /* Set data direction of interrupt 4 to output (PORTE pin 4) */
  DDRE |= (1<<DDE4);

  /* Set interrupt 4 high to start (PORTE pin 4) */
  PORTE |= (1<<PE4);

  /* Set data direction of InPositionPin to output (PORTE pin 5) */
  DDRE |= (1<<DDE5);

  /* /\* Set InPositionPin low to start (PORTE pin 5) *\/ */
  /* PORTE &= ~(1<<PE5); */

  /* Set InPositionPin high (PORTE pin 5) */
  PORTE |= (1<<PE5);

  IO_Enabled = TRUE;
}

static void Interrupt_Init(void)
{
  /* External interrupts initialization */
  /* Disable external interrupt pin 4 before changing interrupt sense control */
  EIMSK &= ~(1<<INT4);

  /* Set external interrupt pin 4 to low level */
  EICRB &= ~((1<<ISC40) | (1<<ISC41));

  /* Enable external interrupt pins 4 */
  EIFR  |= (1<<INTF4);
  EIMSK |= (1<<INT4);

  Interrupt_Enabled = TRUE;
}

static void Timer_Init(void)
{
  /* Timers hardware initialization */

  /* Toggle OC0A on compare match, OC0B disconnected */
  /* Set Phase Correct PWM Mode on Timer 0 */
  /* Top @ OCR0A, Update of OCR0x @ TOP, TOV0 Flag Set on BOTTOM */
  TCCR0A = ((1<<COM0A0) | (1<<WGM00));
  TCCR0B = (1<<WGM02);

  /* Toggle OC1A on compare match, OC1B/OC1C disconnected */
  /* Set Phase and Frequency Correct PWM Mode on Timer 1 */
  /* Top @ OCR1A, Update of OCR1x @ BOTTOM, TOV1 Flag Set on BOTTOM */
  TCCR1A = ((1<<COM1A0) | (1<<COM1B0) | (1<<COM1C0) | (1<<WGM10));
  TCCR1B = (1<<WGM13);

  /* Toggle OC2A on compare match, OC2B disconnected */
  /* Set Phase Correct PWM Mode on Timer 2 */
  /* Top @ OCR2A, Update of OCR2x @ TOP, TOV2 Flag Set on BOTTOM */
  TCCR2A = ((1<<COM2A0) | (1<<WGM20));
  TCCR2B = (1<<WGM22);

  /* Toggle OC3A on compare match, OC3B/OC3C disconnected */
  /* Set Phase and Frequency Correct PWM Mode on Timer 3 */
  /* Top @ OCR3A, Update of OCR3x @ BOTTOM, TOV3 Flag Set on BOTTOM */
  TCCR3A = ((1<<COM3A0) | (1<<COM3B0) | (1<<COM3C0) | (1<<WGM30));
  TCCR3B = (1<<WGM33);

  /* Enable Timer Interrupts */
  TIMSK0 = (1<<TOIE0);
  TIMSK1 = (1<<TOIE1);
  TIMSK2 = (1<<TOIE2);
  TIMSK3 = (1<<TOIE3);

  /* Store Timer Addresses used for setting Timer frequency and prescaler */
  Timer[0].Address.TOP = (uint16_t*)&OCR0A;
  Timer[0].Address.ClockSelect = &TCCR0B;
  Timer[0].Address.PinPort = &PINB;
  Timer[0].OutputPin = DDB7;

  Timer[1].Address.TOP = &OCR1A;
  Timer[1].Address.ClockSelect = &TCCR1B;
  Timer[1].Address.PinPort = &PINB;
  Timer[1].OutputPin = DDB5;

  Timer[2].Address.TOP = (uint16_t*)&OCR2A;
  Timer[2].Address.ClockSelect = &TCCR2B;
  Timer[2].Address.PinPort = &PINB;
  Timer[2].OutputPin = DDB4;

  Timer[3].Address.TOP = &OCR3A;
  Timer[3].Address.ClockSelect = &TCCR3B;
  Timer[3].Address.PinPort = &PINC;
  Timer[3].OutputPin = DDC6;

  /* Store ClockSelect Values for each Prescaler_N */
  Timer[0].ClockSelect[0] = (1<<CS00);
  Timer[0].ClockSelect[1] = (1<<CS01);
  Timer[0].ClockSelect[2] = ((1<<CS01)|(1<<CS00));
  Timer[0].ClockSelect[3] = (1<<CS02);
  Timer[0].ClockSelect[4] = ((1<<CS02)|(1<<CS00));
  Timer[0].ClockSelect[5] = ~((1<<CS02)|(1<<CS01)|(1<<CS00));

  Timer[1].ClockSelect[0] = (1<<CS10);
  Timer[1].ClockSelect[1] = (1<<CS11);
  Timer[1].ClockSelect[2] = ((1<<CS11)|(1<<CS10));
  Timer[1].ClockSelect[3] = (1<<CS12);
  Timer[1].ClockSelect[4] = ((1<<CS12)|(1<<CS10));
  Timer[1].ClockSelect[5] = ~((1<<CS12)|(1<<CS11)|(1<<CS10));

  Timer[2].ClockSelect[0] = (1<<CS20);
  Timer[2].ClockSelect[1] = (1<<CS21);
  Timer[2].ClockSelect[2] = ((1<<CS21)|(1<<CS20));
  Timer[2].ClockSelect[3] = (1<<CS22);
  Timer[2].ClockSelect[4] = ((1<<CS22)|(1<<CS20));
  Timer[2].ClockSelect[5] = ~((1<<CS22)|(1<<CS21)|(1<<CS20));

  Timer[3].ClockSelect[0] = (1<<CS30);
  Timer[3].ClockSelect[1] = (1<<CS31);
  Timer[3].ClockSelect[2] = ((1<<CS31)|(1<<CS30));
  Timer[3].ClockSelect[3] = (1<<CS32);
  Timer[3].ClockSelect[4] = ((1<<CS32)|(1<<CS30));
  Timer[3].ClockSelect[5] = ~((1<<CS32)|(1<<CS31)|(1<<CS30));

  /* Set TOPMax values for each Timer */
  Timer[0].TOPMax = 255;
  Timer[1].TOPMax = 65535;
  Timer[2].TOPMax = 255;
  Timer[3].TOPMax = 65535;

  /* Set ScaleFactor for each timer
     Freq_out = F_CLOCK/(Prescaler*ScaleFactor*TOP) */
  Timer[0].ScaleFactor = 4;
  Timer[1].ScaleFactor = 4;
  Timer[2].ScaleFactor = 4;
  Timer[3].ScaleFactor = 4;

  /* Set Timer 0 to 62.5 Hz */
  Timer_Off(0);
  Timer[0].TOPValue = (uint16_t)250;
  Timer[0].Prescaler_N = 3;
  *Timer[0].Address.TOP = (uint8_t)Timer[0].TOPValue;
}

static void Motor_Init(void)
{
  /* Initialize Motor Variables */
  Motor[0].Timer = MOTOR_0_TIMER;
  Motor[0].DirectionPort = &PORTC;
  Motor[0].DirectionPin = PC0;
  Motor[0].HomePort = &PIND;
  Motor[0].HomePin = DDD0;
  Motor[0].HomeEIMSK = INT0;
  Motor[0].HomeEIFR = INTF0;
  Motor[0].Frequency = 0;
  Motor[0].FrequencyMax = MOTOR_0_FREQUENCY_MAX;
  Motor[0].Direction = 0;
  Motor[0].DirectionPos = MOTOR_0_DIRECTION_POS;
  Motor[0].DirectionNeg = MOTOR_0_DIRECTION_NEG;
  Motor[0].Position = MOTOR_0_POSITION_HOME;
  Motor[0].PositionSetPoint = MOTOR_0_POSITION_HOME;
  Motor[0].Update = TRUE;
  Motor[0].InPosition = TRUE;
  Motor[0].HomeInProgress = FALSE;
  Motor[0].HomeSet = FALSE;
  Motor[0].PositionLimitMax = 44000;
  Motor[0].PositionLimitMin = 1000;
  Motor[0].PositionLimitsEnabled = FALSE;

  Motor[1].Timer = MOTOR_1_TIMER;
  Motor[1].DirectionPort = &PORTC;
  Motor[1].DirectionPin = PC1;
  Motor[1].HomePort = &PIND;
  Motor[1].HomePin = DDD1;
  Motor[1].HomeEIMSK = INT1;
  Motor[1].HomeEIFR = INTF1;
  Motor[1].Frequency = 0;
  Motor[1].FrequencyMax = MOTOR_1_FREQUENCY_MAX;
  Motor[1].Direction = 0;
  Motor[1].DirectionPos = MOTOR_1_DIRECTION_POS;
  Motor[1].DirectionNeg = MOTOR_1_DIRECTION_NEG;
  Motor[1].Position = MOTOR_1_POSITION_HOME;
  Motor[1].PositionSetPoint = MOTOR_1_POSITION_HOME;
  Motor[1].Update = TRUE;
  Motor[1].InPosition = TRUE;
  Motor[1].HomeInProgress = FALSE;
  Motor[1].HomeSet = FALSE;
  Motor[1].PositionLimitMax = 44000;
  Motor[1].PositionLimitMin = 1000;
  Motor[1].PositionLimitsEnabled = FALSE;

  Motor[2].Timer = MOTOR_2_TIMER;
  Motor[2].DirectionPort = &PORTC;
  Motor[2].DirectionPin = PC2;
  Motor[2].HomePort = &PIND;
  Motor[2].HomePin = DDD2;
  Motor[2].HomeEIMSK = INT2;
  Motor[2].HomeEIFR = INTF2;
  Motor[2].Frequency = 0;
  Motor[2].FrequencyMax = MOTOR_2_FREQUENCY_MAX;
  Motor[2].Direction = 0;
  Motor[2].DirectionPos = MOTOR_2_DIRECTION_POS;
  Motor[2].DirectionNeg = MOTOR_2_DIRECTION_NEG;
  Motor[2].Position = MOTOR_2_POSITION_HOME;
  Motor[2].PositionSetPoint = MOTOR_2_POSITION_HOME;
  Motor[2].Update = TRUE;
  Motor[2].InPosition = TRUE;
  Motor[2].HomeInProgress = FALSE;
  Motor[2].HomeSet = FALSE;
  Motor[2].PositionLimitMax = UINT16_MAX;
  Motor[2].PositionLimitMin = UINT16_MIN;
  Motor[2].PositionLimitsEnabled = FALSE;

  /* Update Motors */
  Motor_Update_All();
}

static void IO_Disconnect(void)
{
  /* Set data direction of PORTB pins 4,5,7 to input to reduce current draw */
  DDRB &= ~((1<<DDB4) | (1<<DDB5) | (1<<DDB7));

  /* Set data direction of PORTC pins 0:2,6 to input to reduce current draw */
  DDRC &= ~((1<<DDC0) | (1<<DDC1) | (1<<DDC2) | (1<<DDC6));

  /* Disable external interrupt pin 4 before changing interrupt sense control */
  EIMSK &= ~(1<<INT4);

  /* Set data direction of PORTE pin 4 to input to reduce current draw */
  DDRE &= ~(1<<DDE4);
}

static void Timer_On(uint8_t Timer_N)
{
  /* Turn on Timer_N by writing appropriate ClockSelect value
     to the ClockSelect Register */
  *Timer[Timer_N].Address.ClockSelect |= Timer[Timer_N].ClockSelect[Timer[Timer_N].Prescaler_N];

  /* Timer[Timer_N] is turned on */
  Timer[Timer_N].On = TRUE;
}

static void Timer_Off(uint8_t Timer_N)
{
  /* Turn off Timer_N by writing appropriate ClockSelect value
     to the ClockSelect Register */
  *Timer[Timer_N].Address.ClockSelect &= Timer[Timer_N].ClockSelect[PRESCALER_NUM];

  /* Timer[Timer_N] is turned off */
  Timer[Timer_N].On = FALSE;
}

static void Motor_Home(void)
{
  LookupTableRow_t MotorHomeParameters;

  LookupTablePosMove = FALSE;
  LookupTableVelMove = FALSE;

  /* External interrupts initialization */
  /* Disable external interrupt pins 0:2 before changing interrupt sense control */
  EIMSK &= ~((1<<INT0) | (1<<INT1) | (1<<INT2));

  /* Set external interrupt pins 0:2 to either rising or falling edge */
  EICRA |= ((1<<ISC00) | (1<<ISC10) | (1<<ISC20));
  EICRA &= ~((1<<ISC01) | (1<<ISC11) | (1<<ISC21));

  /* Enable external interrupt pins 0:2 */
  /* EIMSK |= ((1<<INT0) | (1<<INT1) | (1<<INT2)); */
  EIFR  |= (1<<INTF0) | (1<<INTF1);
  EIMSK |= (1<<INT0) | (1<<INT1);

  Motor[0].HomeInProgress = TRUE;
  Motor[0].HomeSet = FALSE;
  Motor[0].Position = UINT16_MAX;
  MotorHomeParameters[0].Frequency = HOME_FREQUENCY_FAST;
  MotorHomeParameters[0].Position = 0;
  Motor_Set_Values(MotorHomeParameters,0);
  Motor_Update(0);

  Motor[1].HomeInProgress = TRUE;
  Motor[1].HomeSet = FALSE;
  Motor[1].Position = UINT16_MAX;
  MotorHomeParameters[1].Frequency = HOME_FREQUENCY_FAST-25;
  MotorHomeParameters[1].Position = 0;
  Motor_Set_Values(MotorHomeParameters,1);
  Motor_Update(1);
}

static void Motor_Update(uint8_t Motor_N)
{
  uint32_t TOPValue;
  uint8_t  Prescaler_N;
  uint16_t Prescaler=1;
  uint32_t ScaleFactor;
  uint8_t  Timer_N;
  uint16_t Freq;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    Freq = Motor[Motor_N].Frequency;
  }

  Timer_N = Motor[Motor_N].Timer;
  if ((Freq == 0) && Timer[Timer_N].On)
    {
      Timer_Off(Timer_N);
    }
  else if (Freq > 0)
    {
      if (Freq > Motor[Motor_N].FrequencyMax)
        {
          Freq = Motor[Motor_N].FrequencyMax;
        }
      ScaleFactor = Timer[Timer_N].ScaleFactor;
      TOPValue = (uint32_t)F_CLOCK/(ScaleFactor*(uint32_t)Freq);
      Prescaler_N = 0;
      while ((TOPValue > Timer[Timer_N].TOPMax) && (Prescaler_N < (PRESCALER_NUM-1)))
        {
          Prescaler_N++;
          if ((Timer_N == 1) || (Timer_N == 3))
            {
              Prescaler = PrescalerArray16[Prescaler_N];
            }
          else
            {
              Prescaler = PrescalerArray8[Prescaler_N];
            }
          TOPValue = (uint32_t)F_CLOCK/((uint32_t)Freq*(uint32_t)ScaleFactor*Prescaler);
          if ((Prescaler_N == (PRESCALER_NUM-1)) && (TOPValue > Timer[Timer_N].TOPMax))
            {
              TOPValue = Timer[Timer_N].TOPMax;
            }
        }
      Timer[Timer_N].TOPValue = (uint16_t)TOPValue;
      Freq = F_CLOCK/(Timer[Timer_N].TOPValue*ScaleFactor*Prescaler);
      Timer[Timer_N].Prescaler_N = Prescaler_N;

      Timer_Off(Timer_N);

      if ((Timer_N == 1) || (Timer_N == 3))
        {
          /* Set PWM frequency for the 16-bit Motor Timer */
          ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
          {
            *Timer[Timer_N].Address.TOP = Timer[Timer_N].TOPValue;
          }
        }
      else
        {
          /* Set PWM frequency for the 8-bit Motor Timer */
          *Timer[Timer_N].Address.TOP = (uint8_t)Timer[Timer_N].TOPValue;
        }

      /* Set Motor Direction pin */
      if (Motor[Motor_N].Direction)
        {
          *Motor[Motor_N].DirectionPort |= (1<<Motor[Motor_N].DirectionPin);
        }
      else
        {
          *Motor[Motor_N].DirectionPort &= ~(1<<Motor[Motor_N].DirectionPin);
        }

      /* Turn on Timer */
      Timer_On(Timer_N);
    }
  Motor[Motor_N].Update = FALSE;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    Motor[Motor_N].Frequency = Freq;
  }

}

static void Motor_Update_All(void)
{
  for ( uint8_t Motor_N=0; Motor_N<MOTOR_NUM; Motor_N++ )
    {
      Motor[Motor_N].Update = (MotorUpdateBits & (1<<Motor_N));
      if (Motor[Motor_N].Update)
        {
          Motor_Update(Motor_N);
        }
    }
}

static void Motor_Set_Values(LookupTableRow_t MotorSetpoint, uint8_t Motor_N)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if (MotorSetpoint[Motor_N].Frequency > Motor[Motor_N].FrequencyMax)
      {
        Motor[Motor_N].Frequency = Motor[Motor_N].FrequencyMax;
      }
    else
      {
        Motor[Motor_N].Frequency = MotorSetpoint[Motor_N].Frequency;
      }
    if (Motor[Motor_N].PositionLimitsEnabled)
      {
        if (Motor[Motor_N].PositionLimitMax < MotorSetpoint[Motor_N].Position)
          {
            Motor[Motor_N].PositionSetPoint = Motor[Motor_N].PositionLimitMax;
          }
        else if (MotorSetpoint[Motor_N].Position < Motor[Motor_N].PositionLimitMin)
          {
            Motor[Motor_N].PositionSetPoint = Motor[Motor_N].PositionLimitMin;
          }
        else
          {
            Motor[Motor_N].PositionSetPoint = MotorSetpoint[Motor_N].Position;
          }
      }
    else
      {
        Motor[Motor_N].PositionSetPoint = MotorSetpoint[Motor_N].Position;
      }
    Motor[Motor_N].InPosition = FALSE;
    if (Motor[Motor_N].PositionSetPoint > Motor[Motor_N].Position)
      {
        Motor[Motor_N].Direction = Motor[Motor_N].DirectionPos;
      }
    else if (Motor[Motor_N].PositionSetPoint < Motor[Motor_N].Position)
      {
        Motor[Motor_N].Direction = Motor[Motor_N].DirectionNeg;
      }
    else
      {
        Motor[Motor_N].Frequency = 0;
        Motor[Motor_N].InPosition = TRUE;
      }
  }
}

static void Motor_Set_Values_All(LookupTableRow_t MotorSetpoint)
{
  for ( uint8_t Motor_N=0; Motor_N<MOTOR_NUM; Motor_N++ )
    {
      Motor[Motor_N].Update = (MotorUpdateBits & (1<<Motor_N));
      if (Motor[Motor_N].Update)
        {
          Motor_Set_Values(MotorSetpoint,Motor_N);
        }
    }
  Motor_Check_In_Position();
}

static void Motor_Check_In_Position(void)
{
  /* If all motors are in position, set InPosition interrupt */
  if (Motor[0].InPosition && Motor[1].InPosition && Motor[2].InPosition)
    {
      AllMotorsInPosition = TRUE;

      if (LookupTablePosMove)
        {
          /* Set interrupt 4 low to enable interrupt (PORTE pin 4) */
          PORTE &= ~(1<<PE4);
        }
    }
}

static void Motor_Check_Home(void)
{
  /* If all motors are in position, set InPosition interrupt */
  if (!Motor[0].HomeInProgress && !Motor[1].HomeInProgress && !Motor[2].HomeInProgress)
    {
      MotorsHomed = TRUE;
      HomeInProgress = FALSE;
    }
}

static void Lookup_Table_Fill(LookupTableRow_t *LookupTableEntries,uint8_t EntryCount,uint8_t EntryLocation)
{
  /* This assignment assumes that entries are always appended onto the LookupTable, not inserted in the middle */
  TableEnd = EntryLocation;
  for ( uint8_t Entry_N=0; Entry_N<EntryCount; Entry_N++ )
    {
      if (Entry_N < LOOKUP_TABLE_SIZE)
        {
          for ( uint8_t Motor_N=0; Motor_N<MOTOR_NUM; Motor_N++ )
            {
              LookupTable[TableEnd][Motor_N] = LookupTableEntries[Entry_N][Motor_N];
            }
          TableEnd++;
        }
    }
}

#define POSITION_UPDATE(Motor_N)                                                   \
uint8_t Timer_N;                                                                   \
                                                                                   \
Timer_N = Motor[Motor_N].Timer;                                                    \
if (*Timer[Timer_N].Address.PinPort & (1<<Timer[Timer_N].OutputPin))               \
  {                                                                                \
    if (Motor[Motor_N].Direction == Motor[Motor_N].DirectionPos)                   \
      {                                                                            \
        Motor[Motor_N].Position += 1;                                              \
      }                                                                            \
    else                                                                           \
      {                                                                            \
        Motor[Motor_N].Position -= 1;                                              \
      }                                                                            \
    if (Motor[Motor_N].Position == Motor[Motor_N].PositionSetPoint)                \
      {                                                                            \
        Motor[Motor_N].Frequency = 0;                                              \
        Timer_Off(Motor[Motor_N].Timer);                                           \
        Motor[Motor_N].InPosition = TRUE;                                          \
        /* Add this to test drift problem... */                                    \
        *Motor[Motor_N].DirectionPort &= ~(1<<Motor[Motor_N].DirectionPin);        \
                                                                                   \
        if (Motor[Motor_N].HomeInProgress)                                         \
          {                                                                        \
            Motor[Motor_N].HomeInProgress = FALSE;                                 \
            Motor[Motor_N].HomeSet = TRUE;                                         \
            Motor[Motor_N].PositionLimitsEnabled = TRUE;                           \
            Motor_Check_Home();                                                    \
          }                                                                        \
                                                                                   \
        /* If all motors are in position, set InPosition interrupt */              \
        Motor_Check_In_Position();                                                 \
      }                                                                            \
    else if (Motor[Motor_N].InPosition)                                            \
      {                                                                            \
        Motor[Motor_N].InPosition = FALSE;                                         \
      }                                                                            \
  }                                                                                \
return                                                                             \

#define HOME(Motor_N)                                                              \
if (Motor[Motor_N].HomeInProgress)                                                 \
  {                                                                                \
    Motor[Motor_N].Frequency = 0;                                                  \
    Timer_Off(Motor[Motor_N].Timer);                                               \
                                                                                   \
    /* Disable external interrupt pin */                                           \
    EIMSK &= ~(1<<Motor[Motor_N].HomeEIMSK);                                       \
    EIFR  |= (1<<Motor[Motor_N].HomeEIFR);                                         \
                                                                                   \
    /* Reenable interrupts so code will not block */                               \
    sei();                                                                         \
                                                                                   \
    uint8_t ones=0, zeros=0, i;                                                    \
    for (i=0;i<9;i++)                                                              \
      {                                                                            \
        if (*Motor[Motor_N].HomePort & (1<<Motor[Motor_N].HomePin))                \
          {                                                                        \
            ones++;                                                                \
          }                                                                        \
        else                                                                       \
          {                                                                        \
            zeros++;                                                               \
          }                                                                        \
        _delay_ms(10);                                                             \
      }                                                                            \
    Motor[Motor_N].Position = 0;                                                   \
    LookupTableRow_t MotorHomeParameters;                                          \
    if (zeros < ones)                                                              \
      {                                                                            \
        MotorHomeParameters[Motor_N].Frequency = HOME_FREQUENCY_SLOW;              \
        MotorHomeParameters[Motor_N].Position = UINT16_MAX;                        \
        EIMSK |= (1<<Motor[Motor_N].HomeEIMSK);                                    \
                                                                                   \
      }                                                                            \
    else                                                                           \
      {                                                                            \
        MotorHomeParameters[Motor_N].Frequency = HOME_FREQUENCY_FAST;              \
        MotorHomeParameters[Motor_N].Position = 12345;                             \
      }                                                                            \
    Motor_Set_Values(MotorHomeParameters,Motor_N);                                 \
    Motor_Update(Motor_N);                                                         \
  }                                                                                \
return                                                                             \


ISR(MOTOR_0_INTERRUPT)
{
  POSITION_UPDATE(0);
}

ISR(MOTOR_1_INTERRUPT)
{
  POSITION_UPDATE(1);
}

ISR(MOTOR_2_INTERRUPT)
{
  POSITION_UPDATE(2);
}

ISR(MOTOR_0_HOME_INTERRUPT)
{
  HOME(0);
}

ISR(MOTOR_1_HOME_INTERRUPT)
{
  HOME(1);
}

ISR(LOOKUP_TABLE_JUMP_INTERRUPT)
{
  /* Set interrupt 4 high to disable interrupt (PORTE pin 4) */
  PORTE |= (1<<PE4);

  if (LookupTablePosMove || LookupTableVelMove)
    {
      /* Toggle InPositionPin high (PORTE pin 5) for testing */
      if (PORTE & (1<<PE5))
        {
          PORTE &= ~(1<<PE5);
        }
      else
        {
          PORTE |= (1<<PE5);
        }

      if (TableEntry < TableEnd)
        {
          TableEntry++;
          Motor_Set_Values_All(LookupTable[TableEntry-1]);
          Motor_Update_All();
        }
      else
        {
          LookupTableMoveInProgress = FALSE;
          LookupTablePosMove = FALSE;
          LookupTableVelMove = FALSE;
          Timer_Off(0);
        }
    }

  return;
}

ISR(LOOKUP_TABLE_VEL_TIMER_INTERRUPT)
{
  if (PINB & (1<<DDB7))
    {
      if (LookupTableVelMove)
        {
          /* Set interrupt 4 low to enable interrupt (PORTE pin 4) */
          PORTE &= ~(1<<PE4);
        }
    }
  return;
}
