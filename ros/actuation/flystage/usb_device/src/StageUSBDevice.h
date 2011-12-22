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
 *  Header file for StageUSBDevice.c.
 */

#ifndef _STAGEUSBDEVICE_H_
#define _STAGEUSBDEVICE_H_

/* Includes: */
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdbool.h>
#include <string.h>

#include "Descriptors.h"

#include <LUFA/Version.h>                    // Library Version Information
#include <LUFA/Scheduler/Scheduler.h>        // Simple scheduler for task management
#include <LUFA/Drivers/USB/USB.h>            // USB Functionality
#include <LUFA/Drivers/Board/LEDs.h>         // LEDs driver

/* Macros: */
#define TRUE  1
#define FALSE 0
#define UINT16_MIN 0
#define UINT16_MAX 65535
/* USB Commands */
#define USB_CMD_GET_STATE                1
#define USB_CMD_SET_STATE                2
#define USB_CMD_HOME                     3
#define USB_CMD_LOOKUP_TABLE_FILL        4
#define USB_CMD_LOOKUP_TABLE_POS_MOVE    5
#define USB_CMD_LOOKUP_TABLE_VEL_MOVE    6
#define USB_CMD_AVR_RESET                200
#define USB_CMD_AVR_DFU_MODE             201

/* Motor Parameter Values */
#define MOTOR_0 0
#define MOTOR_1 1
#define MOTOR_2 2
#define MOTOR_0_POSITION_HOME 0
#define MOTOR_1_POSITION_HOME 0
#define MOTOR_2_POSITION_HOME 0
#define MOTOR_0_FREQUENCY_MAX 50000
#define MOTOR_1_FREQUENCY_MAX 50000
#define MOTOR_2_FREQUENCY_MAX 50000
#define MOTOR_0_DIRECTION_POS 0
#define MOTOR_0_DIRECTION_NEG 1
#define MOTOR_1_DIRECTION_POS 0
#define MOTOR_1_DIRECTION_NEG 1
#define MOTOR_2_DIRECTION_POS 0
#define MOTOR_2_DIRECTION_NEG 1
#define MOTOR_0_TIMER         1
#define MOTOR_1_TIMER         3
#define MOTOR_2_TIMER         2
#define MOTOR_0_INTERRUPT     TIMER1_OVF_vect
#define MOTOR_1_INTERRUPT     TIMER3_OVF_vect
#define MOTOR_2_INTERRUPT     TIMER2_OVF_vect

#define MOTOR_0_HOME_INTERRUPT     INT0_vect
#define MOTOR_1_HOME_INTERRUPT     INT1_vect
#define MOTOR_2_HOME_INTERRUPT     INT2_vect

#define LOOKUP_TABLE_JUMP_INTERRUPT  INT4_vect
#define LOOKUP_TABLE_VEL_TIMER_INTERRUPT  TIMER0_OVF_vect

#define HOME_FREQUENCY_FAST 10000
#define HOME_FREQUENCY_SLOW 100

/* Software reset */
#define AVR_RESET() wdt_enable(WDTO_30MS); while(TRUE) {}
#define AVR_IS_WDT_RESET()  ((MCUSR&(1<<WDRF)) ? TRUE:FALSE)
#define DFU_BOOT_KEY_VAL 0xAA55AA55
#define MOTOR_NUM 3
#define TIMER_NUM 4
#define PRESCALER_NUM 5
#define LOOKUP_TABLE_SIZE 100 // # values in lookup table
#define ENTRIES_MAX 5

/* Type Defines: */
typedef struct
{
  uint8_t     Timer;
  volatile uint8_t     *DirectionPort;
  uint8_t     DirectionPin;
  volatile uint8_t     *HomePort;
  uint8_t     HomePin;
  uint8_t     HomeEIMSK;
  uint8_t     HomeEIFR;
  uint16_t    Frequency;
  uint16_t    FrequencyMax;
  uint8_t     Direction;
  uint8_t     DirectionPos;
  uint8_t     DirectionNeg;
  uint16_t    Position;
  uint16_t    PositionSetPoint;
  uint8_t     Update;
  uint8_t     InPosition;
  uint8_t     HomeInProgress;
  uint8_t     HomeSet;
  uint16_t    PositionLimitMax;
  uint16_t    PositionLimitMin;
  uint8_t     PositionLimitsEnabled;
} MotorWrapper_t;

typedef struct
{
  struct
  {
    volatile uint16_t    *TOP;
    volatile uint8_t     *ClockSelect;
    volatile uint8_t     *PinPort;
  } Address;
  uint8_t     OutputPin;
  uint8_t     ClockSelect[PRESCALER_NUM + 1];
  uint8_t     Prescaler_N;
  uint8_t     ScaleFactor;
  uint16_t    TOPValue;
  uint16_t    TOPMax;
  uint8_t     On;
} TimerWrapper_t;

typedef struct
{
  uint16_t   Frequency;
  uint16_t   Position;
} MotorStatus_t;

typedef MotorStatus_t LookupTableRow_t[MOTOR_NUM];

typedef struct
{
  uint8_t       CommandID;
  uint8_t       MotorUpdate;
  uint8_t       EntryCount;
  uint8_t       EntryLocation;
  LookupTableRow_t Setpoint[ENTRIES_MAX];
} USBPacketOutWrapper_t;

typedef struct
{
  uint8_t       CommandID;
  uint8_t       AllMotorsInPosition;
  uint8_t       LookupTableMoveInProgress;
  uint8_t       HomeInProgress;
  uint8_t       MotorsHomed;
  LookupTableRow_t MotorStatus;
} USBPacketInWrapper_t;

/* Enums: */
/** Enum for the possible status codes for passing to the UpdateStatus() function. */
enum USB_StatusCodes_t
  {
    Status_USBNotReady      = 0, /**< USB is not ready (disconnected from a USB host) */
    Status_USBEnumerating   = 1, /**< USB interface is enumerating */
    Status_USBReady         = 2, /**< USB interface is connected and ready */
    Status_ProcessingPacket = 3, /**< Processing packet */
  };

/* Global Variables: */
const  uint16_t         PrescalerArray16[PRESCALER_NUM] = {1, 8, 64, 256, 1024};
/* Probably a mistake with using the PrescalerArray8!!! */
const  uint16_t         PrescalerArray8[PRESCALER_NUM] = {1, 8, 32, 64, 128};
volatile MotorWrapper_t Motor[MOTOR_NUM];
volatile TimerWrapper_t Timer[TIMER_NUM];
USBPacketOutWrapper_t   USBPacketOut;
USBPacketInWrapper_t    USBPacketIn;
uint8_t                 IO_Enabled=FALSE;
uint8_t                 Interrupt_Enabled=FALSE;
LookupTableRow_t        LookupTable[LOOKUP_TABLE_SIZE];
uint8_t                 TableEnd=0;
uint8_t                 TableEntry=0;
uint8_t                 LookupTablePosMove=FALSE;
uint8_t                 LookupTableVelMove=FALSE;
uint8_t                 MotorUpdateBits=0;
volatile uint8_t        AllMotorsInPosition=FALSE;
uint8_t                 LookupTableMoveInProgress=FALSE;
uint8_t                 HomeInProgress=FALSE;
uint8_t                 MotorsHomed=FALSE;

/* Task Definitions: */
TASK(USB_ProcessPacket);

/* Function Prototypes: */
void EVENT_USB_Connect(void);
void EVENT_USB_Disconnect(void);
void EVENT_USB_ConfigurationChanged(void);
void EVENT_USB_UnhandledControlPacket(void);

void UpdateStatus(uint8_t CurrentStatus);

#if defined(INCLUDE_FROM_STAGEUSBDEVICE_C)
static void USBPacket_Read(void);
static void USBPacket_Write(void);
static void IO_Init(void);
static void IO_Disconnect(void);
static void Interrupt_Init(void);
static void Timer_Init(void);
static void Timer_On(uint8_t Timer_N);
static void Timer_Off(uint8_t Timer_N);
static void Motor_Init(void);
static void Motor_Update(uint8_t Motor_N);
static void Motor_Update_All(void);
static void Motor_Set_Values(LookupTableRow_t MotorSetpoint,uint8_t Motor_N);
static void Motor_Set_Values_All(LookupTableRow_t MotorSetpoint);
static void Motor_Home(void);
static void Motor_Check_In_Position(void);
static void Motor_Check_Home(void);
static void Lookup_Table_Fill(LookupTableRow_t *LookupTableEntries,uint8_t EntryCount,uint8_t EntryLocation);
static void Position_Update(volatile uint8_t Motor_N);
static void Write_Return_USBPacket(void);
#endif

#endif
