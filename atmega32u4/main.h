#ifndef _MAIN_H_
#define _MAIN_H_


/* Includes: */
#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>

#include "descriptors.h"

#include <LUFA/Drivers/Peripheral/Serial.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>


/* HW Setup */


/* Data Structures */


/* Function Prototypes: */
static void InitUSBSerial(void);

static void ResetHandler(void);
static void InitPorts(void);
static void InitTimers(void);
static void InitADC(void);
static void InitAnalogComparator(void);
static void WatchdogTimerEnable(void);
static void MakeTables(void);
static void StartMotor(void);
static void PWMControl(void);
static void StartupDelay(unsigned int delay);
static unsigned long CalculateSpeed(void);
static unsigned long CalculateSpeedSetpoint(void);
static unsigned int CalculateCurrent(void);
static signed int SpeedControl(void);
static unsigned char CurrentControl(void);

void EVENT_USB_Device_Connect(void);
void EVENT_USB_Device_Disconnect(void);
void EVENT_USB_Device_ConfigurationChanged(void);
void EVENT_USB_Device_ControlRequest(void);


#endif
