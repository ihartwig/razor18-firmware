#include "main.h"
#include "BLDC.h"


/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
  {
    .Config =
      {
        .ControlInterfaceNumber   = INTERFACE_ID_CDC_CCI,
        .DataINEndpoint           =
          {
            .Address          = CDC_TX_EPADDR,
            .Size             = CDC_TXRX_EPSIZE,
            .Banks            = 1,
          },
        .DataOUTEndpoint =
          {
            .Address          = CDC_RX_EPADDR,
            .Size             = CDC_TXRX_EPSIZE,
            .Banks            = 1,
          },
        .NotificationEndpoint =
          {
            .Address          = CDC_NOTIFICATION_EPADDR,
            .Size             = CDC_NOTIFICATION_EPSIZE,
            .Banks            = 1,
          },
      },
  };


// standard file streams for the serial ios
static FILE USBSerialStream;
// static FILE HWSerialStream;

/* Global Data */

//! Array of power stage enable signals for each commutation step.
unsigned char driveTable[6];

//! Array of input channel selections for each commutation step.
unsigned char zcInputTable[6];

//! Current channel not being driven, used for zero-crossing detection.
unsigned char zcInputCurrent;

//! Array holding the intercommutation delays used during startup.
unsigned int startupDelays[STARTUP_NUM_COMMUTATIONS];

/*! \brief Filtered commutation timer variable and speed indicator.
 *  This value equals half the time of one commutation step. It is filtered
 *  through an IIR filter, so the value stored is not the most recent measuremnt.
 *  The variable is stored in registers R14-R15 for quicker access.
 */
volatile unsigned int filteredTimeSinceCommutation;

/*! \brief The power stage enable signals that will be output to the motor drivers
 *  at next commutation.
 *
 *  This variable holds the pattern of enable signals that will be output to the
 *  power stage at next commutation. It is stored in register R13 for quick access.
 */
volatile unsigned char nextDrivePattern;

/*! \brief Polarity of the expected zero crossing.
 *
 *  The polarity of the expected zero crossing.
 *  Could be eiter \ref EDGE_FALLING or \ref EDGE_RISING.
 */
volatile unsigned char zcPolarity;

/*! \brief The commutation step that starts at next commutation.
 *
 *  The commutation step that starts at next commutation. This is used to keep
 *  track on where in the commutation cycle we are. Stored in register R11 for
 *  quick access
 */
volatile unsigned char nextCommutationStep;

//! ADC reading of external analog speed reference.
volatile unsigned char speedReferenceADC;

//! ADC reading of shunt voltage.
volatile unsigned char shuntVoltageADC = 0;

//! ADC reading of the known external reference voltage.
volatile unsigned char referenceVoltageADC;

//! Flag that specifies whether a new external speed reference and a motor speed measurement is available.
volatile unsigned char speedUpdated = FALSE;

//! Flag that specifies whether a new current measurement is available.
volatile unsigned char currentUpdated = FALSE;


/*! \brief Program entry point.
 *
 *  Main initializes all peripheral units used and calls the startup procedure.
 *  All commutation control from that point is done in interrupt routines.
 */
int main (void) {
  InitUSBSerial();

  /* Create a regular character stream for the interface so that it can be used with the stdio.h functions */
  CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);
  // Serial_CreateStream(&HWSerialStream);

  // GlobalInterruptEnable();

  // Initialize all sub-systems.
  ResetHandler();
  InitPorts();
  InitTimers();
  InitADC();
  MakeTables();
  InitAnalogComparator();

  // Run startup procedure.
  StartMotor();

  // Turn on watchdog for stall-detection.
  WatchdogTimerEnable();
  sei(); // __enable_interrupt();

  for(;;)
  {
    PWMControl();

    fprintf(&USBSerialStream, "speedReferenceADC: %4d, nextCommutationStep: %1d\n", speedReferenceADC, nextCommutationStep);

    // call maintenance functions
    CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
    USB_USBTask();
  }

  return 0;
}


void InitUSBSerial(void) {
  /* Disable watchdog if enabled by bootloader/fuses */
  // MCUSR &= ~(1 << WDRF);
  // wdt_disable();

  /* Disable clock division */
  // clock_prescale_set(clock_div_1);

  USB_Init();

  fprintf(&USBSerialStream, "Initialization Complete.\n");
}


/*! \brief Examines the reset source and acts accordingly.
 *
 *  This function is called early in the program to disable watchdog timer and
 *  determine the source of reset.
 *
 *  Actions can be taken, based on the reset source. When the watchdog is used as
 *  stall protection, a stall can be detected here. It is possible to e.g. store
 *  a variable in EEPROM that counts the number of failed restart attempts. After a
 *  certain number of attempts, the motor could simply refuse to continue until
 *  an external action happens, indicating that the rotor lock situation could be
 *  fixed.
 */
static void ResetHandler(void)
{
  /*__eeprom*/ unsigned static int restartAttempts;
  // Temporary variable to avoid unnecessary reading of volatile register MCUSR.
  unsigned char tempMCUSR;

  tempMCUSR = MCUSR;
  MCUSR = tempMCUSR & ~((1 << WDRF) | (1 << BORF) | (1 << EXTRF) | (1 << PORF));

  // Reset watchdog to avoid false stall detection before the motor has started.
  cli(); // __disable_interrupt();
  wdt_reset();
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = 0x00;

  // Examine the reset source and take action.
  switch (tempMCUSR & ((1 << WDRF) | (1 << BORF) | (1 << EXTRF) | (1 << PORF)))
  {
  case (1 << WDRF):
    restartAttempts++;
    if (restartAttempts >= MAX_RESTART_ATTEMPTS)
    {
      // Do something here. E.g. wait for a button to be pressed.
      for (;;)
      {

      }
    }

    // Put watchdog reset handler here.
    break;
  case (1 << BORF):
    //Put brownout reset handler here.
    break;
  case (1 << EXTRF):
    restartAttempts = 0;
    // Put external reset handler here.
    break;
  case (1 << PORF):
    restartAttempts = 0;
    // Put power-on reset handler here.
    break;
  }
}


/*! \brief Initializes I/O ports.
 *
 *  Initializes I/O ports.
 */
static void InitPorts(void)
{
  // Turn on boot pin
  DDRE |= (1 << PE6);
  PORTE |= (1 << PE6);

  // Init DRIVE_DDR for motor driving.
  DRIVE_DDR = (1 << UL) | (1 << UH) | (1 << VL) | (1 << VH) | (1 << WL) | (1 << WH);

  // Init PORTD for PWM (OC0B) on PD0.
  DDRD |= (1 << PD0);

  // Disable digital input buffers on ADC channels.
  DIDR0 = (1 << ADC4D) | (1 << ADC3D) | (1 << ADC2D) | (1 << ADC1D) | (1 << ADC0D);
}


/*! \brief Initializes timers (for commutation timing and PWM).
 *
 *  This function initializes Timer/counter0 for PWM operation for motor speed control
 *  and Timer/counter1 for commutation timing.
 */
static void InitTimers(void)
{
  // Set up Timer/counter0 for PWM, output on OCR0B, OCR0A as TOP value, prescaler = 1.
  TCCR0A = (0 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (0 << COM0B0) | (0 << WGM01) | (1 << WGM00);
  TCCR0B = (1 << WGM02) | (0 << CS02) | (0 << CS01) | (1 << CS00);
  OCR0A = PWM_TOP_VALUE;
  TIFR0 = TIFR0;
  TIMSK0 = (0 << TOIE0);

  // Set up Timer/counter1 for commutation timing, prescaler = 8.
  TCCR1B = (1 << CS11) | (0 << CS10);
}


/*! \brief Initializes the AD-converter.
 *
 *  This function initializes the AD-converter and makes a reading of the external
 *  reference voltage.
 */
static void InitADC(void)
{
  // First make a measurement of the external reference voltage.
  ADMUX = ADMUX_REF_VOLTAGE;
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (0 << ADATE) | (1 << ADIF) | (0 << ADIE) | ADC_PRESCALER_16;
  while (ADCSRA & (1 << ADSC))
  {

  }
  referenceVoltageADC = ADCH;

  // Initialize the ADC for normal operation.
  ADCSRA = (1 << ADEN) | (0 << ADSC) | (0 << ADATE) | (1 << ADIF) | (0 << ADIE) | ADC_PRESCALER_8;
}


/*! \brief Initializes the analog comparator.
 *
 *  This function initializes the analog comparator for overcurrent detection.
 */
static void InitAnalogComparator(void)
{
#ifdef ANALOG_COMPARATOR_ENABLE
  // Enable analog comparator interrupt on rising edge.
  ACSR = (0 << ACBG) | (1 << ACI) | (1 << ACIE) | (1 << ACIS1) | (1 << ACIS0);
#endif
}


/*! \brief Initializes the watchdog timer
 *
 *  This function initializes the watchdog timer for stall restart.
 */
static void WatchdogTimerEnable(void)
{
  cli(); // __disable_interrupt();
  wdt_reset();

  WDTCSR |= (1 << WDCE) | (1 << WDE);

  WDTCSR = (1 << WDIF) | (1 << WDIE) | (1 << WDE) | (1 << WDP2);
  sei(); // __enable_interrupt();
}


/*! \brief Initializes arrays for motor driving and AD channel selection.
 *
 *  This function initializes the arrays used for motor driving and AD channel
 *  selection that changes for each commutation step.
 */
static void MakeTables(void)
{
#if DIRECTION_OF_ROTATION == CCW
  driveTable[0] = DRIVE_PATTERN_STEP1_CCW;
  driveTable[1] = DRIVE_PATTERN_STEP2_CCW;
  driveTable[2] = DRIVE_PATTERN_STEP3_CCW;
  driveTable[3] = DRIVE_PATTERN_STEP4_CCW;
  driveTable[4] = DRIVE_PATTERN_STEP5_CCW;
  driveTable[5] = DRIVE_PATTERN_STEP6_CCW;
#else
  driveTable[0] = DRIVE_PATTERN_STEP1_CW;
  driveTable[1] = DRIVE_PATTERN_STEP2_CW;
  driveTable[2] = DRIVE_PATTERN_STEP3_CW;
  driveTable[3] = DRIVE_PATTERN_STEP4_CW;
  driveTable[4] = DRIVE_PATTERN_STEP5_CW;
  driveTable[5] = DRIVE_PATTERN_STEP6_CW;
#endif

  zcInputTable[0] = ZC_COMP_W;
  zcInputTable[1] = ZC_COMP_V;
  zcInputTable[2] = ZC_COMP_U;
  zcInputTable[3] = ZC_COMP_W;
  zcInputTable[4] = ZC_COMP_V;
  zcInputTable[5] = ZC_COMP_U;

  startupDelays[0] = 200;
  startupDelays[1] = 150;
  startupDelays[2] = 100;
  startupDelays[3] = 80;
  startupDelays[4] = 70;
  startupDelays[5] = 65;
  startupDelays[6] = 60;
  startupDelays[7] = 55;
}


/*! \brief Executes the motor startup sequence.
 *
 *  This function locks the motor into a known position and fires off a
 *  commutation sequence controlled by the Timer/counter1 overflow interrupt.
 */
static void StartMotor(void)
{
  unsigned char i;

  SET_PWM_COMPARE_VALUE(STARTUP_PWM_COMPARE_VALUE);

  nextCommutationStep = 0;

  //Preposition.
  StartupDelay(STARTUP_PRE_DELAY);
  DRIVE_PORT = driveTable[nextCommutationStep];
  StartupDelay(STARTUP_LOCK_DELAY);
  zcInputCurrent = zcInputTable[nextCommutationStep];
  nextCommutationStep++;
  nextDrivePattern = driveTable[nextCommutationStep];

  for (i = 0; i < STARTUP_NUM_COMMUTATIONS; i++)
  {
    DRIVE_PORT = nextDrivePattern;
    StartupDelay(startupDelays[i]);

    zcInputCurrent = zcInputTable[nextCommutationStep];

    // Use LSB of nextCommutationStep to determine zero crossing polarity.
    zcPolarity = nextCommutationStep & 0x01;

    nextCommutationStep++;
    if (nextCommutationStep >= 6)
    {
      nextCommutationStep = 0;
    }
    nextDrivePattern = driveTable[nextCommutationStep];
  }

  // Switch to sensorless commutation.
  TCNT1 = 0;
  TIMSK1 = (1 << OCIE1A);

  // Set filteredTimeSinceCommutation to the time to the next commutation.
  filteredTimeSinceCommutation = startupDelays[STARTUP_NUM_COMMUTATIONS - 1] * (STARTUP_DELAY_MULTIPLIER  / 2);
}

//! \brief Waits for pending ADC operations to finish
static inline void wait_adc() {
  while((ADCSRA & (1 << ADSC)))
  {
  }
}

/*! \brief Starts the ADC then waits for and returns the measurement
 *
 * Set up ADMUX before calling.
 */
static inline char run_adc() {
  ADCSRA |= (1 << ADSC);
  wait_adc();
  return ADCH;
}


/*! \brief Timer/counter0 bottom overflow. Used for zero-cross detection.
 *
 *  This interrupt service routine is called every time the up/down counting
 *  PWM counter reaches bottom. An ADC reading on the active channel is
 *  automatically triggered at the same time as this interrupt is triggered.
 *  This is used to detect a zero crossing.
 *
 *  In the event of a zero crossing, the time since last commutation is stored
 *  and Timer/counter1 compare A is set up to trigger at the next commutation
 *  instant.
 */
// #pragma vector=TIMER0_OVF_vect
// __interrupt void MotorPWMBottom()
ISR(TIMER0_OVF_vect)
{
  unsigned char isAboveZero;

  isAboveZero = !(ZC_COMP_PIN & _BV(zcInputCurrent));
  if ((zcPolarity == EDGE_RISING && isAboveZero) || (zcPolarity == EDGE_FALLING && !isAboveZero))
  {
    unsigned int timeSinceCommutation;

    // Find time since last commutation
    timeSinceCommutation = TCNT1;
    TCNT1 = COMMUTATION_CORRECTION;

    // Filter the current ZC detection with earlier measurements through an IIR filter.
    filteredTimeSinceCommutation = (COMMUTATION_TIMING_IIR_COEFF_A * timeSinceCommutation
                                + COMMUTATION_TIMING_IIR_COEFF_B * filteredTimeSinceCommutation)
                                / (COMMUTATION_TIMING_IIR_COEFF_A + COMMUTATION_TIMING_IIR_COEFF_B);
    OCR1A = filteredTimeSinceCommutation;

    speedUpdated = TRUE;

    SET_TIMER1_INT_COMMUTATION;
    CLEAR_ALL_TIMER1_INT_FLAGS;

    // Disable Timer/Counter0 overflow ISR.
    DISABLE_ALL_TIMER0_INTS;

    // Make sure that a sample is not in progress.
    wait_adc();

    // Read speed reference.
    ADMUX = ADMUX_SPEED_REF;
    speedReferenceADC = run_adc();

    // Read voltage reference.
    ADMUX = ADMUX_REF_VOLTAGE;
    referenceVoltageADC = run_adc();
  }
  else
  {
    // Make sure that a sample is not in progress
    wait_adc();
  }

  // Read current.
  ADMUX = ADMUX_CURRENT;
  shuntVoltageADC = run_adc();
  currentUpdated = TRUE;
}


/*! \brief Commutates and prepares for new zero-cross detection.
 *
 *  This interrupt service routine is triggered exactly when a commutation
 *  is scheduled. The commutation is performed instantly and Timer/counter0
 *  is reset to measure the delay between commutation and zero-cross detection.
 *
 *  Commutation causes large transients on all phases for a short while that could
 *  cause false zero-cross detections. A zero cross detection hold-off period is
 *  therefore used to avoid any false readings. This is performed by using Timer/counter1
 *  compare B. The compare is set to happen after the specified hold-off period.
 *  Timer/counter1 compare B interrupt handler then enables the zero-cross detection.
 */
// #pragma vector=TIMER1_COMPA_vect
// __interrupt void Commutate()
ISR(TIMER1_COMPA_vect)
{
  // Commutate and clear commutation timer.
  DRIVE_PORT = nextDrivePattern;
  TCNT1 = 0;

  zcPolarity = nextCommutationStep & 0x01;

  // Set zero-cross detection holdoff time.
  CLEAR_ALL_TIMER1_INT_FLAGS;
  OCR1B = ZC_DETECTION_HOLDOFF_TIME_US;
  SET_TIMER1_INT_HOLDOFF;

  wdt_reset(); // __watchdog_reset();
}


/*! \brief Enables zero-cross detection.
 *
 *  This interrupt service routine is triggered when the zero cross detection
 *  hold-off time after commutation is over. All Timer/counter1 interrupts are
 *  disabled and Timer/counter0 (PWM) overflow interrupt is enabled to allow
 *  the ADC readings to be used for zero-cross detection.
 */
// #pragma vector=TIMER1_COMPB_vect
// __interrupt void EnableZCDetection()
ISR(TIMER1_COMPB_vect)
{
  // Enable TCNT0 overflow ISR.
  CLEAR_ALL_TIMER0_INT_FLAGS;
  CLEAR_ALL_TIMER1_INT_FLAGS;
  SET_TIMER0_INT_ZC_DETECTION;
  DISABLE_ALL_TIMER1_INTS;

  // Move to next input for zero-cross detection
  zcInputCurrent = zcInputTable[nextCommutationStep];

  // Rotate commutation step counter.
  nextCommutationStep++;
  if (nextCommutationStep >= 6)
  {
    nextCommutationStep = 0;
  }
  nextDrivePattern = driveTable[nextCommutationStep];
}


/*! \brief Watchdog interrupt
 *
 *  This ISR is called before the watchdog timer resets the device because of a stall.
 *  It simply disables driving, but other tasks that must be done before a watchdog reset,
 *  such as storage of variables in non-volatile memory can be done here.
 */
// #pragma vector=WDT_vect
// __interrupt void WatchdogISR()
ISR(WDT_vect)
{
  DISABLE_DRIVING;
  for(;;)
  {
    ;
  }
}

/*! \brief Overcurrent interrupt
 *
 *  This interrupt service routine cuts power to the motor when an overcurrent situation
 *  is detected.
 */
#ifdef ANALOG_COMPARATOR_ENABLE
// #pragma vector=ANA_COMP_vect
// __interrupt void OverCurrentISR()
ISR(ANA_COMP_vect)
{
  DISABLE_DRIVING;
  for(;;)
  {
    ;
  }
}
#endif


/*! \brief Generates a delay used during startup
 *
 *  This functions is used to generate a delay during the startup procedure.
 *  The length of the delay equals delay * STARTUP_DELAY_MULTIPLIER microseconds.
 *  Since Timer/Counter1 is used in this function, it must never be called when
 *  sensorless operation is running.
 */
void StartupDelay(unsigned int delay)
{
  CLEAR_ALL_TIMER1_INT_FLAGS;
  do
  {
    TCNT1 = 0xffff - STARTUP_DELAY_MULTIPLIER;
    // Wait for timer to overflow.
    while (!(TIFR1 & (1 << TOV1)))
    {

    }

    CLEAR_ALL_TIMER1_INT_FLAGS;
    delay--;
  } while (delay);
}



#ifdef SPEED_CONTROL_CLOSED_LOOP
/*! \brief Controls the PWM duty cycle based on speed set-point and current consumption.
 *
 *  This function controls the PWM duty cycle by calling a speed controller and a
 *  current controller. The speed controller signal is directly applied to the duty
 *  cycle. The current controller signal is used to limit the maximum duty cycle.
 */
static void PWMControl(void)
{
  signed int speedCompensation;
  static unsigned char currentCompensation = 0;
  static signed int duty = STARTUP_PWM_COMPARE_VALUE;

  // Run speed control only if a new speed measurement is available.
 if (speedUpdated)
  {
    speedCompensation = SpeedControl();
    speedUpdated = FALSE;
    duty += speedCompensation;
  }

  // Run current control only if a new current measurement is available.
  if (currentUpdated)
  {
     currentCompensation = CurrentControl();
     currentUpdated = FALSE;
  }

 // Keep duty cycle within limits.
  if (duty < MIN_PWM_COMPARE_VALUE)
  {
    duty = MIN_PWM_COMPARE_VALUE;
  }
  if (duty > (MAX_PWM_COMPARE_VALUE - currentCompensation))
  {
    duty = MAX_PWM_COMPARE_VALUE - currentCompensation;
  }

  SET_PWM_COMPARE_VALUE((unsigned char)duty);
}
#endif

#ifdef SPEED_CONTROL_OPEN_LOOP
static void PWMControl(void)
{
  // Only update duty cycle if a new speed reference measurement has been made. (Done right after speed measurement is ready)
  if (speedUpdated)
  {
    // Calculate duty cycle from speed reference value.
    SET_PWM_COMPARE_VALUE(MIN_PWM_COMPARE_VALUE + speedReferenceADC * (MAX_PWM_COMPARE_VALUE - MIN_PWM_COMPARE_VALUE) / ADC_RESOLUTION);
  }
}
#endif


/*! \brief Calculates the current speed in electrical RPM.
 *
 *  This function calculates the current speed in electrical rotations per
 *  minute from the global variable \ref filteredTimeSinceCommutation.
 */
static unsigned long CalculateSpeed()
{
  // Copy used to minimize period where interrupts are disabled.
  unsigned int filteredTimeSinceCommutationCopy;
  unsigned long rotationPeriod;
  unsigned long speed;

  /*
  Disable interrupts to ensure that \ref filteredTimeSinceCommutation is accessed in
  an atomic operation.
  */
  cli(); // __disable_interrupt();
  filteredTimeSinceCommutationCopy = filteredTimeSinceCommutation;
  sei(); // __enable_interrupt();

  /*
  filteredTimeSinceCommutation is one half commutation time. Must be multiplied by 12 to get
  one full rotation.
  */
  rotationPeriod = (unsigned long)filteredTimeSinceCommutationCopy * 12;
  speed = (TICKS_PER_MINUTE / rotationPeriod);

  return speed;
}


/*! \brief Calculates the speed set-point in electrical RPM.
 *
 *  This function calculates the speed set-point from the global variable
 *  speedReferenceADC.
 *
 *  In this implementation, the speed reference values from 0x00 to 0xff are
 *  linearly mapped into the allowable speed range, set by \ref MIN_SPEED and
 *  \ref MAX_SPEED.
 */
static unsigned long CalculateSpeedSetpoint()
{
  return (MIN_SPEED + ((MAX_SPEED - MIN_SPEED) * (unsigned int)speedReferenceADC) / ADC_RESOLUTION);
}


/*! \brief Calculates current consumption.
 *
 *  This function calculates the current consumption in milliAmperes from the
 *  global variable \ref shuntVoltageADC. The external know reference voltage
 *  is used to compensate for varying AREF voltage.
 */
static unsigned int CalculateCurrent()
{
  unsigned long ADCref;
  unsigned int current;

  // Calculate the voltage at AREF pin (scaled down motor supply voltage),
  // using the known reference voltage. (In milliVolts)
  ADCref = EXTERNAL_REF_VOLTAGE * 256UL / referenceVoltageADC;

  // Calculate the current through the shunt. (In milliAmperes)
  current = (unsigned int)((shuntVoltageADC * ADCref * 1000UL / 256UL) / SHUNT_RESISTANCE);

  return current;
}


/*! \brief Speed control loop
 *
 *  This function runs a simple P-regulator speed control loop. The duty cycle
 *  is only updated each time a new speed measurement is ready (on each zero-crossing).
 *  The time step is thus variable, so the P-gain of the P-regulator corresponds to
 *  a speed-varying P-gain for the continous system.
 */
static signed int SpeedControl(void)
{
  unsigned long speedSetpoint;
  unsigned long currentSpeed;
  signed long speedError;
  signed long dutyChange;



  speedSetpoint = CalculateSpeedSetpoint();
  currentSpeed = CalculateSpeed();
  speedError = (speedSetpoint - currentSpeed);
  dutyChange = speedError * P_REG_K_P / P_REG_SCALING;

  return dutyChange;
}


/*! \brief Current control loop
 *
 *  This function is called after the speed control loop. The desired duty cycle
 *  calculated by the speed control loop is available, and this function is
 *  responsible for adjusting the duty cycle to ensure that the current stays
 *  within limits.
 */
static unsigned char CurrentControl(void)
{
  unsigned int current;
  unsigned int overCurrentCorrection = 0;

  current = CalculateCurrent();

  // Cut power to motor if current is critically high.
  if (current > CURRENT_LIMITER_CRITICAL)
  {
    DRIVE_PORT = 0x00;
    for (;;)
    {
      // Stop and let watchdog timer reset part.
    }
  }

  if (current > CURRENT_LIMITER_START)
  {
    overCurrentCorrection = (current - CURRENT_LIMITER_START) * CURRENT_LIMITER_FACTOR;
  }

  if (overCurrentCorrection > 255)
  {
    return 255;
  }

  return overCurrentCorrection;
}


/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
  // LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}


/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
  // LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}


/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
  bool ConfigSuccess = true;

  ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

  // LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}


/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
  CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}
