#include <xc.h>
#include <math.h>
#include <stdbool.h>
#include "globals.h"
#include <stdio.h>

uint16_t g_firmwareRev = 13;            // Revision number of firmware
uint16_t g_statusFlags = 0;             // flags containing status information

// *** VARIABLES THAT MUST BE THE SAME IN MASTER AND SLAVE ***
uint16_t g_maxPWMInteger;        // maximum PWM integer allowed
 

// ****  END SECTION

// For bare motor testing only - spin a motor axle by outputting fixed PWM signal
bool g_motorTestMode = false;           // true to put in test mode to just spin a motor
int16_t g_motorTestPwm = 3000;          // for motor test mode only - pwm integer to send to motor

uint16_t g_pingVar;                     // used to confirm round-trip FIFO ping to secondary core
uint32_t g_secondaryQuadEncPos;         // quadrature encoder position read from secondary core
int16_t g_secondaryQuadEncVel;          // quadrature encoder velocity read from secondary core


bool g_faultDetected;
uint16_t g_errorFlags;

bool g_output1Enabled = false;          // true when motion output is enabled, false otherwise.
bool g_output2Enabled = false;          // true when motion output is enabled, false otherwise.
bool g_userMotor1Enable = false;        // true when the user requests motor 1 to be enabled on the UI
bool g_userMotor2Enable = false;        // true when the user requests motor 2 to be enabled on the UI

// User configuration parameters
uint16_t g_maxDisplacementMM;           // Defines the displacement that corresponds to full scale voltage output
                                        // ...in analog output signal

// Waveform parameters
uint16_t g_numArrayVals;                // number of array values in output waveform
uint16_t g_waveformType;                // stores which type of waveform is selected
uint16_t g_reverseDirection1;            // 1 to move opposite direction, 0 for forward motor 1
uint16_t g_reverseDirection2;            // 1 to move opposite direction, 0 for forward motor 2
uint16_t g_motionAmplitudeMM1;          // amplitude of motion in mm motor 1
uint16_t g_motionAmplitudeMM2;          // amplitude of motion in mm motor 2

// Other waveform properties
uint32_t g_waveformTimeStep_microS;     // Time between each element of the waveform array in microseconds
uint16_t g_freqUser;                    // frequency requested by user in cycles / min
uint16_t g_waveformUpdatePeriod;        // number of PWM1 interrupts between waveform index updates

//PWM parameters
int16_t g_pwm1Cycles;                   // Duty cycle parameter for PWM1
int16_t g_pwm2Cycles;                   // Duty cycle parameter for PWM1

// PWM parameters for velocity and position outputs
uint16_t g_pwm1ZeroOffset;               // offset for motor 1 position output. This will be the pwm output for zero position
uint16_t g_pwm2ZeroOffset;               // offset for velocity output. This will be the pwm output for zero speed. 
uint16_t g_pwm3ZeroOffset;
int16_t g_encoderToPwmDenom;             // divisor for position encoder steps to pwm output for analog position output pwm (pwm2))

// Timer1 interrupt period sets the update rate of the feedback loop
uint16_t g_feedbackHalfUpdatePeriod;    // Timer1 interrupt period in units of Timer1 cycles. Feedback output for each motor is updated every two of these periods
uint16_t g_timer1Prescale;              // Pre-scale factor in Timer1 configuration, needed for other calculations

// Quadrature Encoder parameters
uint32_t g_encoder1ZeroPos;              // quadrature encoder zero position
uint32_t g_encoder2ZeroPos;              // quadrature encoder zero position
uint16_t g_encoderStepsPerMM;            // Number of encoder steps per mm of travel
uint32_t g_landmark1Position;            // Position of Landmark for motor 1 in encoder units
uint32_t g_landmark2Position;            // Position of Landmark for motor 2 in encoder units


// Feedback Parameters for motor 1. The proportional, integral, and derivative constants are expressed as a numerator and a denominator. 
// The ratio is the proportionality between that error term in the position and the PWM output component that results.
volatile uint16_t g_propConstNum1;       // Numerator of proportionality constant for Proportional Feedback component
volatile uint16_t g_propConstDenom1;     // Denominator of proportionality constant for Proportional Feedback component
volatile uint16_t g_intConstNum1;        // Numerator of proportionality constant for Integral Feedback component
volatile uint16_t g_intConstDenom1;      // Denominator of proportionality constant for Integral Feedback component
volatile uint16_t g_derivConstNum1;      // Numerator of proportionality constant for Derivative Feedback component
volatile uint16_t g_derivConstDenom1;    // Denominator of proportionality constant for Derivative Feedback component
volatile int32_t g_displacement1Demand;  // Demand for encoder displacement (relative)

// Feedback Parameters for motor 2. The proportional, integral, and derivative constants are expressed as a numerator and a denominator. 
// The ratio is the proportionality between that error term in the position and the PWM output component that results.
// TO DO: Make all of these for motor 2
volatile uint16_t g_propConstNum2;       // Numerator of proportionality constant for Proportional Feedback component
volatile uint16_t g_propConstDenom2;     // Denominator of proportionality constant for Proportional Feedback component
volatile uint16_t g_intConstNum2;        // Numerator of proportionality constant for Integral Feedback component
volatile uint16_t g_intConstDenom2;      // Denominator of proportionality constant for Integral Feedback component
volatile uint16_t g_derivConstNum2;      // Numerator of proportionality constant for Derivative Feedback component
volatile uint16_t g_derivConstDenom2;    // Denominator of proportionality constant for Derivative Feedback component
volatile int32_t g_displacement2Demand;  // Demand for encoder displacement (relative)


// Commands and other output-related variables
volatile bool g_startMotor;             // set to true to cause motor to start
volatile bool g_gotoLandmark;           // set to true to cause system to go to landmark

    