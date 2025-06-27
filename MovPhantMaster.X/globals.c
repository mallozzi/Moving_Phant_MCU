#include <xc.h>
#include <math.h>
#include <stdbool.h>
#include "globals.h"
#include <stdio.h>

uint16_t g_firmwareRev = 13;             // Revision number of firmware

// *** VARIABLES THAT MUST BE THE SAME IN MASTER AND SLAVE ***
uint16_t g_maxPWMInteger;        // maximum PWM integer allowed
 

// ****  END SECTION

// For bare motor testing only - spin a motor axle by outputting fixed PWM signal
bool g_motorTestMode = false;                // true to put in test mode to just spin a motor
int16_t g_motorTestPwm = 3000;              // for motor test mode only - pwm integer to send to motor




bool g_faultDetected;
uint16_t g_errorFlags;

bool g_outputEnabled = false;           // true when motion output is enabled, false otherwise.

// User configuration parameters
uint16_t g_maxDisplacementMM;           // Maximum peak-to-peak displacement in mm for translational motion

// Waveform arrays
//int32_t* g_outputWaveform = NULL;       // pointer to whatever array is currently being used to determine output. It could represent speed or position
//int32_t* g_zeroWaveform = NULL;         // waveform of zeros useful for bringing output to zero gradually
//volatile bool g_resetWaveform = false;  // causes waveform array index to be reset to zero.
//volatile bool g_stopMotionIssued=false; // causes motion to stop the next time a waveform has been completed

// Waveform parameters
uint16_t g_numArrayVals;                // number of array values in output waveform
uint16_t g_waveformType;                // stores which type of waveform is selected
uint16_t g_motionAmplitudeMM;           // amplitude of motion in mm
uint16_t g_reverseDirection;            // 1 to move opposite direction, 0 for forward

// Other waveform properties
uint32_t g_waveformTimeStep_microS;     // Time between each element of the waveform array in microseconds
uint16_t g_freqUser;                    // frequency requested by user in cycles / min
uint16_t g_waveformUpdatePeriod;        // number of PWM1 interrupts between waveform index updates

//PWM parameters
int16_t g_pwm1Cycles;                   // Duty cycle parameter for PWM1
//int16_t* g_pwmArray=NULL;
              // maximum PWM integer allowed

//int16_t g_filtNumerator;                // low-pass PWM filter parameter defined as integer numerator and denominator
//int16_t g_filtDenominator; 

// PWM parameters for velocity and position outputs
uint16_t g_pwm2ZeroOffset;               // offset for velocity output. This will be the pwm output for zero speed. 
uint16_t g_pwm3ZeroOffset;
int16_t g_encoderToPwmDenom;            // divisor for position encoder steps to pwm output for position output pwm (pwm2))

// Timer1 interrupt period sets the update rate of the feedback loop
uint16_t g_feedbackUpdatePeriod;
uint16_t g_timer1Prescale;              // Pre-scale factor in Timer1 configuration, needed for other calculations
//float g_velReadsPerWfUpdate;            // number of times velocity is read per waveform update period (one index advancement) -- non integer 

// Quadrature Encoder parameters
uint32_t g_encoderZeroPos;              // quadrature encoder zero position
uint16_t g_encoderStepsPerMM;           // Number of encoder steps per mm of travel
uint32_t g_landmarkPosition;            // Position of Landmark in encoder units

// motor control feedback parameters
//volatile uint16_t g_velocity;
//volatile int16_t g_velDemand;
//volatile uint16_t g_tmp;

// Feedback Parameters. The proportional, integral, and derivative constants are expressed as a numerator and a denominator. The ratio
// is the proportionality between that error term in the position and the PWM output component that results.
volatile uint16_t g_propConstNum;       // Numerator of proportionality constant for Proportional Feedback component
volatile uint16_t g_propConstDenom;     // Denominator of proportionality constant for Proportional Feedback component
volatile uint16_t g_intConstNum;        // Numerator of proportionality constant for Integral Feedback component
volatile uint16_t g_intConstDenom;      // Denominator of proportionality constant for Integral Feedback component
volatile uint16_t g_derivConstNum;      // Numerator of proportionality constant for Derivative Feedback component
volatile uint16_t g_derivConstDenom;    // Denominator of proportionality constant for Derivative Feedback component
volatile int32_t g_displacementDemand;  // Demand for encoder displacement (relative)
//volatile int16_t g_velocityDemand;      // demand requested for motor velocity. Units are number of encoder steps per encoder read period. Negative for reverse direction

// Commands and other output-related variables
//volatile bool g_zeroPosOutput;          // variable to cause zero voltage output when true
//volatile bool g_playSingleWaveformOnly; // set to true to play just one period of the waveform array
volatile bool g_startMotor;             // set to true to cause motor to start
volatile bool g_gotoLandmark;           // set to true to cause system to go to landmark

    