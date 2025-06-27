/* 
 * File:   globals.h
 * Author: richa
 *
 * Created on September 24, 2019, 10:40 AM
 */

#ifndef GLOBALS_H
#define	GLOBALS_H

#ifdef	__cplusplus
extern "C" {
#endif


#ifdef	__cplusplus
}
#endif


#include <stdbool.h>

extern uint16_t g_firmwareRev;              // Revision number of firmware

extern uint16_t g_maxPWMInteger;            // maximum PWM integer allowed

// For bare motor testing only - spin a motor axle by outputting fixed PWM signal
extern bool g_motorTestMode;                // true to put in test mode to just spin a motor
extern int16_t g_motorTestPwm;              // for motor test mode only - pwm integer to send to motor

extern unsigned long int g_OscillatorFreq; //Oscillator frequency


enum DIG_STATE {LOW = 0, HIGH = 1}; 
enum LOGICAL {FALSE,TRUE};

extern bool g_outputEnabled;                // true when motion output is enabled, false otherwise.

// PWM parameters
extern int16_t g_pwm1Cycles;               // Signed duty cycle parameter for PWM1.
//extern uint16_t g_maxPWMInteger;           // maximum PWM integer allowed

// User configuration parameters
extern uint16_t g_maxDisplacementMM;         // Maximum peak-to-peak displacement in mm for translational motion. Not used by secondary

// Waveform arrays
extern int32_t* g_outputWaveform;           // pointer to whatever array is currently being used to determine output. It could represent speed or position
extern int32_t* g_zeroWaveform;             // waveform of zeros useful for bringing output to zero gradually

// Waveform Management
//extern volatile bool g_resetWaveform;       // causes waveform array index to be reset to zero.
//extern volatile bool g_stopMotionIssued;    // causes motion to stop the next time a waveform has been completed

// Other waveform properties
extern uint16_t g_numArrayVals;             // number of array values in output waveform
extern uint16_t g_waveformUpdatePeriod;     // number of PWM1 interrupts between waveform index updates
extern uint16_t g_waveformType;             // stores which type of waveform is selected
extern uint16_t g_motionAmplitudeMM;        // amplitude of motion in mm
extern uint16_t g_reverseDirection;         // 1 to move opposite direction, 0 for forward
extern uint32_t g_waveformTimeStep_microS;  // Time between each element of the waveform array in microseconds
extern uint16_t g_freqUser;                 // frequency requested by user in cycles / min

extern uint16_t g_errorFlags;               // Each bit is an error flag. See globals.c
extern bool g_faultDetected;


// Velocity and Position PWM outputs
extern uint16_t g_pwm2ZeroOffset;           // offset for position output. This will be the pwm output for zero speed
extern uint16_t g_pwm3ZeroOffset;           // offset for velocity output. This will be the pwm output for zero speed
extern int16_t g_encoderToPwmDenom;         // divisor for position encoder steps to pwm output for position output pwm (pwm2))

//extern int16_t g_filtNumerator;            // low-pass PWM filter parameter defined as integer numerator and denominator
//extern int16_t g_filtDenominator; 

// Timer1 interrupt period sets the update rate of the feedback loop
extern uint16_t g_feedbackUpdatePeriod;
extern uint16_t g_timer1Prescale;          // Pre-scale factor in Timer1 configuration, needed for other calculations
//extern float g_velReadsPerWfUpdate;        // number of times velocity is read per waveform update period (one index advancement) -- non integer 


// motor control feedback parameters
//extern volatile uint16_t g_velocity;
//extern volatile int16_t g_velDemand;
//extern volatile uint16_t g_tmp;
extern volatile uint16_t g_propConstNum;        // Numerator of proportionality constant for Proportional Feedback component
extern volatile uint16_t g_propConstDenom;      // Denominator of proportionality constant for Proportional Feedback component
extern volatile uint16_t g_intConstNum;         // Numerator of proportionality constant for Integral Feedback component
extern volatile uint16_t g_intConstDenom;       // Denominator of proportionality constant for Integral Feedback component
extern volatile uint16_t g_derivConstNum;       // Numerator of proportionality constant for Derivative Feedback component
extern volatile uint16_t g_derivConstDenom;     // Denominator of proportionality constant for Derivative Feedback component
extern volatile int32_t g_displacementDemand;   // Demand for encoder displacement (relative)
//extern volatile int16_t g_velocityDemand;       // demand requested for motor velocity. Units are number of encoder steps per encoder read period. Negative for reverse direction

// Quadrature Encoder parameters
extern uint32_t g_encoderZeroPos;          // quadrature encoder zero position. This is a short-term reference position
extern uint16_t g_encoderStepsPerMM;       // Number of encoder steps per mm of travel
extern uint32_t g_landmarkPosition;        // Position of Landmark in encoder units

// Commands and other output-related variables
//extern volatile bool g_zeroPosOutput;           // variable to cause zero voltage output when true
//extern volatile bool g_playSingleWaveformOnly;  // set to true to play just one period of the waveform array
extern volatile bool g_startMotor;              // set to true to cause motor to start
extern volatile bool g_gotoLandmark;            // set to true to cause system to go to landmark




#endif	/* GLOBALS_H */

