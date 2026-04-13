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

extern unsigned long int g_OscillatorFreq;  //Oscillator frequency
extern uint16_t g_firmwareRev;              // Revision number of firmware
extern volatile uint16_t g_statusFlags;     // flags containing status information
extern bool g_stepMode;                     // true if user is stepping rather than running waveform
extern bool g_landmarkMode;                 // true if user requested to go to landmark
extern bool g_stopButtonPushed;             // used to distinguish user stop request from proximity sensor stop request

extern uint16_t g_maxPWMInteger;            // maximum PWM integer allowed

// For bare motor testing only - spin a motor axle by outputting fixed PWM signal
extern bool g_motorTestMode;                // true to put in test mode to just spin a motor
extern int16_t g_motorTestPwm;              // for motor test mode only - pwm integer to send to motor

extern uint16_t g_pingVar;                  // used to confirm round-trip FIFO ping to secondary core
extern uint32_t g_secondaryQuadEncPos;      // quadrature encoder position read from secondary core
extern int16_t g_secondaryQuadEncVel;       // quadrature encoder velocity read from secondary core


enum DIG_STATE {LOW = 0, HIGH = 1}; 
enum LOGICAL {FALSE,TRUE};

extern bool g_output1Enabled;               // true when motion output is enabled, false otherwise.
extern bool g_output2Enabled;               // true when motion output is enabled, false otherwise.
extern bool g_userMotor1Enable;             // true when the user requests motor 1 to be enabled on the UI
extern bool g_userMotor2Enable;             // true when the user requests motor 2 to be enabled on the UI

// PWM parameters
extern int16_t g_pwm1Cycles;                // Signed duty cycle parameter for PWM1.
extern int16_t g_pwm2Cycles;                // Signed duty cycle parameter for PWM1.

// User configuration parameters
extern uint16_t g_maxDisplacementMM;        // Defines the displacement that corresponds to full scale voltage output
                                            // ...in analog output signal

// Waveform arrays
//extern int32_t* g_outputWaveform;           // pointer to whatever array is currently being used to determine output. It could represent speed or position
extern int32_t* g_zeroWaveform;             // waveform of zeros useful for bringing output to zero gradually


// Other waveform properties
extern uint16_t g_numArrayVals;                 // number of array values in output waveform
extern uint16_t g_waveformUpdatePeriod;         // number of PWM1 interrupts between waveform index updates
extern uint16_t g_waveformType;                 // stores which type of waveform is selected
extern uint16_t g_reverseDirection1;            // 1 to move opposite direction, 0 for forward motor 1
extern uint16_t g_reverseDirection2;            // 1 to move opposite direction, 0 for forward motor 2
extern uint32_t g_waveformTimeStep_microS;      // Time between each element of the waveform array in microseconds
extern uint16_t g_freqUser;                     // frequency requested by user in cycles / min
extern uint16_t g_motionAmplitudeMM1;           // amplitude of motion in mm motor 1
extern uint16_t g_motionAmplitudeMM2;           // amplitude of motion in mm motor 2
extern uint16_t g_whichWaveform;                // flag that is 0 for Motor 1, 1 for Motor 2 used for transmitting custom waveform data between cores

extern uint16_t g_errorFlags;                   // Each bit is an error flag. See globals.c
extern bool g_faultDetected;


// Velocity and Position PWM outputs
extern uint16_t g_pwm1ZeroOffset;               // offset for motor 1 position output. This will be the pwm output for zero position
extern uint16_t g_pwm2ZeroOffset;               // offset for motor 2 position output. This will be the pwm output for zero speed
extern uint16_t g_pwm3ZeroOffset;               // offset for velocity output. This will be the pwm output for zero speed
extern int16_t g_encoderToPwmDenom_1;           // divisor for position encoder steps to pwm output for analog position output pwm (pwm2) for motor 1)
extern int16_t g_encoderToPwmDenom_2;           // divisor for position encoder steps to pwm output for analog position output pwm (pwm2) for motor 2)

// Timer1 interrupt period sets the update rate of the feedback loop
extern uint16_t g_feedbackHalfUpdatePeriod;     // Timer1 interrupt period in units of Timer1 cycles. Feedback output for each motor is updated every two of these periods
extern uint16_t g_timer1Prescale;               // Pre-scale factor in Timer1 configuration, needed for other calculations
//extern float g_velReadsPerWfUpdate;           // number of times velocity is read per waveform update period (one index advancement) -- non integer 


// motor 1 control feedback parameters
extern volatile uint16_t g_propConstNum1;        // Numerator of proportionality constant for Proportional Feedback component
extern volatile uint16_t g_propConstDenom1;      // Denominator of proportionality constant for Proportional Feedback component
extern volatile uint16_t g_intConstNum1;         // Numerator of proportionality constant for Integral Feedback component
extern volatile uint16_t g_intConstDenom1;       // Denominator of proportionality constant for Integral Feedback component
extern volatile uint16_t g_derivConstNum1;       // Numerator of proportionality constant for Derivative Feedback component
extern volatile uint16_t g_derivConstDenom1;     // Denominator of proportionality constant for Derivative Feedback component
extern volatile int32_t g_displacement1Demand;   // Demand for encoder displacement (relative)

// motor 2 control feedback parameters
// TO DO: MAKE THESE ALL FOR MOTOR 2
extern volatile uint16_t g_propConstNum2;      // Numerator of proportionality constant for Proportional Feedback component
extern volatile uint16_t g_propConstDenom2;      // Denominator of proportionality constant for Proportional Feedback component
extern volatile uint16_t g_intConstNum2;         // Numerator of proportionality constant for Integral Feedback component
extern volatile uint16_t g_intConstDenom2;       // Denominator of proportionality constant for Integral Feedback component
extern volatile uint16_t g_derivConstNum2;       // Numerator of proportionality constant for Derivative Feedback component
extern volatile uint16_t g_derivConstDenom2;     // Denominator of proportionality constant for Derivative Feedback component
extern volatile int32_t g_displacement2Demand;   // Demand for encoder displacement (relative)


// Quadrature Encoder parameters
extern uint32_t g_encoder1ZeroPos;              // quadrature encoder zero position. This is a short-term reference position
extern uint32_t g_encoder2ZeroPos;              // quadrature encoder zero position. This is a short-term reference position
extern uint16_t g_encoderStepsPerMM_1;          // Number of encoder steps per mm of travel motor 1 (HF)
extern uint16_t g_encoderStepsPerMM_2;          // Number of encoder steps per mm of travel motor 2 (LR)
extern uint32_t g_landmark1Position;            // Position of Landmark for motor 1 in encoder units
extern uint32_t g_landmark2Position;            // Position of Landmark for motor 2 in encoder units

// Commands and other output-related variables
extern volatile bool g_startMotor;              // set to true to cause motor to start
extern volatile bool g_gotoLandmark;            // set to true to cause system to go to landmark





#endif	/* GLOBALS_H */

