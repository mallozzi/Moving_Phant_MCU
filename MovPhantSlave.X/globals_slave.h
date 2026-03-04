

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef GLOBALS_SLAVE_H
#define	GLOBALS_SLAVE_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdbool.h>


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

// *** VARIABLES THAT MUST BE SYNCRONIZED BETWEEN MASTER AND SECONDARY ***


extern bool gs_output1Enabled;                      // true when motion output is enabled, false otherwise.
extern bool gs_output2Enabled;                      // true when motion output is enabled, false otherwise.
extern bool gs_userMotor1Enable;                    // true when the user requests motor 1 to be enabled on the UI
extern bool gs_userMotor2Enable;                    // true when the user requests motor 2 to be enabled on the UI

// PWM parameters
extern int16_t gs_pwm1Cycles;                       // Signed duty cycle parameter for PWM1.
extern int16_t gs_pwm2Cycles;                       // Signed duty cycle parameter for PWM2.
extern uint16_t gs_maxPWMInteger;                   // maximum PWM integer allowed

// Waveform Management
extern volatile bool gs_resetWaveform;              // causes waveform array index to be reset to zero.

// Waveform parameters
extern uint16_t gs_numArrayVals;                    // number of array values in output waveform
extern uint16_t gs_waveformUpdatePeriod;            // number of PWM1 interrupts between waveform index updates
extern uint16_t gs_waveformType;                    // stores which type of waveform is selected
extern uint16_t gs_motionAmplitudeMM1;              // amplitude of motion in mm motor 1
extern uint16_t gs_motionAmplitudeMM2;              // amplitude of motion in mm motor 2
extern uint16_t gs_reverseDirection1;               // 1 to move opposite direction, 0 for forward motor 1
extern uint16_t gs_reverseDirection2;               // 1 to move opposite direction, 0 for forward motor 2
extern uint32_t gs_waveformTimeStep_microS;         // Time between each element of the waveform array in microseconds
extern uint16_t gs_freqUser;                        // frequency requested by user in cycles / min

// motor control feedback parameters
extern volatile int32_t gs_displacement1Demand;     // Demand for encoder displacement (relative)for motor 1
extern volatile int32_t gs_displacement2Demand;     // Demand for encoder displacement (relative) for motor 2



// Quadrature Encoder parameters
extern uint16_t gs_encoderStepsPerMM;               // Number of encoder steps per mm of travel

// Commands and other output-related variables
extern volatile bool gs_stopMotors;              // variable to cause zero voltage output when true
extern volatile bool gs_playSingleWaveformOnly;     // set to true to play just one period of the waveform array
extern volatile bool gs_startMotor;                 // set to true to cause motor to start

//extern float gs_velReadsPerWfUpdate;                // number of times velocity is read per waveform update period (one index advancement) -- non integer 

// ****  END SECTION


// *** SECONDARY CORE ONLY GLOBAL PARAMETERS *****

// Waveform parameters
extern int32_t* gs_outputWaveform1;                 // pointer to output waveform for motor 1
extern int32_t* gs_outputWaveform2;                 // pointer to output waveform for motor 2
extern int32_t* gs_zeroWaveform;                    // waveform of zeros useful for bringing output to zero gradually
extern bool gs_waveformReset;                       // flag to indicate that custom waveform should be reset


extern int16_t gs_filtNumerator;                    // low-pass PWM filter parameter defined as integer numerator and denominator
extern int16_t gs_filtDenominator; 

#endif	/* GLOBALS_SLAVE_H */



