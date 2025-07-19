#include <xc.h>
#include "globals_slave.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// *** VARIABLES THAT MUST BE THE SAME IN MASTER AND SLAVE ***
//PWM parameters
int16_t gs_pwm1Cycles=0;                    // Duty cycle parameter for PWM1
int16_t gs_pwm2Cycles=0;                    // Duty cycle parameter for PWM2
uint16_t gs_maxPWMInteger;                  // maximum PWM integer allowed

bool gs_output1Enabled;                     // true when motion output is enabled, false otherwise.
bool gs_output2Enabled;                     // true when motion output is enabled, false otherwise.
bool gs_userMotor1Enable;                   // true when the user requests motor 1 to be enabled on the UI
bool gs_userMotor2Enable;                   // true when the user requests motor 2 to be enabled on the UI

// Waveform parameters
volatile bool gs_resetWaveform = false;     // causes waveform array index to be reset to zero.
uint16_t gs_waveformUpdatePeriod;           // number of PWM1 interrupts between waveform index updates
uint16_t gs_numArrayVals;                   // number of array values in output waveform
uint16_t gs_freqUser;                       // frequency requested by user in cycles / min
uint16_t gs_motionAmplitudeMM1;             // amplitude of motion in mm motor 1
uint16_t gs_motionAmplitudeMM2;             // amplitude of motion in mm motor 2

volatile int32_t gs_displacement1Demand;    // Demand for encoder displacement (relative) for motor 1
volatile int32_t gs_displacement2Demand;    // Demand for encoder displacement (relative) for motor 2

uint16_t gs_reverseDirection1;              // 1 to move opposite direction, 0 for forward motor 1
uint16_t gs_reverseDirection2;              // 1 to move opposite direction, 0 for forward motor 2
uint16_t gs_waveformType;                   // stores which type of waveform is selected
uint32_t gs_waveformTimeStep_microS;        // Time between each element of the waveform array in microseconds


// Quadrature Encoder parameters
uint16_t gs_encoderStepsPerMM;               // Number of encoder steps per mm of travel

// Commands and other output-related variables
volatile bool gs_stopMotors;             // variable to cause zero voltage output when true
volatile bool gs_playSingleWaveformOnly;    // set to true to play just one period of the waveform array
volatile bool gs_startMotor;                // set to true to cause motor to start

//float gs_velReadsPerWfUpdate;               // number of times velocity is read per waveform update period (one index advancement) -- non integer 

// ****  END SECTION

// *** SECONDARY CORE GLOBAL PARAMETERS *****

// Waveform parameters
int32_t* gs_outputWaveform1 = NULL;          // pointer to output waveform for motor 1
int32_t* gs_outputWaveform2 = NULL;          // pointer to output waveform for motor 2
int32_t* gs_zeroWaveform = NULL;            // waveform of zeros useful for bringing output to zero gradually

int16_t gs_filtNumerator;                   // low-pass PWM filter parameter defined as integer numerator and denominator
int16_t gs_filtDenominator; 
