#include <xc.h>
#include "globals_slave.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// *** VARIABLES THAT MUST BE THE SAME IN MASTER AND SLAVE ***
//PWM parameters
int16_t gs_pwm1Cycles=0;                      // Duty cycle parameter for PWM1
uint16_t gs_maxPWMInteger;                  // maximum PWM integer allowed

bool gs_outputEnabled;              // true when motion output is enabled, false otherwise.

// Waveform parameters
volatile bool gs_resetWaveform = false;     // causes waveform array index to be reset to zero.
uint16_t gs_waveformUpdatePeriod;           // number of PWM1 interrupts between waveform index updates
uint16_t gs_numArrayVals;                   // number of array values in output waveform
uint16_t gs_freqUser;                       // frequency requested by user in cycles / min
uint16_t gs_motionAmplitudeMM;              // amplitude of motion in mm

volatile int32_t gs_displacementDemand;     // Demand for encoder displacement (relative)

uint16_t gs_reverseDirection;               // 1 to move opposite direction, 0 for forward
uint16_t gs_waveformType;                   // stores which type of waveform is selected
uint32_t gs_waveformTimeStep_microS;        // Time between each element of the waveform array in microseconds


// Quadrature Encoder parameters
//uint32_t g_encoderZeroPos;                  // quadrature encoder zero position
uint16_t gs_encoderStepsPerMM;               // Number of encoder steps per mm of travel
//uint32_t g_landmarkPosition;                // Position of Landmark in encoder units

// Commands and other output-related variables
volatile bool gs_zeroPosOutput;             // variable to cause zero voltage output when true
volatile bool gs_playSingleWaveformOnly;    // set to true to play just one period of the waveform array
volatile bool gs_startMotor;                // set to true to cause motor to start

//float gs_velReadsPerWfUpdate;               // number of times velocity is read per waveform update period (one index advancement) -- non integer 

// ****  END SECTION

// *** SECONDARY CORE GLOBAL PARAMETERS *****

// Waveform parameters
int32_t* gs_outputWaveform = NULL;          // pointer to whatever array is currently being used to determine output. It could represent speed or position
int32_t* gs_zeroWaveform = NULL;            // waveform of zeros useful for bringing output to zero gradually

int16_t gs_filtNumerator;                   // low-pass PWM filter parameter defined as integer numerator and denominator
int16_t gs_filtDenominator; 
