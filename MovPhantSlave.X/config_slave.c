#include <xc.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "config_slave.h"
#include "globals_slave.h"

void configSlaveInitial() {
    gs_numArrayVals = 400;                      // number of array values in output waveform. Will be overwritten
    
    gs_filtNumerator = 97;                      // low-pass PWM filter parameter defined as integer numerator and denominator
    gs_filtDenominator = 100;
    
// Some state variables
    gs_zeroPosOutput = true;                    // will get set when starting motion
    gs_outputEnabled = false;
    
    TRISBbits.TRISB15 = 0;                      // dir output
    TRISBbits.TRISB14 = 0;                      // pwm output
   
}

void setOutputWaveform(int32_t* waveformArray) {
    // Sets the output waveform array to waveformArray. If the output waveform is not already set
    // to waveformArray, it frees the memory first the global variable containing the output waveform array.
    if(gs_outputWaveform != NULL && gs_outputWaveform != waveformArray) {
        free(gs_outputWaveform);
    }
    gs_outputWaveform = waveformArray;
}

void setUpWaveform() {
    int16_t signedAmplitudeMM;
    // Adjust waveform update time (time between index advances) based on the frequency requested. The goal is to have a 10 ms update time
    // for waveforms with a period of less than 8 seconds, 20 ms for 8-16, and 40 ms for periods longer than 16s. Assuming that the max pwm1
    // integer is 12799, and the input clocking is set up so that this corresponds to 50 microseconds per interrupt time, then a value
    // of g_waveformUpdatePeriod of 200 gives 10 ms update time.
    // 
//    uint16_t thresh1 = 60;  // period of 1 sec
//    uint16_t thresh2 = 8;   // period of 7.5 sec
//    uint16_t thresh3 = 4;   //period of 15 sec
//    uint16_t thresh4 = 2;   // period of 30 sec
//    if(gs_freqUser > thresh1) {
//        gs_waveformUpdatePeriod = 100;  // targets 5 ms update period assuming PWM1 max integer of 12799
//    }
//    else if(gs_freqUser > thresh2)
//    {
//        gs_waveformUpdatePeriod = 200;  // targets 10 ms update period assuming PWM1 max integer of 12799
//    }
//    else if(gs_freqUser > thresh3) {
//        gs_waveformUpdatePeriod = 400; // targets 20 ms update period assuming PWM1 max integer of 12799
//    }
//    else if(gs_freqUser > thresh4) {
//        gs_waveformUpdatePeriod = 800; // targets 40 ms update period assuming PWM1 max integer of 12799
//    }
//    else {
//        gs_waveformUpdatePeriod = 1600; // targets 80 ms update period assuming PWM1 max integer of 12799
//    }
//    configureDerivedQuantities();
    
    // free memory from previous waveform
    if(gs_outputWaveform != NULL) {
        free(gs_outputWaveform);
        gs_outputWaveform = NULL;
    }
    
    // set up waveforms
    if(gs_reverseDirection == 0) {
       signedAmplitudeMM = (int16_t)gs_motionAmplitudeMM;     
    }
    else {
        signedAmplitudeMM = -(int16_t)gs_motionAmplitudeMM; 
    }
    if(gs_waveformType == 0) {  // sine waveform
        designPosSineWaveform(signedAmplitudeMM);      
    }
    else if(gs_waveformType == 1) {  //Step waveform
        designRampWaveform(signedAmplitudeMM);
    }
}

void designPosSineWaveform(int16_t mmDisplacementPP) {
    // Creates a sine waveform designed to give mmDisplacement as the peak-peak position
    // displacement
    
    int32_t posAmplitudePP;  // encoder units
    int32_t* waveformArray;
    
//    if(gs_numArrayVals > 0) {
//        LATBbits.LATB1 = 1;
//    }
    posAmplitudePP = (int32_t)mmDisplacementPP * (int32_t)gs_encoderStepsPerMM;
    waveformArray = makePosSineWaveform(posAmplitudePP, gs_numArrayVals);
    setOutputWaveform(waveformArray);   
    gs_playSingleWaveformOnly = false;   // plays multiple waveforms
    
//    if(gs_numArrayVals == 0) {
//        LATBbits.LATB1 = 1;
//    }
}

void designRampWaveform(int16_t mmStepSize) {
    // designs the ramp waveform to step by at total of mmStepSize mm. Plays one waveform only
    int32_t stepSizeEncoder;            // number of encoder steps (signed)
    int32_t* waveformArray;
    
    stepSizeEncoder = (int32_t)mmStepSize * (int32_t)gs_encoderStepsPerMM;
    waveformArray = makeRampWaveform(stepSizeEncoder, gs_numArrayVals);
    setOutputWaveform(waveformArray); 
    gs_playSingleWaveformOnly = true;    // play waveform only once, then stop
}


//void designVelSineWaveform(int16_t mmDisplacementPP) {
//    // Creates a sine waveform designed to give mmDisplacement as the peak-peak position
//    // displacement
//    
//    int32_t posAmplitudePP;
//    int32_t* waveformArray;
//    
//    posAmplitudePP = (int32_t)mmDisplacementPP * (int32_t)gs_encoderStepsPerMM;
//    waveformArray = makeVelSineWaveform(posAmplitudePP, gs_numArrayVals);
//    setOutputWaveform(waveformArray);
//    
//}

//void designVelPulseWaveform(int16_t mmDisplacement) {
//    int32_t posAmplitudePP;
//    int32_t* waveformArray;
//    
//    posAmplitudePP = (int32_t)mmDisplacement * (int32_t)gs_encoderStepsPerMM;
//    waveformArray = makeWideVelPulseWaveform(posAmplitudePP, gs_numArrayVals);
//    setOutputWaveform(waveformArray);
//}

int32_t* makePosSineWaveform(int32_t amplitudeEncPP, uint16_t numValues) {
    // Allocates a sine waveform of a given amplitude number of values numValues. Also allocates the global zero waveform of the same length.
    // INPUTS
    // amplitudeEncPP is the peak-to-peak amplitude of the waveform in units of encoder steps
    // numValues is the number of values in the array.
    uint16_t ii;
    float x;
    float step = 2*3.14159265359 / numValues;
    float amplitude;                            // true encoder amplitude rather than peak-to-peak
    
    amplitude = (float)amplitudeEncPP / 2.0;
    if(gs_zeroWaveform != NULL) {
        free(gs_zeroWaveform);
    }
    
    // Allocate memory. Note: to use dynamic memory allocation, a heap must be 
    //  ...created in the linker. See notes on project
    int32_t* waveformArray = (int32_t*)malloc(numValues*sizeof(int32_t));
    for(ii=0; ii<numValues; ii++) {
        x = amplitude * sin(ii*step);
        waveformArray[ii] = (int32_t)(x+0.5);
    }
    
//    g_zeroWaveform = makeZeroWaveform(numValues);  // create corresponding zero waveform
    gs_numArrayVals = numValues;
    
    return waveformArray;  
}


//int32_t* makeVelSineWaveform(int32_t posAmplitude, uint16_t numValues) {
//    // Allocates a sine waveform of a given amplitude number of values numValues. Also allocates the global zero waveform of the same length.
//    // This is designed as a velocity-controlled waveform, so the amplitude is estimated so that the position
//    // displacement is the posAmplitude input.
//    // INPUTS
//    // posAmplitude is the target position displacement (0-peak) in units of encoder steps 
//    uint16_t ii;
//    float x;
//    float step = 2*3.14159265359 / numValues;
//    float amplitude;
//    
//    // scale amplitude to make the position amplitude (0-peak) equal to the posAmplitude input
//    // A half sine wave has average value of 2/pi = 0.636953.
//     amplitude = (float)posAmplitude / numValues / gs_velReadsPerWfUpdate / 0.635953;
//    //amplitude = (float)posAmplitude / numValues / 0.635953;
//    
//    
//    if(gs_zeroWaveform != NULL) {
//        free(gs_zeroWaveform);
//    }
//    
//    // Allocate memory. Note: to use dynamic memory allocation, a heap must be 
//    //  ...created in the linker. See notes on project
//    int32_t* waveformArray = (int32_t*)malloc(numValues*sizeof(int32_t));
//    for(ii=0; ii<numValues; ii++) {
//        x = amplitude * sin(ii*step);
//        waveformArray[ii] = (int32_t)(x+0.5);
//    }
//    
// //   g_zeroWaveform = makeZeroWaveform(numValues);  // create corresponding zero waveform
//    gs_numArrayVals = numValues;
//    
//    return waveformArray;  
//}

int32_t* makeRampWaveform(int32_t stepSize, uint16_t numValues) {
    // Creates a waveform meant to move the motor a defined amount. The last 10%
    // of the waveform sits steady to allow the motor to settle.
    // INPUTS
    // stepSize is the number of encoder steps to move the motor
    // numValues is the number of values in the array
    uint16_t ii;
    float rampValue;
    float rampIncrement;
    uint16_t settleInd;         // index where ramping is finished to allow settling time
    
    if(gs_zeroWaveform != NULL) {
        free(gs_zeroWaveform);
    }

    // Allocate memory. Note: to use dynamic memory allocation, a heap must be 
    //  ...created in the linker. See notes on project
    int32_t* waveformArray = (int32_t*)malloc(numValues*sizeof(int32_t));
    
    settleInd = (uint16_t)(numValues * 0.9);        // float operation instead of multiplying by 9/10 in case someone makes very long array causing overflow
    rampIncrement = (float)(stepSize) / settleInd;
    rampValue = 0;
    
    for(ii=0; ii<numValues; ii++) {
        if(ii < settleInd) {
            waveformArray[ii] = rampValue;
            rampValue+= rampIncrement;
        }
        else {
            waveformArray[ii] = rampValue;          // constant for last portion of array
        }
    }
    
 //   g_zeroWaveform = makeZeroWaveform(numValues);
    gs_numArrayVals = numValues;
 //   g_playSingleWaveformOnly = true;
    return waveformArray;
}

int32_t* makeWideVelPulseWaveform(int32_t totalSteps, uint16_t numValues) {
    // Allocates an inverted cosine waveform of a given amplitude number of values numValues. 
    // Waveform is one period, starting and ending at 0. The waveform is intended as a velocity
    // waveform, with the total number of steps targeted to be equal to the totalSteps input
    // Also allocates the global zero waveform of the same length.
    // INPUTS
    // totalSteps is the total encoder steps to take in units of encoder steps. This will be used as a target
    //      target in calculating the pulse amplitude, assuming this is a velocity waveform
    uint16_t ii;
    float x;
    float step = 2*3.14159265359 / numValues;
    float amplitude;
    
    //amplitude = (float)totalSteps / (g_velReadsPerWfUpdate * numValues);
    amplitude = (float)totalSteps / ( numValues);
    
    if(gs_zeroWaveform != NULL) {
        free(gs_zeroWaveform);
    }
    
    // Allocate memory. Note: to use dynamic memory allocation, a heap must be 
    //  ...created in the linker. See notes on project
    int32_t* waveformArray = (int32_t*)malloc(numValues*sizeof(int32_t));
    for(ii=0; ii<numValues; ii++) {
        x = -amplitude * (cos(ii*step)-1.0);
        waveformArray[ii] = (int32_t)(x+0.5);
    }
    
//    g_zeroWaveform = makeZeroWaveform(numValues);  // create corresponding zero waveform
    gs_numArrayVals = numValues;
    return waveformArray;  
}

int32_t* makeZeroWaveform(uint16_t numValues) {
    // Creates a waveform of zeros that is useful in bringing output to zero gracefully. Returns a pointer to the 
    // newly-allocated array of zeros
    
    uint16_t ii;
    int32_t* waveformArray = (int32_t*)malloc(numValues*sizeof(int32_t));
    for(ii=0; ii<numValues; ii++) {
        waveformArray[ii] = 0;
    }
    
    return waveformArray;
}

int32_t* makeConstWaveform(int32_t value, uint16_t numValues) {
    // Creates a waveform of constant values 
    
    uint16_t ii;
    int32_t* waveformArray = (int32_t*)malloc(numValues*sizeof(int32_t));
    for(ii=0; ii<numValues; ii++) {
        waveformArray[ii] = value;
    }
    
    return waveformArray;
}

//void calcNumPoints() {
//    // Calculate the number of points in one period of the waveform (one pass through the waveform array). The intention is that the caller
//    // of this function knows the value of g_waveformTimeStep_microS (update time of waveform index)
//    // and ensures that the period of the waveform array is an integer number of waveform update time steps. This function will round to the
//    // nearest number of points just in case.
//    
//    float period_microSec =  60.0 * 1000000.0 / (float)gs_freqUser;
//    gs_numArrayVals = (uint16_t)( period_microSec / (float)gs_waveformTimeStep_microS + 0.5 );
//  
//}