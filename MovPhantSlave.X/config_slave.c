#include <xc.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "config_slave.h"
#include "globals_slave.h"

void configSlaveInitial() {
    
    // Start by making all I/O pins digital outputs in their low state
    ANSELA = 0;
    TRISA = 0;
    LATA = 0;
    ANSELB = 0;
    TRISB = 0;
    LATB = 0;
    ANSELC = 0;
    TRISC = 0;
    LATC = 0;
    ANSELD = 0;
    TRISD = 0;
    LATD = 0;
    
    INTCON1bits.NSTDIS = 1;                     // allow nested interrupts so that CPU priority can be changed on the fly
    gs_numArrayVals = 400;                      // number of array values in output waveform. Will be overwritten
    
    gs_filtNumerator = 97;                      // low-pass PWM filter parameter defined as integer numerator and denominator
    gs_filtDenominator = 100;
    
// Some state variables
    gs_stopMotors = true;                    // will get set when starting motion
    gs_output1Enabled = false;
    gs_output2Enabled = false;
    
    // LED 2 Pin
    TRISBbits.TRISB1 = 0;
 
    // Configure Motor 1 and 2 Driver, direction outputs as digital output pins
    TRISCbits.TRISC4 = 0;                       // PWM_Driver1 (Motor 1)
    TRISCbits.TRISC5 = 0;                       // DIR1  (Motor 1 direction)
    TRISCbits.TRISC10 = 0;                      // PWM_Driver2 (Motor 2)
    TRISCbits.TRISC11 = 0;                      // DIR2  (Motor 2 direction)
    
    // Quadrature encoder inputs. No analog capability on these pins, so no need to set ANSELB
    TRISBbits.TRISB11 = 1;                      // Quad Encoder 2 A channel
    TRISBbits.TRISB13 = 1;                      // Quad Encoder 2 B channel
   
}

void configureSecondaryPPS() {
        // Unlock control register
    __builtin_write_RPCON(0x0000);
    
    TRISBbits.TRISB11 = 1;                      // Quad Encoder 2 A channel
    TRISBbits.TRISB13 = 1;                      // Quad Encoder 2 B channel
    
    // Assign RB11 to Quadrature Input A and RB13 to Quadrature Input B
    RPINR14bits.QEIA1R = 43;
    RPINR14bits.QEIB1R = 45;
    
    // Lock control register
    __builtin_write_RPCON(0x0800);
}

void configureQuadEncoder() {
    
    QEI1CONbits.QEIEN = 1;      // Enable quad encoder module
    QEI1CONbits.PIMOD = 0;      // Index input does not affect counter
    
    // Initialize counter to approximately halfway through its full range to avoid rolling over
    // counter is 32 bits, so halfway through in hex is 0x7FFF FFFF
    POS1CNTH = 0x0100;          // This line seems unnecessary and may be a relic of testing. Test without it at some point
    POS1HLD = 0x7FFF;           // Write high bit to Position 1 Counter Hold Register
    POS1CNTL = 0x0000;    
}

//void setOutputWaveformMotor1(int32_t* waveformArray) {
//    // Sets the output waveform array to waveformArray. 
//    if(gs_outputWaveform1 != NULL) {
//        free(gs_outputWaveform1);
//    }
//    gs_outputWaveform1 = waveformArray;
//}

void setUpWaveforms() {
    //
    
    int16_t signedAmplitudeMM;
    
    // free memory from previous waveforms
    if(gs_outputWaveform1 != NULL) {
        free(gs_outputWaveform1);
        gs_outputWaveform1 = NULL;
    }
    if(gs_outputWaveform2 != NULL) {
        free(gs_outputWaveform2);
        gs_outputWaveform2 = NULL;
    }
    
    // set up motor 1 waveforms
    if(gs_reverseDirection1 == 0) {
       signedAmplitudeMM = (int16_t)gs_motionAmplitudeMM1;     
    }
    else {
        signedAmplitudeMM = -(int16_t)gs_motionAmplitudeMM1; 
    }
    if(gs_waveformType == 0) {  // sine waveform
        gs_outputWaveform1 = designPosSineWaveform(signedAmplitudeMM);      
    }
    else if(gs_waveformType == 1) {  //Step waveform
        gs_outputWaveform1 = designRampWaveform(signedAmplitudeMM);
    }
    
    // set up motor 2 waveforms
    if(gs_reverseDirection2 == 0) {
       signedAmplitudeMM = (int16_t)gs_motionAmplitudeMM2;     
    }
    else {
        signedAmplitudeMM = -(int16_t)gs_motionAmplitudeMM2; 
    }
    if(gs_waveformType == 0) {  // sine waveform
        gs_outputWaveform2 = designPosSineWaveform(signedAmplitudeMM);      
    }
    else if(gs_waveformType == 1) {  //Step waveform
        gs_outputWaveform2 = designRampWaveform(signedAmplitudeMM);
    }
}

int32_t* designPosSineWaveform(int16_t mmDisplacementPP) {
    // Creates a sine waveform designed to give mmDisplacement as the peak-peak position
    // displacement
    
    int32_t posAmplitudePP;  // encoder units
    int32_t* waveformArray;
    
    posAmplitudePP = (int32_t)mmDisplacementPP * (int32_t)gs_encoderStepsPerMM;
    waveformArray = makePosSineWaveform(posAmplitudePP, gs_numArrayVals);
//    setOutputWaveformMotor1(waveformArray);   
    gs_playSingleWaveformOnly = false;   // plays multiple waveforms
    return waveformArray;
}

int32_t* designRampWaveform(int16_t mmStepSize) {
    // designs the ramp waveform to step by at total of mmStepSize mm. Plays one waveform only
    // returns waveform array pointer
    
    int32_t stepSizeEncoder;            // number of encoder steps (signed)
    int32_t* waveformArray;
    
    stepSizeEncoder = (int32_t)mmStepSize * (int32_t)gs_encoderStepsPerMM;
    waveformArray = makeRampWaveform(stepSizeEncoder, gs_numArrayVals);
//    setOutputWaveformMotor1(waveformArray); 
    gs_playSingleWaveformOnly = true;    // play waveform only once, then stop
    return waveformArray;
}

int32_t* makePosSineWaveform(int32_t amplitudeEncPP, uint16_t numValues) {
    // Allocates a sine waveform of a given amplitude number of values numValues. Also allocates the global zero waveform of the same length.
    // INPUTS
    // amplitudeEncPP is the peak-to-peak amplitude of the waveform in units of encoder steps
    // numValues is the number of values in the array.
    // OUTPUT
    // waveformArray is an array of int32_t with the waveform values
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
    
    gs_numArrayVals = numValues;

    return waveformArray;
}

void allocateArbitraryWaveform(uint16_t nPts) {
    // Allocates the array for an arbitrary waveform that later gets populated from values transmitted from CPU
    
    // First free existing waveform memory
    // free memory from previous waveforms
    if(gs_outputWaveform1 != NULL) {
        free(gs_outputWaveform1);
        gs_outputWaveform1 = NULL;
    }
    if(gs_outputWaveform2 != NULL) {
        free(gs_outputWaveform2);
        gs_outputWaveform2 = NULL;
    }
    
    // Allocate new arrays
    gs_outputWaveform1 = (int32_t*)malloc(nPts*sizeof(int32_t));
    gs_outputWaveform2 = (int32_t*)malloc(nPts*sizeof(int32_t));
    
}

void setWaveformValue(int32_t value, uint16_t whichWaveform) {
    // Sets a custom waveform data value at a given index. The index is tracked as a static variable.
    // INPUTS
    // value is the value of the waveform at the specific indx
    // whichWaveform is 1 to set Motor 1 (HF motion), 2 for Motor 2 (LR motion)
    static uint16_t indx=0;
    if(gs_waveformReset) {
        indx=0;
        gs_waveformReset = false;      // so that next incoming data value does not reset index
    }
    
    if(whichWaveform==1) {
        gs_outputWaveform1[indx] = value;
    }
    else{
        gs_outputWaveform2[indx] = value;
    }
    indx++;
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
