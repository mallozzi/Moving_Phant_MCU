#include <xc.h>
#include <libpic30.h>
#include <stdlib.h>
#include "StateManagement.h"
#include "globals.h"
#include "MathUtil.h"
#include "Configure.h"
#include "MSCommunication_mstr.h"
#include "motorCntrlMstr.h"

void enableDriver(bool enable) {
    // Sends signal to bring driver chip out of sleep mode or put it in sleep mode. Input is true to enable, false to put in sleep.
    if(enable) {
        LATBbits.LATB13 = 1;    // brings motor driver chip out of sleep mode
    }
    else {
        LATBbits.LATB13 = 0;    // puts driver chip in sleep mode
    }
    
}

void startMotion() {
    // The purpose of this is to have a short function to call from an I2C command
    
    // First issue a stop motion command in case the motion is already happening
    stopMotion();
    // delay to  make sure slave core stops output. g_OscillatorFreq / 40000 is about 50 microseconds.
    // Be careful not to make this any longer than it needs to be, or slave-write-master-read FIFO could fill up
    __delay32(g_OscillatorFreq / 40000);   
    g_startMotor = true;

}

void stopMotion() {
    // The purpose of this is to have a short function to call from an I2C command
    sendCommandToSecondary(STOP_MOTION);
//    g_outputWaveform = g_zeroWaveform;
//    g_stopMotionIssued = true;
//    g_zeroPosOutput = true;
 //   g_velocityDemand = 0;
    
}


void setZeroPosition() {
    // Reads the current encoder position register and sets that as the new zero
    uint16_t posLowByte;
    uint32_t posHighByte;
    uint16_t pingvalue = 9;  // arbitrary integer
    
    posLowByte = POS1CNTL; // Should load POS1CNTH into POS1HLD
    posHighByte = POS1HLD;
    g_encoder1ZeroPos = (posHighByte << 16) + posLowByte;  
    
    // Encoder 2 - more complicated because it is on the secondary core
    //... read the encoder position
    readSecondaryQuadEncoder();
    g_encoder2ZeroPos = g_secondaryQuadEncPos;
}

void setLandmarkPosition() {
    // Reads the current encoder position registers for motor 1 and motor 2 and sets that them as the new landmark positions
    
    uint16_t posLowByte;
    uint32_t posHighByte;
    
    posLowByte = POS1CNTL; // Should load POS1CNTH into POS1HLD
    posHighByte = POS1HLD;
    g_landmark1Position = (posHighByte << 16) + posLowByte;
    
    readSecondaryQuadEncoder();
    g_landmark2Position = g_secondaryQuadEncPos;
}

void gotoLandmark() {
    // Designs a ramp pulse to go from current position to landmark
    // Find out how far to travel
    
    uint16_t posLowByte;
    uint32_t posHighByte;
    uint32_t currentPosition;
    int32_t encoderSteps;
    int16_t travelDistMM;       // signed distance to travel in mm
    int16_t encoderStepsPerMM; 
    
    // Read current position
    posLowByte = POS1CNTL; // Should load POS1CNTH into POS1HLD
    posHighByte = POS1HLD;
    currentPosition = (posHighByte << 16) + posLowByte;
    
    encoderStepsPerMM = (int16_t)g_encoderStepsPerMM;           
    encoderSteps = g_landmark1Position - currentPosition;
    travelDistMM = (int16_t)( (encoderSteps + encoderStepsPerMM/2) / encoderStepsPerMM );
    
    if(travelDistMM >= 0) {
        g_motionAmplitudeMM = (uint16_t)travelDistMM;
        g_reverseDirection = 0;
    }
    else {
        g_motionAmplitudeMM = (uint16_t)(-travelDistMM);
        g_reverseDirection = 1;
    }
    g_waveformType = 1;  // Ramp
    g_freqUser = 30;     // do it in 2 seconds (30 cycles / min)
    
    g_gotoLandmark = false;    // so this function is not executed again
    g_startMotor = true;
    
}

//void setUpWaveform() {
//    int16_t signedAmplitudeMM;
//    // Adjust waveform update time (time between index advances) based on the frequency requested. The goal is to have a 10 ms update time
//    // for waveforms with a period of less than 8 seconds, 20 ms for 8-16, and 40 ms for periods longer than 16s. Assuming that the max pwm1
//    // integer is 12799, and the input clocking is set up so that this corresponds to 50 microseconds per interrupt time, then a value
//    // of g_waveformUpdatePeriod of 200 gives 10 ms update time.
//    // 
//    uint16_t thresh1 = 60;  // period of 1 sec
//    uint16_t thresh2 = 8;   // period of 7.5 sec
//    uint16_t thresh3 = 4;   //period of 15 sec
//    uint16_t thresh4 = 2;   // period of 30 sec
//    if(g_freqUser > thresh1) {
//        g_waveformUpdatePeriod = 100;  // targets 5 ms update period assuming PWM1 max integer of 12799
//    }
//    else if(g_freqUser > thresh2)
//    {
//        g_waveformUpdatePeriod = 200;  // targets 10 ms update period assuming PWM1 max integer of 12799
//    }
//    else if(g_freqUser > thresh3) {
//        g_waveformUpdatePeriod = 400; // targets 20 ms update period assuming PWM1 max integer of 12799
//    }
//    else if(g_freqUser > thresh4) {
//        g_waveformUpdatePeriod = 800; // targets 40 ms update period assuming PWM1 max integer of 12799
//    }
//    else {
//        g_waveformUpdatePeriod = 1600; // targets 80 ms update period assuming PWM1 max integer of 12799
//    }
//    configureDerivedQuantities();
//    
//    // free memory from previous waveform
//    if(g_outputWaveform != NULL) {
//        free(g_outputWaveform);
//    }
//    
//    // set up waveforms
//    if(g_reverseDirection == 0) {
//       signedAmplitudeMM = (int16_t)g_motionAmplitudeMM;     
//    }
//    else {
//        signedAmplitudeMM = -(int16_t)g_motionAmplitudeMM; 
//    }
//    if(g_waveformType == 0) {  // sine waveform
//        designPosSineWaveform(signedAmplitudeMM);      
//    }
//    else if(g_waveformType == 1) {  //Step waveform
//        designRampWaveform(signedAmplitudeMM);
//    }
//}
//
//void designPosSineWaveform(int16_t mmDisplacementPP) {
//    // Creates a sine waveform designed to give mmDisplacement as the peak-peak position
//    // displacement
//    
//    int32_t posAmplitudePP;  // encoder units
//    int32_t* waveformArray;
//    
//    posAmplitudePP = (int32_t)mmDisplacementPP * (int32_t)g_encoderStepsPerMM;
//    waveformArray = makePosSineWaveform(posAmplitudePP, g_numArrayVals);
//    setOutputWaveform(waveformArray);   
//    g_playSingleWaveformOnly = false;   // plays multiple waveforms
//}
//
//void designRampWaveform(int16_t mmStepSize) {
//    // designs the ramp waveform to step by at total of mmStepSize mm. Plays one waveform only
//    int32_t stepSizeEncoder;            // number of encoder steps (signed)
//    int32_t* waveformArray;
//    
//    stepSizeEncoder = (int32_t)mmStepSize * (int32_t)g_encoderStepsPerMM;
//    waveformArray = makeRampWaveform(stepSizeEncoder, g_numArrayVals);
//    setOutputWaveform(waveformArray); 
//    g_playSingleWaveformOnly = true;    // play waveform only once, then stop
//}
//
//
//void designVelSineWaveform(int16_t mmDisplacementPP) {
//    // Creates a sine waveform designed to give mmDisplacement as the peak-peak position
//    // displacement
//    
//    int32_t posAmplitudePP;
//    int32_t* waveformArray;
//    
//    posAmplitudePP = (int32_t)mmDisplacementPP * (int32_t)g_encoderStepsPerMM;
//    waveformArray = makeVelSineWaveform(posAmplitudePP, g_numArrayVals);
//    setOutputWaveform(waveformArray);
//    
//}
//
//void designVelPulseWaveform(int16_t mmDisplacement) {
//    int32_t posAmplitudePP;
//    int32_t* waveformArray;
//    
//    posAmplitudePP = (int32_t)mmDisplacement * (int32_t)g_encoderStepsPerMM;
//    waveformArray = makeWideVelPulseWaveform(posAmplitudePP, g_numArrayVals);
//    setOutputWaveform(waveformArray);
//}

