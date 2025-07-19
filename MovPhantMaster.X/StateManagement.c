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
        LATDbits.LATD1 = 1;    // brings motor driver chip out of sleep mode
    }
    else {
        LATDbits.LATD1 = 0;    // puts driver chip in sleep mode
    }
    
}

void startMotion() {
    // The purpose of this is to have a short function to call from an I2C command
    
    // First issue a stop motion command in case the motion is already happening
    stopMotion();
    // delay to  make sure slave core stops output. g_OscillatorFreq / 40000 is about 50 microseconds.
    // Be careful not to make this any longer than it needs to be, or slave-write-master-read FIFO could fill up
    __delay32(g_OscillatorFreq / 10000);   
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
    
    posLowByte = POS1CNTL; // Should load POS1CNTH into POS1HLD
    posHighByte = POS1HLD;
    g_encoder1ZeroPos = (posHighByte << 16) + posLowByte;  
    
    // Encoder 2 position is always kept up to date from reading it from the secondary
    g_encoder2ZeroPos = g_secondaryQuadEncPos;
}

void setLandmarkPosition() {
    // Reads the current encoder position registers for motor 1 and motor 2 and sets that them as the new landmark positions
    
    uint16_t posLowByte;
    uint32_t posHighByte;
    
    posLowByte = POS1CNTL; // Should load POS1CNTH into POS1HLD
    posHighByte = POS1HLD;
    g_landmark1Position = (posHighByte << 16) + posLowByte;
    
//    readSecondaryQuadEncoder();
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
    
    // Motor 1
    // ...Read current position
    posLowByte = POS1CNTL; // Should load POS1CNTH into POS1HLD
    posHighByte = POS1HLD;
    currentPosition = (posHighByte << 16) + posLowByte;
    
    // ...figure out how far to travel
    encoderStepsPerMM = (int16_t)g_encoderStepsPerMM;           
    encoderSteps = g_landmark1Position - currentPosition;
    travelDistMM = (int16_t)( (encoderSteps + encoderStepsPerMM/2) / encoderStepsPerMM );
    
    if(travelDistMM >= 0) {
        g_motionAmplitudeMM1 = (uint16_t)travelDistMM;
        g_reverseDirection1 = 0;
    }
    else {
        g_motionAmplitudeMM1 = (uint16_t)(-travelDistMM);
        g_reverseDirection1 = 1;
    }
    
    // Motor 2
    // ... read current position
//    readSecondaryQuadEncoder();         // encoder position is in g_secondaryQuadEncPos
    
    // ... figure out how far to travel
    encoderSteps = g_landmark2Position - g_secondaryQuadEncPos;
    travelDistMM = (int16_t)( (encoderSteps + encoderStepsPerMM/2) / encoderStepsPerMM );
    
    if(travelDistMM >= 0) {
        g_motionAmplitudeMM2 = (uint16_t)travelDistMM;
        g_reverseDirection2 = 0;
    }
    else {
        g_motionAmplitudeMM2 = (uint16_t)(-travelDistMM);
        g_reverseDirection2 = 1;
    }
 
    // common to both motors
    g_waveformType = 1;  // Ramp
    g_freqUser = 30;     // do it in 2 seconds (30 cycles / min)
    
    g_gotoLandmark = false;    // so this function is not executed again
    g_startMotor = true;
    
}

void setLED1(uint16_t onoff) {
    LATDbits.LATD10 = onoff;
}
