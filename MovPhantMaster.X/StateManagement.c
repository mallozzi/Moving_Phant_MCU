#include <xc.h>
#include <libpic30.h>
#include <stdlib.h>
#include "StateManagement.h"
#include "globals.h"
#include "MathUtil.h"
#include "Configure.h"
#include "MSCommunication_mstr.h"
#include "motorCntrlMstr.h"
#include "enums.h"

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
    
//    INTCON2bits.GIE  = 0;                   // disable global interrupts
//    readSecondaryQuadEncoder();             // Perform initial read of secondary quadrature encoder
//    INTCON2bits.GIE  = 1;                   //global interrupt enable
    
    // delay to  make sure slave core stops output. g_OscillatorFreq / 40000 is about 50 microseconds.
    // Be careful not to make this any longer than it needs to be, or slave-write-master-read FIFO could fill up
    __delay32(g_OscillatorFreq / 10000);   
    g_stopButtonPushed = false;
    g_statusFlags = 0;    // Clear status flags  
    g_startMotor = true;
    g_landmarkMode = false;

}

void stopMotion() {
    // The purpose of this is to have a short function to call from an I2C command
    sendCommandToSecondary(STOP_MOTION);
    g_landmarkMode = false;
    // For diagnostics - turn off LED 1
//    LATDbits.LATD10 = 0;
//    g_outputWaveform = g_zeroWaveform;
//    g_stopMotionIssued = true;
//    g_zeroPosOutput = true;
 //   g_velocityDemand = 0;
    
}


void setZeroPosition() {
    // Reads the current encoder position and sets as the new zero for each motor
    uint16_t posLowByte;
    uint32_t posHighByte;
    
    // Motor 1
    posLowByte = POS1CNTL; // Should load POS1CNTH into POS1HLD
    posHighByte = POS1HLD;
    g_encoder1ZeroPos = (posHighByte << 16) + posLowByte;  
    
    // Motor 2. Encoder 2 position is always kept up to date from reading it from the secondary
    g_encoder2ZeroPos = g_secondaryQuadEncPos;
}

void setLandmarkPosition() {
    // Reads the current encoder position registers for motor 1 and motor 2 and sets that them as the new landmark positions
    
    uint16_t posLowByte;
    uint32_t posHighByte;
    
    posLowByte = POS1CNTL; // Should load POS1CNTH into POS1HLD
    posHighByte = POS1HLD;
    g_landmark1Position = (posHighByte << 16) + posLowByte;
    
    // Encoder 2 position is always kept up to date from reading it from the secondary
    g_landmark2Position = g_secondaryQuadEncPos;
    
    setZeroPosition();      // make the current position the reference too.
}

void gotoLandmark() {
    // Designs a ramp pulse to go from current position to landmark
    // Find out how far to travel
    
    uint16_t posLowByte;
    uint32_t posHighByte;
    uint32_t currentPosition;
    int32_t currentPositionSigned;
    int32_t encoderSteps1;
    int32_t encoderSteps2;
    int16_t travelDistMM1;       // signed distance to travel in mm motor 1
    int16_t travelDistMM2;       // signed distance to travel in mm motor 2
    int16_t encoderStepsPerMM1; 
    int16_t encoderStepsPerMM2; 
    
    // Motor 1
    // ...Read current position
    posLowByte = POS1CNTL; // Should load POS1CNTH into POS1HLD
    posHighByte = POS1HLD;
    currentPosition = (posHighByte << 16) + posLowByte;
    currentPositionSigned = (int32_t)currentPosition;        // value will never be big enough to wrap bits
    
    // ...figure out how far to travel
    encoderStepsPerMM1 = (int16_t)g_encoderStepsPerMM_1;           
    encoderSteps1 = (int32_t)g_landmark1Position - currentPositionSigned;
    travelDistMM1 = __builtin_divsd(encoderSteps1 + encoderStepsPerMM1/2, encoderStepsPerMM1);  // use built in to get the rounding toward zero rather than neg infinity
   // travelDistMM1 = (int16_t)( (encoderSteps1 + encoderStepsPerMM1/2) / encoderStepsPerMM1 );
    
    if(travelDistMM1 >= 0) {
        g_motionAmplitudeMM1 = (uint16_t)travelDistMM1;
        g_reverseDirection1 = 1;
    }
    else {
        g_motionAmplitudeMM1 = (uint16_t)(-travelDistMM1);
        g_reverseDirection1 = 0;
    }
    
    // Motor 2
    
    // Quad encoder for motor 2 is read constantly so does not have to be read here.
    // ... figure out how far to travel
    encoderStepsPerMM2 = (int16_t)g_encoderStepsPerMM_2;
    encoderSteps2 = (int32_t)g_landmark2Position - (int32_t)g_secondaryQuadEncPos;
    travelDistMM2 = __builtin_divsd(encoderSteps2 + encoderStepsPerMM2/2, encoderStepsPerMM2);  // use built in to get the rounding toward zero rather than neg infinity
    //travelDistMM2 = (int16_t)( (encoderSteps2 + encoderStepsPerMM2/2) / encoderStepsPerMM2 );
    
    if(travelDistMM2 >= 0) {
        g_motionAmplitudeMM2 = (uint16_t)travelDistMM2;
        g_reverseDirection2 = 0;
    }
    else {
        g_motionAmplitudeMM2 = (uint16_t)(-travelDistMM2);
        g_reverseDirection2 = 1;
    }
 
    // common to both motors
    g_waveformType = 1;  // Ramp
    if(g_motionAmplitudeMM1 > 10 || g_motionAmplitudeMM2 > 3) {
        g_freqUser = 15;     // do it in 4 seconds (15 cycles / min)
    }
    else {
        g_freqUser = 30;     // do it in 2 seconds (30 cycles / min)
    }
        
    
    //enable both motors
    g_userMotor1Enable = true;
    g_userMotor2Enable = true;
    
    g_gotoLandmark = false;    // so this function is not executed again
    g_landmarkMode = true;
    g_startMotor = true;
    
    clearStatusFlag(MOTORS_STOPPED);
    clearStatusFlag(PROXIMITY_ERROR);   // allow going to landmark even if we have proximity error
    
}

void setLED1(uint16_t onoff) {
    // input 1 for on, 0 for off
    LATDbits.LATD10 = onoff;
}

void setStatusFlag(StatusBit whichFlag) {
    // sets one bit of the status flag integer
    uint16_t bitPattern;
    
    bitPattern = 1 << whichFlag;
    g_statusFlags = g_statusFlags | bitPattern;
}

void clearStatusFlag(StatusBit whichFlag) {
    // clears one bit of the status flag integer
    uint16_t bitPattern;
    
    bitPattern = ~(1 << whichFlag);        // all bits are 1 except the whichFlag bit
    g_statusFlags = g_statusFlags & bitPattern;
}

bool getStatusFlag(StatusBit whichFlag) {
    // gets one bit of the status flag integer, returning a true if the bit is set, false otherwise
    uint16_t bitPattern;
    bool flagSet;
    
    bitPattern = (1 << whichFlag);        // all bits are 1 except the whichFlag bit
    flagSet = (bool)(g_statusFlags & bitPattern);
    
    return flagSet;
}