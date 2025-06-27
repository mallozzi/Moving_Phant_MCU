#include "registerHandler.h"
#include <xc.h>
#include "StateManagement.h"
#include "globals.h"
#include "Configure.h"

//Definitions of hardware constants that will also be used from master device.
//These values must match the ones used in the master.

// I2C Register Constants
#define REG_COMMAND 1                       // register for issuing a command
#define REG_FIRMWARE_REV 2                  // firmware revision
#define REG_MOTION_AMPLITUDE_1 10           // amplitude (peak-to-peak) in mm of motor 1
#define REG_WAVEFORM_TYPE   11              // waveform type
#define REG_STEPS_PER_MM_1 12               // steps per mm
#define REG_MAX_MOTION_AMP_1 13             // maximum allowable motion amplitude in motor 1
#define REG_FREQ 14                         // register for frequency
#define REG_REVERSE 15                      // register to reverse frequency
#define REG_PROP_NUM 16                     // register to set numerator of proportional feedback constant
#define REG_PROP_DEN 17                     // register to set denominator of proportional feedback constant
#define REG_INT_NUM 18                      // register to set numerator of integral feedback constant
#define REG_INT_DEN 19                      // register to set denominator of integral feedback constant
#define REG_DER_NUM 20                      // register to set numerator of derivative feedback constant
#define REG_DER_DEN 21                      // register to set denominator of derivative feedback constant
#define REG_PWM_POS_OFFSET 22               // register to set pwm position offset
#define REG_PWM_VEL_OFFSET 23               // register to set pwm velocity offset

// Value definitions


// Register commands. These are what gets passed into the dataVal field of setRegisterValue and determine which command is executed
#define START_MOTION  1        // begin pulsing electric field
#define STOP_MOTION 2          // stop pulsing electric field
//#define SET_ZERO_POSITION 3    // sets the current encoder position as zero
#define STEP_FORWARD 4         // step forward once
#define STEP_BACKWARD 5        // step backward once
#define SET_LANDMARK 6         // set landmark
#define GOTO_LANDMARK 7        // goto landmark position

// Fault States
#define FS_NOFAULT 0
#define FS_COIL_NOT_PULSING 1

uint16_t lastValueWritten;

void setRegisterValue(uint8_t regNum, uint16_t dataVal) {
// Sets a register value from an I2C command
    if(regNum == REG_COMMAND) { //Commands go here. What gets done depends upon the dataVal
        if(dataVal == START_MOTION) {
            startMotion();             // in StateManagement.c
        }
        else if(dataVal == STOP_MOTION) {
            stopMotion();              // in StateManagement.c
        }
        else if(dataVal == STEP_FORWARD) {
            g_waveformType = 1;
            g_motionAmplitudeMM = 5;
            g_reverseDirection = 0;
            g_freqUser = 80;
            startMotion();
        }
        else if(dataVal == STEP_BACKWARD) {
            g_waveformType = 1;
            g_motionAmplitudeMM = 5;
            g_reverseDirection = 1;
            g_freqUser = 80;
            startMotion();
        }
        else if(dataVal == SET_LANDMARK) {
            setLandmarkPosition();
        }
        else if(dataVal == GOTO_LANDMARK) {
            g_gotoLandmark = true;
        }
    }
    // The else if statements are for passing data to a function
    else if(regNum == REG_MAX_MOTION_AMP_1) { // amplitude in mm of motor 1
        g_maxDisplacementMM = dataVal;  
        configureDerivedQuantities();
    }
    else if(regNum == REG_STEPS_PER_MM_1) {
        g_encoderStepsPerMM = dataVal;
        configureDerivedQuantities();
    }
    else if(regNum == REG_WAVEFORM_TYPE) {
        g_waveformType = dataVal;
    }
    else if(regNum == REG_MOTION_AMPLITUDE_1) {
        g_motionAmplitudeMM = dataVal;
    }
    else if(regNum == REG_FREQ) {
        g_freqUser = dataVal;
        configureDerivedQuantities();
    }
    else if(regNum == REG_REVERSE) {
        g_reverseDirection = dataVal;
    }
    else if(regNum == REG_PROP_NUM) {
        g_propConstNum = dataVal;
    }
    else if(regNum == REG_PROP_DEN) {
        g_propConstDenom = dataVal;
    }
    else if(regNum == REG_INT_NUM) {
        g_intConstNum = dataVal;
    }
    else if(regNum == REG_INT_DEN) {
        g_intConstDenom = dataVal;
    }
    else if(regNum == REG_DER_NUM) {
        g_derivConstNum = dataVal;
    }
    else if(regNum == REG_DER_DEN) {
        g_derivConstDenom = dataVal;
    }
    else if(regNum == REG_PWM_POS_OFFSET) {
        g_pwm2ZeroOffset = dataVal;
    }
    else if(regNum == REG_PWM_VEL_OFFSET) {
        g_pwm3ZeroOffset = dataVal;
    }
    
    
    lastValueWritten = dataVal;
    
}

uint16_t getRegisterValue(uint8_t regNum) {
// Retrieves a register value from an I2C request
    uint16_t val = 65535; //indicates unrecognized register
    
    if(regNum == REG_COMMAND) {  // for commands, there is no register to read, so just return the last command sent
        val = lastValueWritten;
    }
    else if (regNum == REG_FIRMWARE_REV) {
        val = g_firmwareRev;
    }
    else if(regNum == REG_MAX_MOTION_AMP_1) {
        val = g_maxDisplacementMM;
    }
    else if (regNum == REG_STEPS_PER_MM_1) {
        val = g_encoderStepsPerMM;
    }
    else if(regNum == REG_WAVEFORM_TYPE) {
        val = g_waveformType;
    }
    else if (regNum == REG_MOTION_AMPLITUDE_1) {
        val = g_motionAmplitudeMM;
    }
    else if(regNum == REG_FREQ) {
        val = g_freqUser;
    }
    else if (regNum == REG_REVERSE) {
        val = g_reverseDirection;
    }
    else if(regNum == REG_PROP_NUM) {
        val = g_propConstNum;
    }
    else if(regNum == REG_PROP_DEN) {
        val = g_propConstDenom;
    }
    else if(regNum == REG_INT_NUM) {
        val = g_intConstNum;
    }
    else if(regNum == REG_INT_DEN) {
        val = g_intConstDenom;
    }
    else if(regNum == REG_DER_NUM) {
        val = g_derivConstNum;
    }
    else if(regNum == REG_DER_DEN) {
        val = g_derivConstDenom;
    }
    else if(regNum == REG_PWM_POS_OFFSET) {
        val = g_pwm2ZeroOffset;
    }
    else if(regNum == REG_PWM_VEL_OFFSET) {
        val = g_pwm3ZeroOffset;
    }
    
    return val;
}