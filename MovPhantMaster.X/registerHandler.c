#include "registerHandler.h"
#include <xc.h>
#include "StateManagement.h"
#include "globals.h"
#include "Configure.h"
#include "paramManagerMaster.h"

//Definitions of hardware constants that will also be used from master device.
//These values must match the ones used in the master.

// I2C Register Constants
#define REG_COMMAND 1                       // register for issuing a command
#define REG_FIRMWARE_REV 2                  // firmware revision
#define REG_MOTION_AMPLITUDE_1 10           // amplitude (peak-to-peak) in mm of motor 1
#define REG_WAVEFORM_TYPE   11              // waveform type
#define REG_STEPS_PER_MM_1 12               // steps per mm motor 1 (HF)
#define REG_STEPS_PER_MM_2 13               // steps per mm motor 1 (HF)
#define REG_MAX_MOTION_AMP_1 14             // maximum allowable motion amplitude in motor 1
#define REG_FREQ 15                         // register for frequency
#define REG_REVERSE1 16                     // register to reverse motion direction motor 1
#define REG_PROP_NUM1 17                    // register to set numerator of proportional feedback constant motor 1
#define REG_PROP_DEN1 18                    // register to set denominator of proportional feedback constant motor 1
#define REG_INT_NUM1 19                     // register to set numerator of integral feedback constant motor 1
#define REG_INT_DEN1 20                     // register to set denominator of integral feedback constant motor 1
#define REG_DER_NUM1 21                     // register to set numerator of derivative feedback constant motor 1
#define REG_DER_DEN1 22                     // register to set denominator of derivative feedback constant motor 1
#define REG_PWM_POS1_OFFSET 23              // register to set pwm position offset motor 1
#define REG_PWM_VEL_OFFSET 24               // register to set pwm velocity offset motor 1 or motor 2
#define REG_PROP_NUM2 25                    // register to set numerator of proportional feedback constant motor 2
#define REG_PROP_DEN2 26                    // register to set denominator of proportional feedback constant motor 2
#define REG_INT_NUM2 27                     // register to set numerator of integral feedback constant motor 2
#define REG_INT_DEN2 28                     // register to set denominator of integral feedback constant motor 2
#define REG_DER_NUM2 29                     // register to set numerator of derivative feedback constant motor 2
#define REG_DER_DEN2 30                     // register to set denominator of derivative feedback constant motor 2
#define REG_PWM_POS2_OFFSET 31              // register to set pwm position offset motor 2
#define REG_MOTION_AMPLITUDE_2 32           // amplitude (peak-to-peak) in mm of motor 2
#define REG_REVERSE2 33                     // register to reverse motion direction motor 2
#define REG_MOT1_ENABLED 34                 // register to enable motor 1
#define REG_MOT2_ENABLED 35                 // register to enable motor 2
#define REG_STATUS_FLAGS 36                 // get the status flags byte
#define REG_REC_DATA 37                     // prepare for data transmission from CPU to MCU
#define REG_WHICH_WAVEFORM 38               // identifies which motor (1 or 2) the data being sent from primary to secondary belongs to
#define REG_DATA_VAL 39                     // data value transmission from CPU to MCU



// Value definitions


// Register commands. These are what gets passed into the dataVal field of setRegisterValue and determine which command is executed
#define START_MOTION  199                     // begin pulsing electric field
#define STOP_MOTION 2                       // stop pulsing electric field
//#define SET_ZERO_POSITION 3               // sets the current encoder position as zero
#define STEP_HEAD 4                         // step forward once
#define STEP_FOOT 5                         // step backward once
#define SET_LANDMARK 6                      // set landmark
#define GOTO_LANDMARK 7                     // goto landmark position
#define STEP_LEFT 8                         // step left once
#define STEP_RIGHT 9                        // step right once


// Fault States
#define FS_NOFAULT 0
#define FS_COIL_NOT_PULSING 1

uint16_t lastValueWritten;

void setRegisterValue(uint8_t regNum, uint16_t dataVal) {
// Sets a register value from an I2C command
    if(regNum == REG_COMMAND) { //Commands go here. What gets done depends upon the dataVal
        if(dataVal == START_MOTION) {
            g_stepMode = false;
            //setLED1(1);
            startMotion();             // in StateManagement.c
        }
        else if(dataVal == STOP_MOTION) {
            g_stopButtonPushed = true;
            //setLED1(0);
            stopMotion();              // in StateManagement.c
        }
        else if(dataVal == STEP_HEAD) {
            g_stepMode = true;
            g_waveformType = 1;
            g_motionAmplitudeMM1 = 5;
            g_motionAmplitudeMM2 = 0;
            g_userMotor1Enable = true;
            g_userMotor2Enable = false;
            g_reverseDirection1 = 1;
            g_freqUser = 80;
            startMotion();
        }
        else if(dataVal == STEP_FOOT) {
            g_stepMode = true;
            g_waveformType = 1;
            g_motionAmplitudeMM1 = 5;
            g_motionAmplitudeMM2 = 0;
            g_userMotor1Enable = true;
            g_userMotor2Enable = false;
            g_reverseDirection1 = 0;
            g_freqUser = 80;
            startMotion();
        }
        else if(dataVal == STEP_RIGHT) {
            g_stepMode = true;
            g_waveformType = 1;
            g_motionAmplitudeMM1 = 0;
            g_motionAmplitudeMM2 = 15;
            g_userMotor1Enable = false;
            g_userMotor2Enable = true;
            g_reverseDirection2 = 1;
            g_freqUser = 80;
            startMotion();
        }
        else if(dataVal == STEP_LEFT) {
            g_stepMode = true;
            g_waveformType = 1;
            g_motionAmplitudeMM1 = 0;
            g_motionAmplitudeMM2 = 15;
            g_userMotor1Enable = false;
            g_userMotor2Enable = true;
            g_reverseDirection2 = 0;
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
    }
    else if(regNum == REG_STEPS_PER_MM_1) {
        g_encoderStepsPerMM_1 = dataVal;
    }
    else if(regNum == REG_STEPS_PER_MM_2) {
        g_encoderStepsPerMM_2 = dataVal;
    }
    else if(regNum == REG_WAVEFORM_TYPE) {
        g_waveformType = dataVal;
    }
    else if(regNum == REG_MOTION_AMPLITUDE_1) {
        g_motionAmplitudeMM1 = dataVal;
    }
    else if(regNum == REG_MOTION_AMPLITUDE_2) {
        g_motionAmplitudeMM2 = dataVal;
    }
    else if(regNum == REG_FREQ) {
        g_freqUser = dataVal;
    }
    else if(regNum == REG_REVERSE1) {
        g_reverseDirection1 = dataVal;
    }
    else if(regNum == REG_REVERSE2) {
        g_reverseDirection2 = dataVal;
    }
    else if(regNum == REG_PROP_NUM1) {
        g_propConstNum1 = dataVal;
    }
    else if(regNum == REG_PROP_DEN1) {
        g_propConstDenom1 = dataVal;
    }
    else if(regNum == REG_INT_NUM1) {
        g_intConstNum1 = dataVal;
    }
    else if(regNum == REG_INT_DEN1) {
        g_intConstDenom1 = dataVal;
    }
    else if(regNum == REG_DER_NUM1) {
        g_derivConstNum1 = dataVal;
    }
    else if(regNum == REG_DER_DEN1) {
        g_derivConstDenom1 = dataVal;
    }
    else if(regNum == REG_PWM_POS1_OFFSET) {
        g_pwm1ZeroOffset = dataVal;
    }
    else if(regNum == REG_PWM_VEL_OFFSET) {
        g_pwm3ZeroOffset = dataVal;
    }
    else if(regNum == REG_PROP_NUM2) {
        g_propConstNum2 = dataVal;
    }
    else if(regNum == REG_PROP_DEN2) {
        g_propConstDenom2 = dataVal;
    }
    else if(regNum == REG_INT_NUM2) {
        g_intConstNum2 = dataVal;
    }
    else if(regNum == REG_INT_DEN2) {
        g_intConstDenom2 = dataVal;
    }
    else if(regNum == REG_DER_NUM2) {
        g_derivConstNum2 = dataVal;
    }
    else if(regNum == REG_DER_DEN2) {
        g_derivConstDenom2 = dataVal;
    }
    else if(regNum == REG_PWM_POS2_OFFSET) {
        g_pwm2ZeroOffset = dataVal;
    }
    else if(regNum == REG_MOT1_ENABLED) {
        g_userMotor1Enable = (bool)dataVal;
    }
    else if(regNum == REG_MOT2_ENABLED) {
        g_userMotor2Enable = (bool)dataVal;
    }
    else if(regNum == REG_STATUS_FLAGS) {
        g_statusFlags = dataVal;
    }
    else if(regNum == REG_REC_DATA) {
        IEC0bits.T1IE = 0;                  // Disable Timer1 interrupt temporarily 
        allocateWaveforms(dataVal);    // sends command to secondary to allocate waveforms with num points = dataVal
        IEC0bits.T1IE = 1;                  // Re-enable Timer1 interrupt
    }
    else if(regNum == REG_WHICH_WAVEFORM) {
        g_whichWaveform = dataVal;    // this should be done before REG_DATA_VAL block
    }
    else if(regNum == REG_DATA_VAL) {
        IEC0bits.T1IE = 0;                  // Disable Timer1 interrupt temporarily 
        sendDataValToSecondary(dataVal);
        IEC0bits.T1IE = 1;                  // Re-enable Timer1 interrupt
    }
    

    lastValueWritten = dataVal;     // scope is within whole file
    
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
        val = g_encoderStepsPerMM_1;
    }
    else if (regNum == REG_STEPS_PER_MM_2) {
        val = g_encoderStepsPerMM_2;
    }
    else if(regNum == REG_WAVEFORM_TYPE) {
        val = g_waveformType;
    }
    else if (regNum == REG_MOTION_AMPLITUDE_1) {
        val = g_motionAmplitudeMM1;
    }
    else if (regNum == REG_MOTION_AMPLITUDE_2) {
        val = g_motionAmplitudeMM2;
    }
    else if(regNum == REG_FREQ) {
        val = g_freqUser;
    }
    else if (regNum == REG_REVERSE1) {
        val = g_reverseDirection1;
    }
    else if (regNum == REG_REVERSE2) {
        val = g_reverseDirection2;
    }
    else if(regNum == REG_PROP_NUM1) {
        val = g_propConstNum1;
    }
    else if(regNum == REG_PROP_DEN1) {
        val = g_propConstDenom1;
    }
    else if(regNum == REG_INT_NUM1) {
        val = g_intConstNum1;
    }
    else if(regNum == REG_INT_DEN1) {
        val = g_intConstDenom1;
    }
    else if(regNum == REG_DER_NUM1) {
        val = g_derivConstNum1;
    }
    else if(regNum == REG_DER_DEN1) {
        val = g_derivConstDenom1;
    }
    else if(regNum == REG_PWM_POS1_OFFSET) {
        val = g_pwm1ZeroOffset;
    }
    else if(regNum == REG_PWM_VEL_OFFSET) {
        val = g_pwm3ZeroOffset;
    }
    else if(regNum == REG_PROP_NUM2) {
        val = g_propConstNum2;
    }
    else if(regNum == REG_PROP_DEN2) {
        val = g_propConstDenom2;
    }
    else if(regNum == REG_INT_NUM2) {
        val = g_intConstNum2;
    }
    else if(regNum == REG_INT_DEN2) {
        val = g_intConstDenom2;
    }
    else if(regNum == REG_DER_NUM2) {
        val = g_derivConstNum2;
    }
    else if(regNum == REG_DER_DEN2) {
        val = g_derivConstDenom2;
    }
    else if(regNum == REG_PWM_POS2_OFFSET) {
        val = g_pwm2ZeroOffset;
    }
    else if(regNum == REG_MOT1_ENABLED) {
        val = g_userMotor1Enable;
    }
    else if(regNum == REG_MOT2_ENABLED) {
        val = g_userMotor2Enable;
    }
    else if(regNum == REG_STATUS_FLAGS) {
        val = g_statusFlags;
    }
    else if(regNum == REG_REC_DATA) {
        val = lastValueWritten;   // assuming here we never use this for anything other than verifying transmission
    }
    else if(regNum == REG_WHICH_WAVEFORM) {
        val = g_whichWaveform;
    }
    else if(regNum == REG_DATA_VAL) {
        val = lastValueWritten;   // assuming here we never use this for anything other than verifying transmission
    }
    
    return val;
}