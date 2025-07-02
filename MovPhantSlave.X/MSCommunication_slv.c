#include <xc.h>
#include <libpic30.h>
#include <stdlib.h>
#include <stdbool.h>
#include "globals_slave.h"
#include "MSCommunication_slv.h"
#include "enums_slave.h"
#include "StateManagement_slv.h"


void processCommandFromPrimary() {
    Command fifoCmd;
    while(SI1FIFOCSbits.SRFEMPTY);          // wait for next FIFO data to come through
        fifoCmd = SRMWFDATA;
        if(fifoCmd == START_MOTION) {
           startMotion();
        }
        else if (fifoCmd == STOP_MOTION) {
            stopMotion();
        }
}

void receiveVariableFromPrimary() {
    // Reads 16-bit variable from Master-Slave Fifo and sets the appropriate secondary core variable
    Variable whichVar;
    uint16_t fifoVal;
     
    // First entry in FIFO is a Variable type that identifies which variable is being sent.
    while(SI1FIFOCSbits.SRFEMPTY);          // wait for next FIFO data to come through
    whichVar = SRMWFDATA;                   // which variable
    
    // Second entry is the value of the variable to set
    while(SI1FIFOCSbits.SRFEMPTY);          // wait for next FIFO data to come through
    fifoVal = SRMWFDATA;                    // value in the variable data.
    
    switch(whichVar) {
        case PWM1_CYCLES:
            gs_pwm1Cycles = (int16_t)fifoVal;
            break;
        case MAX_PWM_INT:
            gs_maxPWMInteger = fifoVal;
            break;
        case NUM_ARRAY_VALS:
            gs_numArrayVals = fifoVal;
            break;
        case WAVEFORM_UPDATE_PERIOD:
            gs_waveformUpdatePeriod = fifoVal;
            break;
        case WAVEFORM_TYPE:
            gs_waveformType = fifoVal;
            break;
        case MOTION_AMPLITUDE_MM:
            gs_motionAmplitudeMM = fifoVal;
            break;
        case REVERSE_DIRECTION:
            gs_reverseDirection = fifoVal;
            break;
        case FREQ_USER:
            gs_freqUser = fifoVal;
            break;
        case ENCODER_STEPS_PER_MM:
            gs_encoderStepsPerMM = fifoVal;
            break;
        default:
            break;
    }
}

void receive32bVariableFromPrimary() {
    // Reads 32-bit variable from Master-Slave Fifo and sets the appropriate secondary core variable
    Variable32 whichVar;
    uint16_t fifoVal;
    uint32_t var32;
     
    // First entry in FIFO is a Variable type that identifies which variable is being sent.
    while(SI1FIFOCSbits.SRFEMPTY);          // wait for next FIFO data to come through
    whichVar = SRMWFDATA;                   // which variable
    
    // Read two registers from the FIFO and construct the 32-bit integer
    //...most significant bits first
    while(SI1FIFOCSbits.SRFEMPTY);          // wait for next FIFO data to come through
    fifoVal = SRMWFDATA;                    // value in the variable data.
    var32 = ((uint32_t)fifoVal) << 16;
    
    // ...least significant bits
    while(SI1FIFOCSbits.SRFEMPTY);          // wait for next FIFO data to come through
    fifoVal = SRMWFDATA;                    // value in the variable data.
    var32 = var32 | ((uint32_t)fifoVal);
    
    switch(whichVar) {
        case WAVEFORM_TIMESTEP_MICROS:
            gs_waveformTimeStep_microS = var32;
            break;
        case DISPLACEMENT1_DEMAND:
            gs_displacement1Demand = (int32_t)var32;
            break;
        case DISPLACEMENT2_DEMAND:
            gs_displacement2Demand = (int32_t)var32;
            break;
    }
}

void receiveBoolVarFromPrimary() {
    // Reads boolean variable from Master-Slave Fifo and sets the appropriate secondary core variable
    BoolVariable whichVar;
    bool fifoVal;
     
    // First entry in FIFO is a Variable type that identifies which variable is being sent.
    while(SI1FIFOCSbits.SRFEMPTY);          // wait for next FIFO data to come through
    whichVar = SRMWFDATA;                   // which variable
    
    // Second entry is the value of the variable to set
    while(SI1FIFOCSbits.SRFEMPTY);          // wait for next FIFO data to come through
    fifoVal = (bool)SRMWFDATA;                    // value in the variable data.
    
    switch(whichVar) {
        case OUTPUT1_ENABLED:
            gs_output1Enabled = fifoVal;
            break;
        case OUTPUT2_ENABLED:
            gs_output2Enabled = fifoVal;
            break;
        default:
            break;
    }
}

void sendVariableToPrimary(Variable whichVar, uint16_t value) {
    // Sends a single 16-bit variable to the primary through the MS FIFO
    
    // Send register that identifies it as a 16-bit variable
    while(SI1FIFOCSbits.SWFFULL);       //wait until write FIFO is not full
    SWMRFDATA = REG_VARIABLE;
    
    while(SI1FIFOCSbits.SWFFULL);       //wait until write FIFO is not full
    SWMRFDATA = whichVar;
    
    while(SI1FIFOCSbits.SWFFULL);       //wait until write FIFO is not full
    SWMRFDATA = value;
}

void send32bVariableToPrimary(Variable32 whichVar, uint32_t value) {
    // Sends a single 32-bit variable to the secondary through the MS FIFO
    
    // extract the msb and lsb
    uint16_t lsb;
    uint16_t msb;    
    
    msb = (uint16_t)((value & 0xFFFF0000) >> 16);     //most significant bits
    lsb = (uint16_t)(value & 0x0000FFFF);           //least significant bits
    
    // Send register that identifies it as a 32-bit variable
    while(SI1FIFOCSbits.SWFFULL);                   //wait until write FIFO is not full
    SWMRFDATA = REG_VARIABLE_32;
    
    while(SI1FIFOCSbits.SWFFULL);                   //wait until write FIFO is not full
    SWMRFDATA = whichVar;
    
    // Transmit most significant bits
    while(SI1FIFOCSbits.SWFFULL);                   //wait until write FIFO is not full
    SWMRFDATA = msb;
    
    // Transmit least significant bits
    while(SI1FIFOCSbits.SWFFULL);                   //wait until write FIFO is not full
    SWMRFDATA = lsb;
}

void sendBoolVarToPrimary(Variable whichVar, bool value) {
    // Sends a single boolean variable to the primary through the MS FIFO
    
    // Send register that identifies it as a 16-bit variable
    while(SI1FIFOCSbits.SWFFULL);       //wait until write FIFO is not full
    SWMRFDATA = REG_BOOLVAR;
    
    while(SI1FIFOCSbits.SWFFULL);       //wait until write FIFO is not full
    SWMRFDATA = whichVar;
    
    while(SI1FIFOCSbits.SWFFULL);       //wait until write FIFO is not full
    SWMRFDATA = (uint16_t)value;
}