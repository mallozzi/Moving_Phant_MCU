#include <xc.h>
#include <libpic30.h>
#include <stdlib.h>
#include <stdbool.h>
#include "globals_slave.h"
#include "StateManagement_slv.h"
#include "config_slave.h"
#include "enums_slave.h"
#include "MSCommunication_slv.h"



void startMotion() {
    
    // Note: parameters from master must be transmitted before calling this function
    //setLED2(1);
    setUpWaveforms();
    gs_stopMotors = false;
    gs_resetWaveform = true;
    gs_displacement1Demand=0;
    gs_displacement2Demand=0;
    while(SI1FIFOCSbits.SWFFULL);  // wait until write FIFO is not full
    gs_output1Enabled = gs_userMotor1Enable;
    gs_output2Enabled = gs_userMotor2Enable;
    sendBoolVarToPrimary(OUTPUT1_ENABLED, gs_output1Enabled);
    sendBoolVarToPrimary(OUTPUT2_ENABLED, gs_output2Enabled);
}

void stopMotion() {
    // The purpose of this is to have a short function to call from an I2C command
    gs_stopMotors = true;
    
}

void setLED2(uint16_t onoff) {
    LATBbits.LATB1 = onoff;
}

//Diagnostic output pins. Must be set to secondary core ownership to use

void setDiag1_slv(uint16_t onoff) {
    // Set diagnostic pin 1. onoff = 1 for on, 0 for off. Pin must be set to be owned by secondary core.
    LATAbits.LATA4 = onoff;
}

void setFaultPin_slv(uint16_t onoff) {
    // Set fault pin. onoff = 1 for on, 0 for off. Pin must be set to be owned by secondary core.
    LATBbits.LATB0 = onoff;
}