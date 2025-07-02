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
    setUpWaveform();
    gs_zeroPosOutput = false;
    gs_resetWaveform = true;
    gs_displacement1Demand=0;
    gs_displacement2Demand=0;
    while(SI1FIFOCSbits.SWFFULL);  // wait until write FIFO is not full
    sendBoolVarToPrimary(OUTPUT1_ENABLED, true);
    sendBoolVarToPrimary(OUTPUT2_ENABLED, true);
    gs_output1Enabled=true;
    gs_output2Enabled=true;
}

void stopMotion() {
    // The purpose of this is to have a short function to call from an I2C command
    gs_zeroPosOutput = true;
    
}