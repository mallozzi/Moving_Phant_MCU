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
  //  LATBbits.LATB1 = 1;
    setUpWaveform();
    gs_zeroPosOutput = false;
    gs_resetWaveform = true;
    gs_displacementDemand=0;
    while(SI1FIFOCSbits.SWFFULL);  // wait until write FIFO is not full
    sendBoolVarToPrimary(OUTPUT_ENABLED, true);
    gs_outputEnabled=true;
  //  LATBbits.LATB1 = 0;
}

void stopMotion() {
    // The purpose of this is to have a short function to call from an I2C command
//    g_outputWaveform = g_zeroWaveform;
//    gs_stopMotionIssued = true;
    gs_zeroPosOutput = true;
    LATBbits.LATB1 = 0;
 //   g_velocityDemand = 0;
    
}