#include <xc.h>
#include "globals.h"
#include "paramManagerMaster.h"
#include "enums.h"
#include "MSCommunication_mstr.h"
#include "StateManagement.h"


// General Parameter management
void sendDataValToSecondary(uint16_t val) {
    // Sends one 32-bit integer with a waveform data point. 
    if(g_whichWaveform == 1) {
        sendVariableToSecondary(WAVEFORM_DATA_VAL_1, val);  // motor 1
    }
    else {
        sendVariableToSecondary(WAVEFORM_DATA_VAL_2, val);  // motor 2
    }
}

void allocateWaveforms(uint16_t nPts) {
    sendVariableToSecondary(WAVEFORM_NPTS, nPts);
}
