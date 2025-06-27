#include <xc.h>
#include <libpic30.h>
#include <stdbool.h>
#include "MSCommunication_mstr.h"
#include "globals.h"
#include "enums.h"


void sendCommandToSecondary(Command whichCommand) {
   // Sneds a command to the secondary through the MS FIFO
    
    // Send register that identifies it as a command.
    while(MSI1FIFOCSbits.WFFULL);       //wait until write FIFO is not full
    MWSRFDATA = REG_COMMAND;
    
    while(MSI1FIFOCSbits.WFFULL);       //wait until write FIFO is not full
    MWSRFDATA = whichCommand;
}

void sendVariableToSecondary(Variable whichVar, uint16_t value) {
    // Sends a single 16-bit variable to the secondary through the MS FIFO
 //   LATBbits.LATB2 = 1;
    // Send register that identifies it as a 16-bit variable
    while(MSI1FIFOCSbits.WFFULL);       //wait until write FIFO is not full
    MWSRFDATA = REG_VARIABLE;
    
    while(MSI1FIFOCSbits.WFFULL);       //wait until write FIFO is not full
    MWSRFDATA = whichVar;
    
    while(MSI1FIFOCSbits.WFFULL);       //wait until write FIFO is not full
    MWSRFDATA = value;
 //   LATBbits.LATB2 = 0;
}

void send32bVariableToSecondary(Variable32 whichVar, uint32_t value) {
    // Sends a single 32-bit variable to the secondary through the MS FIFO
    
    // extract the msb and lsb
    uint16_t lsb;
    uint16_t msb;    
    
    msb = (uint16_t)((value & 0xFFFF0000) >> 16);     //most significant bits
    lsb = (uint16_t)(value & 0x0000FFFF);           //least significant bits
    
    // Send register that identifies it as a 32-bit variable
    while(MSI1FIFOCSbits.WFFULL);                   //wait until write FIFO is not full
    MWSRFDATA = REG_VARIABLE_32;
    
    while(MSI1FIFOCSbits.WFFULL);                   //wait until write FIFO is not full
    MWSRFDATA = whichVar;
    
    // Transmit most significant bits
    while(MSI1FIFOCSbits.WFFULL);                   //wait until write FIFO is not full
    MWSRFDATA = msb;
    
    // Transmit least significant bits
    while(MSI1FIFOCSbits.WFFULL);                   //wait until write FIFO is not full
    MWSRFDATA = lsb;
}

void sendBoolVarToSecondary(BoolVariable whichVar, bool boolVal) {
    // Sends a single boolean variable to the secondary through the MS FIFO
    
    // Send register that identifies it as a 16-bit variable
    while(MSI1FIFOCSbits.WFFULL);       //wait until write FIFO is not full
    MWSRFDATA = REG_BOOLVAR;
    
    while(MSI1FIFOCSbits.WFFULL);       //wait until write FIFO is not full
    MWSRFDATA = whichVar;
    
    while(MSI1FIFOCSbits.WFFULL);       //wait until write FIFO is not full
    MWSRFDATA = (uint16_t)boolVal;
}

void sendParamtersToSecondary() {
    // Sends global parameters to secondary core.
    
    // note: __delay32(g_OscillatorFreq) would give a 2-second delay
    
    sendVariableToSecondary(MAX_PWM_INT, g_maxPWMInteger);
    __delay32(g_OscillatorFreq / 40000);  // 50 microsecond delay  
    
//    if(g_numArrayVals == 0) {
//        LATBbits.LATB2 = 1;
//    }
    sendVariableToSecondary(NUM_ARRAY_VALS, g_numArrayVals);
    __delay32(g_OscillatorFreq / 40000);  
    
    sendVariableToSecondary(WAVEFORM_UPDATE_PERIOD, g_waveformUpdatePeriod);
    __delay32(g_OscillatorFreq / 40000);  
    
    sendVariableToSecondary(WAVEFORM_TYPE, g_waveformType);
    __delay32(g_OscillatorFreq / 40000);
    
    sendVariableToSecondary(MOTION_AMPLITUDE_MM, g_motionAmplitudeMM);
    __delay32(g_OscillatorFreq / 40000);
    
    sendVariableToSecondary(REVERSE_DIRECTION, g_reverseDirection);
    __delay32(g_OscillatorFreq / 40000);
    
    send32bVariableToSecondary(WAVEFORM_TIMESTEP_MICROS, g_waveformTimeStep_microS);
    __delay32(g_OscillatorFreq / 40000);
    
    sendVariableToSecondary(FREQ_USER, g_freqUser);
    __delay32(g_OscillatorFreq / 40000);
    
    sendVariableToSecondary(ENCODER_STEPS_PER_MM, g_encoderStepsPerMM);
    __delay32(g_OscillatorFreq / 40000);
}

void receiveVariableFromSecondary() {
    // Reads 16-bit variable from Master-Slave Fifo and sets the appropriate primary core variable
    Variable whichVar;
    uint16_t fifoVal;
    
   // LATBbits.LATB2 = 1;
    // First entry in FIFO is a Variable type that identifies which variable is being sent.
    while(MSI1FIFOCSbits.RFEMPTY);          // wait for next FIFO data to come through
    whichVar = MRSWFDATA;                   // which variable
    
    // Second entry is the value of the variable to set
    while(MSI1FIFOCSbits.RFEMPTY);          // wait for next FIFO data to come through
    fifoVal = MRSWFDATA;                    // value in the variable data.
    
    switch(whichVar) {
        case PWM1_CYCLES:            // Dummy test - may not need anything yet
            break;
        default:
            break;
    }
}

void receive32bVariableFromSecondary() {
    // Reads 32-bit variable from Master-Slave Fifo and sets the appropriate primary core variable
    Variable32 whichVar;
    uint16_t fifoVal;
    uint32_t var32;
     
   // LATBbits.LATB2 = 1;
    // First entry in FIFO is a Variable type that identifies which variable is being sent.
    while(MSI1FIFOCSbits.RFEMPTY);          // wait for next FIFO data to come through
    whichVar = MRSWFDATA;                    // which variable
    
    // Read two registers from the FIFO and construct the 32-bit integer
    //...most significant bits first
    while(MSI1FIFOCSbits.RFEMPTY);          // wait for next FIFO data to come through
    fifoVal = MRSWFDATA;                    // value in the variable data.
    var32 = ((uint32_t)fifoVal) << 16;
    
    // ...least significant bits
    while(MSI1FIFOCSbits.RFEMPTY);          // wait for next FIFO data to come through
    fifoVal = MRSWFDATA;                    // value in the variable data.
    var32 = var32 | ((uint32_t)fifoVal);
    
    
    switch(whichVar) {
        case DISPLACEMENT_DEMAND:
            g_displacementDemand = (int32_t)var32;
//            if (g_displacementDemand > 0x00010000) {
//                LATBbits.LATB2 = 1;
//            }
//            else {
//                LATBbits.LATB2 = 0;
//            }
            break;
        case WAVEFORM_TIMESTEP_MICROS:
            g_waveformTimeStep_microS = var32;
//            if (g_waveformTimeStep_microS == 0x0F000010) {
//                LATBbits.LATB2 = 1;
//            }
//            else if (g_waveformTimeStep_microS == 7000){
//                LATBbits.LATB2 = 0;
//            }
            break;
    }
}

void receiveBoolVarFromSecondary() {
    // Reads 16-bit variable from Master-Slave Fifo and sets the appropriate primary core variable
    BoolVariable whichVar;
    bool fifoVal;
     
    // First entry in FIFO is a Variable type that identifies which variable is being sent.
    while(MSI1FIFOCSbits.RFEMPTY);          // wait for next FIFO data to come through
    whichVar = MRSWFDATA;                   // which variable
    
    // Second entry is the value of the variable to set
    while(MSI1FIFOCSbits.RFEMPTY);          // wait for next FIFO data to come through
    fifoVal = (bool)MRSWFDATA;                    // value in the variable data.
    
    switch(whichVar) {
        case OUTPUT_ENABLED:                // Dummy test - may not need anything yet
            g_outputEnabled = fifoVal;
//            if(g_outputEnabled) {
//                LATBbits.LATB2 = 1;
//            }
//            else {
//                LATBbits.LATB2 = 0;
//            }
        default:
            break;
    }
}

void enableMSFifo() {
    MSI1FIFOCSbits.WFEN = 1;            // enable MSI Master-write FIFO
    MSI1FIFOCSbits.RFEN = 1;            // enable MSI Master-read FIFO
}



