#include <xc.h>
#include <libpic30.h>
#include <stdbool.h>
#include "MSCommunication_mstr.h"
#include "globals.h"
#include "enums.h"
#include "motorCntrlMstr.h"
#include "StateManagement.h"


void sendCommandToSecondary(Command whichCommand) {
    // Sends a command to the secondary through the MS FIFO. It is important not to
    // allow FIFO writes to get interrupted or else the data stream can be corrupted
    
    IEC0bits.T1IE = 0;                  // Disable Timer1 interrupt temporarily
            
    // Send register that identifies it as a command.
    while(MSI1FIFOCSbits.WFFULL);       //wait until write FIFO is not full
    MWSRFDATA = REG_COMMAND;
    
    while(MSI1FIFOCSbits.WFFULL);       //wait until write FIFO is not full
    MWSRFDATA = whichCommand;
    
    IEC0bits.T1IE = 1;                  // Re-enable Timer1 interrupt
}

void sendVariableToSecondary(Variable whichVar, uint16_t value) {
    // Sends a single 16-bit variable to the secondary through the MS FIFO
    
    IEC0bits.T1IE = 0;                  // Disable Timer1 interrupt temporarily 
    
    // ...Send register that identifies it as a 16-bit variable
    while(MSI1FIFOCSbits.WFFULL);       //wait until write FIFO is not full
    MWSRFDATA = REG_VARIABLE;
    
    while(MSI1FIFOCSbits.WFFULL);       //wait until write FIFO is not full
    MWSRFDATA = whichVar;
    
    while(MSI1FIFOCSbits.WFFULL);       //wait until write FIFO is not full
    MWSRFDATA = value;
    
    IEC0bits.T1IE = 1;                  // Re-enable Timer1 interrupt

}

void send32bVariableToSecondary(Variable32 whichVar, uint32_t value) {
    // Sends a single 32-bit variable to the secondary through the MS FIFO
    
    // extract the msb and lsb
    uint16_t lsb;
    uint16_t msb;    
    
    msb = (uint16_t)((value & 0xFFFF0000) >> 16);     //most significant bits
    lsb = (uint16_t)(value & 0x0000FFFF);           //least significant bits
    
    IEC0bits.T1IE = 0;                              // Disable Timer1 interrupt temporarily 
    
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
    
    IEC0bits.T1IE = 1;                              // Re-enable Timer1 interrupt
    
}

void sendBoolVarToSecondary(BoolVariable whichVar, bool boolVal) {
    // Sends a single boolean variable to the secondary through the MS FIFO
     
    IEC0bits.T1IE = 0;                  // Disable Timer1 interrupt temporarily 
    
    // Send register that identifies it as a 16-bit variable
    while(MSI1FIFOCSbits.WFFULL);       //wait until write FIFO is not full
    MWSRFDATA = REG_BOOLVAR;
    
    while(MSI1FIFOCSbits.WFFULL);       //wait until write FIFO is not full
    MWSRFDATA = whichVar;
    
    while(MSI1FIFOCSbits.WFFULL);       //wait until write FIFO is not full
    MWSRFDATA = (uint16_t)boolVal;
    
    IEC0bits.T1IE = 1;                  // Re-enable Timer1 interrupt
    
}

void sendPingRequestToSecondary(uint16_t pingVal) {
    // Sends the value pingVal to the secondary. The protocol is that the ping request
    // includes the value pingVal, an arbitrary integer to identify the ping request. The
    // secondary reads this value and sends it back as a single-byte ping variable. The 
    // primary receives that variable and sets the local variable g_pingVal with this value
    // which can be checked against the transmitted pingVal to match, verifying the ping.
    // The variable g_pingVal should be set to 0 before calling this function.
    // This ping is used to ensure that an earlier FIFO transmission has occurred.
    
    IEC0bits.T1IE = 0;                  // Disable Timer1 interrupt temporarily 
    
    // ...Send register that identifies it as a ping variable
    while(MSI1FIFOCSbits.WFFULL);       //wait until write FIFO is not full
    MWSRFDATA = REG_PING;
    
    while(MSI1FIFOCSbits.WFFULL);       //wait until write FIFO is not full
    MWSRFDATA = pingVal;
    
    IEC0bits.T1IE = 1;                  // Re-enable Timer1 interrupt
    
}

void sendParamtersToSecondary() {
    // Sends global parameters to secondary core.
    
    // note: __delay32(g_OscillatorFreq) would give a 2-second delay
    
    sendVariableToSecondary(MAX_PWM_INT, g_maxPWMInteger);
    __delay32(g_OscillatorFreq / 40000);  // 50 microsecond delay  
    
    sendVariableToSecondary(NUM_ARRAY_VALS, g_numArrayVals);
    __delay32(g_OscillatorFreq / 40000);  
    
    sendVariableToSecondary(WAVEFORM_UPDATE_PERIOD, g_waveformUpdatePeriod);
    __delay32(g_OscillatorFreq / 40000);  
    
    sendVariableToSecondary(WAVEFORM_TYPE, g_waveformType);
    __delay32(g_OscillatorFreq / 40000);
    
    sendVariableToSecondary(MOTION_AMPLITUDE_MM1, g_motionAmplitudeMM1);
    __delay32(g_OscillatorFreq / 40000);
    
    sendVariableToSecondary(MOTION_AMPLITUDE_MM2, g_motionAmplitudeMM2);
    __delay32(g_OscillatorFreq / 40000);
    
    sendVariableToSecondary(REVERSE_DIRECTION1, g_reverseDirection1);
    __delay32(g_OscillatorFreq / 40000);
    
    sendVariableToSecondary(REVERSE_DIRECTION2, g_reverseDirection2);
    __delay32(g_OscillatorFreq / 40000);
    
    send32bVariableToSecondary(WAVEFORM_TIMESTEP_MICROS, g_waveformTimeStep_microS);
    __delay32(g_OscillatorFreq / 40000);
    
    sendVariableToSecondary(FREQ_USER, g_freqUser);
    __delay32(g_OscillatorFreq / 40000);
    
    sendVariableToSecondary(ENCODER_STEPS_PER_MM, g_encoderStepsPerMM);
    __delay32(g_OscillatorFreq / 40000);
    
    sendBoolVarToSecondary(USER_MOT1_ENABLED, g_userMotor1Enable);
    __delay32(g_OscillatorFreq / 40000);
    
    sendBoolVarToSecondary(USER_MOT2_ENABLED, g_userMotor2Enable);
    __delay32(g_OscillatorFreq / 40000);
    
}

void receiveVariableFromSecondary() {
    // Reads 16-bit variable from Master-Slave Fifo and sets the appropriate primary core variable
    Variable whichVar;
    uint16_t fifoVal;
       
    // First entry in FIFO is a Variable type that identifies which variable is being sent.
    while(MSI1FIFOCSbits.RFEMPTY);          // wait for next FIFO data to come through
    whichVar = MRSWFDATA;                   // which variable
    
    // Second entry is the value of the variable to set
    while(MSI1FIFOCSbits.RFEMPTY);          // wait for next FIFO data to come through
    fifoVal = MRSWFDATA;                    // value in the variable data.
    
    switch(whichVar) {
        case PING_VAR:                      // return value from a master-requested ping to secondary
            g_pingVar = (uint16_t)fifoVal;  // g_pingVar can be tested to make sure it matches what was sent
            break;
        case QUAD_ENC_VEL:                      // return value from a master-requested ping to secondary
            g_secondaryQuadEncVel = (int16_t)fifoVal;  // g_pingVar can be tested to make sure it matches what was sent
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
    
    // disable timer1 interrupt
    IEC0bits.T1IE = 0;
    
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
        case DISPLACEMENT1_DEMAND:
            g_displacement1Demand = (int32_t)var32;
            break;
        case DISPLACEMENT2_DEMAND:
            g_displacement2Demand = (int32_t)var32;
            break;
        case WAVEFORM_TIMESTEP_MICROS:
            g_waveformTimeStep_microS = var32;
            break;
        case QUAD_ENC_POS:
            g_secondaryQuadEncPos = var32;
            break;
    }
    
    // re-enable timer1 interrupt
    IEC0bits.T1IE = 1;
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
    fifoVal = (bool)MRSWFDATA;              // value in the variable data.
    
    switch(whichVar) {
        case OUTPUT1_ENABLED:                
            g_output1Enabled = fifoVal;
        case OUTPUT2_ENABLED:                
            g_output2Enabled = fifoVal;
        default:
            break;
    }
}

void enableMSFifo() {
    MSI1FIFOCSbits.WFEN = 1;            // enable MSI Master-write FIFO
    MSI1FIFOCSbits.RFEN = 1;            // enable MSI Master-read FIFO
}



