/*
 * File:   main_MovPhantSlav.c
 * Author: richa
 *
 * Created on February 25, 2025, 4:06 PM
 */


#include "xc.h"
#include <libpic30.h>
#include "globals_slave.h"
#include "PWMcontrol.h"
#include "motorCntrlSlv.h"
#include "config_slave.h"
#include "enums_slave.h"
#include "MSCommunication_slv.h"



int main(void) {
    
    
    //Configure PLL. There are many combinations to get the same frequency. Here I set the feedback divider to 128,
    //which sets the VCO to oscillate 128 times faster than the FRC source of 8 MHz, and then divide down by a factor
    //of 4 in the first post scaler (POST1DIV), and a factor of 1 in the second post scaler (POST2DIV). PLL output  
    //frequency is then (8 MHz)*128/4/1 = 256 MHz. The Oscillator frequency is half this, or 128 MHz, and the instruction
    //cycle is two oscillator cycles, or 64 MIPS. This choice is designed for convenience so that for a Timer1 prescaler of 256, 
    //there is sort of a 'round' number of Timer1 cycles in a second of 250,000
    PLLFBDbits.PLLFBDIV = 128;
    PLLDIVbits.POST1DIV = 4;
    PLLDIVbits.POST2DIV = 1;

     
    // Initiate Clock Switch to FRC with PLL
    __builtin_write_OSCCONH(0x01); //makes choice of FRC with PLL after switch
    __builtin_write_OSCCONL(OSCCON | 0x01); //initiates the switch
    
       
    while (OSCCONbits.OSWEN != 0); // Wait for Clock switch to occur
    // Wait for PLL to lock
    while (OSCCONbits.LOCK!= 1);
    
    uint32_t blinkCounter=0;
    uint32_t blinkCounterMax = 1000000;  // defines blinking interval
    configureSecondaryPPS();
    configSlaveInitial();
    configurePWM1();
    configurePWM2();
    configureQuadEncoder();
    startPWM1();
    startPWM2();


    // main loop continually monitors the master-slave FIFO and reacts accordingly
    Register fifoReg;
    while(1) {
        
        // Monitor the FIFO for incoming commands and variables from the primary core
        if(!SI1FIFOCSbits.SRFEMPTY) {
            fifoReg=SRMWFDATA;
            if(fifoReg == REG_COMMAND)  {               // command
                processCommandFromPrimary();
            }
            else if (fifoReg == REG_VARIABLE)
            {
                receiveVariableFromPrimary();
            }
            else if (fifoReg == REG_VARIABLE_32) {
                receive32bVariableFromPrimary();
            }
            else if (fifoReg == REG_BOOLVAR) {
                receiveBoolVarFromPrimary();
            }
            else if (fifoReg == REG_PING) {
                processPingRequest();
            }
            
            
        }
        
        // blink LED2
        if(blinkCounter > blinkCounterMax) {
            blinkCounter = 0;
//            LATBbits.LATB1 = ~PORTBbits.RB1;
        }
        else{
            blinkCounter++;
        }
        
    }  // main while loop
    return 0;
}

void __attribute__((__interrupt__,no_auto_psv)) _PWM2Interrupt(void)
{   
    // Performs two primary functions
    //      1) Manages the waveform updates. Every gs_waveformUpdatePeriod interrupts, the demand from the waveform is loaded 
    //         into gs_displacementDemand and the waveform index is advanced
    //      2) Sets the motor output to whatever is in gs_pwmxCycles. That value is set in the feedback loop code and ultimately
    //         comes in through the master-secondary interface.
    // NOTE: This interrupt should be disabled during FIFO writes to the primary core, because this ISR also writes to the
    //       FIFO. The writes could get disrupted if this interrupt happens in the middle of a FIFO write. This interrupt
    //       does not trigger any FIFO reads, so it does not need to be disabled for reads.
    
    
    static uint16_t interruptCount=0;               // tracks the number of PWM1 interrupts since the output has been updated
    
    // motorUpdateHalfInteral is the number of pwm interrupts between motor 1 and motor 2 output updates. So twice this number is the 
    // number of interrupts between each update of one of the motor outputs. For a pwm interrupt interval of 50 microseconds, 
    // setting this number to 25 means each motor gets updated every 2.5 ms, and they take turns getting updated with 1.25 ms 
    // between a motor 1 and motor 2 update.
    static uint16_t motorUpdateHalfInterval=25;     // number of PWM interrupts to motor 1 update 
    static uint16_t motorUpdateFullInterval=50;     // number of PWM interrupts for a full cycle of motor 1-motor 2 update.
    static uint16_t waveform1Count=0;                // tracks how many PWM1 periods between advances in the waveform array    
    static uint16_t waveform2Count=0;                // tracks how many PWM1 periods between advances in the waveform array    
    static uint16_t wf1_ind=0;                      // array index into waveform 1
    static uint16_t wf2_ind=0;                      // array index into waveform 2
    
    
    // Motor 1 
    if(gs_output1Enabled) {  // if output is enabled use waveform
        // Every outputUpdatePeriod interrupts update the pwm output. For pwm period of 50 microseconds, this is once every 2.5 milliseconds.
        // Within the loop, alternate updating motor 1 and motor 2 outputs, so that motor 1 is updated at the beginning of an
        // ...update period, and motor 2 is updated halfway through the update period.
        if(interruptCount == motorUpdateHalfInterval) {      // time to update output to motor with whatever is currently requested
            if(!gs_stopMotors) { // normal condition - no call to zero the output position            
                setMotorOutput1(gs_pwm1Cycles);
            }
            else { // zero output has been requested  
                gs_output1Enabled = false;
                sendBoolVarToPrimary(OUTPUT1_ENABLED, false);
            }
        }

        // increment the waveform index in the waveform array every 10 milliseconds. For a pwm period of 50 microseconds, this means 
        // we need to update every waveformUpdatePeriod=200 interrupts
        waveform1Count++;                                       // this is really counting interrupts
        if(waveform1Count >= gs_waveformUpdatePeriod) {         // time to update what is requested (next element in waveform array)           
            // TO DO NOTE: I BELIEVE THIS IS NO LONGER NECESSARY SINCE WE RESET THE WAVEFORM INDEX (wf1_ind) UPON TERMINATION OF THE LOOP. DOUBLE CHECK AND DELETE
            if(gs_resetWaveform) {                              // if motor has been off, reset the waveform index for the initial run 
                wf1_ind = 0;
                gs_resetWaveform = false;
            }
            if(wf1_ind >= gs_numArrayVals) {                    // if end of array, cycle back to beginning
                wf1_ind = 0;
                if(gs_playSingleWaveformOnly) {                 // if playing a single waveform only, set things up to stop
                    sendBoolVarToPrimary(OUTPUT1_ENABLED, false);
                    gs_output1Enabled = false;
                }
            }
            gs_displacement1Demand = gs_outputWaveform1[wf1_ind]; // Update demand from waveform array
            
            // Make sure MS FIFO is not full, then send the displacement demand back to the primary core for use in feedback calculation
            if(!SI1FIFOCSbits.SWFFULL) {  // FIFO should not fill up, but if a __delay command were put on the master side, it could happen
                send32bVariableToPrimary(DISPLACEMENT1_DEMAND, (uint32_t)gs_displacement1Demand);
            }
            
            wf1_ind++;
            waveform1Count = 0;
        }
//        
    }
    else { // if gs_output1Enabled is false
        waveform1Count = 0;
        wf1_ind = 0;
        setMotorOutput1(gs_pwm1Cycles);  // if output is disabled, gs_pwm1Cycles will be decayed to zero in Timer1 interrupt loop in primary core
        if(!gs_output2Enabled) {  // is both motors are not enabled, set interrupt count to zero. 
            interruptCount = 0;
        }
    }
    
    // Motor 2
    if(gs_output2Enabled) {  // if output is enabled use waveform
        // Every outputUpdatePeriod interrupts update the pwm output. For pwm period of 50 microseconds, this is once every 2.5 milliseconds.
        // Within the loop, alternate updating motor 1 and motor 2 outputs, so that motor 1 is updated at the beginning of an
        // ...update period, and motor 2 is updated halfway through the update period.
        
        
        if(interruptCount >= motorUpdateFullInterval) {      // time to update output to motor with whatever is currently requested
            LATBbits.LATB1 = 1;
            if(!gs_stopMotors) { // normal condition - no call to zero the output position            
                setMotorOutput2(gs_pwm2Cycles);
            }
            else { // zero output has been requested  
                gs_output2Enabled = false;
                sendBoolVarToPrimary(OUTPUT2_ENABLED, false);
            }
        }

        // increment the waveform index in the waveform array every 10 milliseconds. For a pwm period of 50 microseconds, this means 
        // we need to update every waveformUpdatePeriod=200 interrupts
        waveform2Count++;                                       // this is really counting interrupts
        if(waveform2Count >= gs_waveformUpdatePeriod) {         // time to update what is requested (next element in waveform array)           
            // NOTE: I BELIEVE THIS IS NO LONGER NECESSARY SINCE WE RESET THE WAVEFORM INDEX (wf1_ind) UPON TERMINATION OF THE LOOP. DOUBLE CHECK AND DELETE
            if(gs_resetWaveform) {                              // if motor has been off, reset the waveform index for the initial run 
                wf2_ind = 0;
                gs_resetWaveform = false;
            }
            if(wf2_ind >= gs_numArrayVals) {                    // if end of array, cycle back to beginning
                wf2_ind = 0;
                if(gs_playSingleWaveformOnly) {                 // if playing a single waveform only, set things up to stop
                    sendBoolVarToPrimary(OUTPUT2_ENABLED, false);
                    gs_output2Enabled = false;
                 //   gs_playSingleWaveformOnly = false;          // reset for future waveforms. 
                }
            }
            gs_displacement2Demand = gs_outputWaveform2[wf2_ind]; // Update demand from waveform array. 
            
            // Make sure MS FIFO is not full, then send the displacement demand back to the primary core for use in feedback calculation
            if(!SI1FIFOCSbits.SWFFULL) {  // FIFO should not fill up, but if a __delay command were put on the master side, it could happen
                send32bVariableToPrimary(DISPLACEMENT2_DEMAND, (uint32_t)gs_displacement2Demand);
            }
            
            wf2_ind++;
            waveform2Count = 0;
        }
//        
    }
    else { // if gs_output2Enabled is false
        LATBbits.LATB1 = 0;
        waveform2Count = 0;
        wf2_ind = 0;
        setMotorOutput2(gs_pwm2Cycles);  // if output is disabled, gs_pwm2Cycles will be decayed to zero in Timer1 interrupt loop in primary core
    } // end Motor 2
    
    interruptCount++;   
    // Once a full cycle of motor and waveform updates has been completed, reset the interrupt counter
    if(interruptCount > motorUpdateFullInterval) {
        interruptCount = 0;
    }
    // Clear the PWM1 interrupt flag
    IFS4bits.PWM2IF = 0;
    
}



//void __attribute__((__interrupt__,no_auto_psv)) _PWM2Interrupt(void)
//{   
//    // Performs two primary functions
//    //      1) Manages the waveform updates. Every gs_waveformUpdatePeriod interrupts, the demand from the waveform is loaded 
//    //         into gs_displacementDemand and the waveform index is advanced
//    //      2) Sets the motor output to whatever is in gs_pwm1Cycles. That value is set in the feedback loop code and ultimately
//    //         comes in through the master-secondary interface.
//    
//    
//    static uint16_t outputCount=0;              // tracks the number of PWM1 interrupts since the output has been updated
//    static uint16_t outputUpdatePeriod=50;      // number of PWM periods between an update to the output. Make it an even number.   
//    static uint16_t waveformCount=0;            // tracks how many PWM1 periods between advances in the waveform array    
//    static uint16_t wf_ind=0;
//    
//    
//    // Motor 1 
//    if(gs_output1Enabled) {  // if output is enabled use waveform
//        // Every outputUpdatePeriod interrupts update the pwm output. For pwm period of 50 microseconds, this is once every 2.5 milliseconds.
//        // Within the loop, alternate updating motor 1 and motor 2 outputs, so that motor 1 is updated at the beginning of an
//        // ...update period, and motor 2 is updated halfway through the update period.
//        outputCount++;
//        if(outputCount >= outputUpdatePeriod) {      // time to update output to motor with whatever is currently requested
//            if(!gs_zeroPosOutput) {             
//                setMotorOutput1(gs_pwm1Cycles);
//                outputCount = 0;
//            }
//            else { // zero output has been requested
//                gs_output1Enabled = false;
//                sendBoolVarToPrimary(OUTPUT1_ENABLED, false);
//            }
//        }
//
//        // increment the waveform index in the waveform array every 10 milliseconds. For a pwm period of 50 microseconds, this means 
//        // we need to update every waveformUpdatePeriod=200 interrupts
//        waveformCount++;
//        if(waveformCount >= gs_waveformUpdatePeriod) {       // time to update what is requested (next element in waveform array)           
//            if(gs_resetWaveform) {                           // if motor has been off, reset the waveform index for the initial run
//                wf_ind = 0;
//                gs_resetWaveform = false;
//            }
//            if(wf_ind >= gs_numArrayVals) {                  // if end of array, cycle back to beginning
//                wf_ind = 0;
//                if(gs_playSingleWaveformOnly) {
//                    sendBoolVarToPrimary(OUTPUT1_ENABLED, false);
//                    gs_output1Enabled = false;
//                    sendBoolVarToPrimary(OUTPUT2_ENABLED, false);
//                    gs_output2Enabled = false;
//                    gs_playSingleWaveformOnly = false;        // reset for future waveforms. 
//                }
//            }
//            gs_displacementDemand = gs_outputWaveform[wf_ind];
//
//            if(!SI1FIFOCSbits.SWFFULL) {  // FIFO should not fill up, but if a __delay command were put on the master side, it could happen
//                send32bVariableToPrimary(DISPLACEMENT_DEMAND, (uint16_t)gs_displacementDemand);
//            }
//            
//            wf_ind++;
//            waveformCount = 0;
//        }
////        
//    }
//    else { // if gs_output1Enabled
//        outputCount = 0;
//        waveformCount = 0;
//        wf_ind = 0;
//        setMotorOutput1(gs_pwm1Cycles);  // if output is disabled, gs_pwm1Cycles will be decayed to zero in T1 interrupt loop in primary core
//    }
//    
//   
//    // Clear the PWM1 interrupt flag
//    IFS4bits.PWM2IF = 0;
//    
//}