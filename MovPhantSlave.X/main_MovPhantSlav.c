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
    
    configSlaveInitial();
    configurePWM1();
//    configurePWM2();
 //   configurePWM3();
    startPWM1();
    
    // main loop continually monitors the master-slave FIFO and reacts accordingly
    //Register fifo_reg;
    Register fifoReg;
//    Command fifoCmd;
//    Variable fifoVar;
//    uint16_t fifoVal;

    TRISBbits.TRISB1 = 0;
    ANSELBbits.ANSELB1 = 0;

    while(1) {
        
        // Monitor the FIFO for incoming commands and variables from the primary core
        if(!SI1FIFOCSbits.SRFEMPTY) {
//            LATBbits.LATB2 = 1;
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
                //send32bVariableToPrimary(WAVEFORM_TIMESTEP_MICROS, gs_waveformTimeStep_microS);
            }
            else if (fifoReg == REG_BOOLVAR) {
                receiveBoolVarFromPrimary();
            //    sendBoolVarToPrimary(OUTPUT_ENABLED, gs_outputEnabled);
            }
            
            
        }
//        if(gs_startMotor) {
//            setUpWaveform();
//            
//            
//            gs_startMotor = false;    // stops code from entering this block until start requested again
//        }
    }
    return 0;
}

void __attribute__((__interrupt__,no_auto_psv)) _PWM1Interrupt(void)
{
    // The PWM1 interrupt is used to update the motor output. Every outputCount interrupts, the output to the motor is updated
    // with whatever value is in g_pwm1Cycles. That variable gets updated elsewhere. It also controls the advancement through the
    // output waveform. Every g_waveformUpdatePeriod interrupts, the index into the waveform array g_outputWaveform is advanced.
    
    
    
    static uint16_t outputCount=0;              // tracks the number of PWM1 interrupts since the output has been updated
    static uint16_t outputUpdatePeriod=50;      // number of PWM periods between an update to the output.   
    static uint16_t waveformCount=0;            // tracks how many PWM1 periods between advances in the waveform array
    
    static uint16_t wf_ind=0;
    
    if(gs_outputEnabled) {  // if output is enabled use waveform
        // Every 50 PWM1 interrupts update the pwm output. For pwm period of 50 microseconds, this is once every 2.5 milliseconds
        outputCount++;
        if(outputCount >= outputUpdatePeriod) {      // time to update output to motor with whatever is currently requested
            if(!gs_zeroPosOutput) {             
                setMotorOutput(gs_pwm1Cycles);
                outputCount = 0;
 //               LATBbits.LATB1 = 1;
            }
            else { // zero output has been requested
             //   setMotorOutput(0);
                gs_outputEnabled = false;
 //               while(SI1FIFOCSbits.SWFFULL);  // TEMP -- try waiting for FIFO to clear (testing delay)
                sendBoolVarToPrimary(OUTPUT_ENABLED, false);
 //               LATBbits.LATB1 = 0;
            }
        }

        // increment the waveform index in the waveform array every 10 milliseconds. For a pwm period of 50 microseconds, this means 
        // we need to update every waveformUpdatePeriod=200 interrupts
        waveformCount++;
        if(waveformCount >= gs_waveformUpdatePeriod) {       // time to update what is requested (next element in waveform array)           
            if(gs_resetWaveform) {                           // if motor has been off, reset the waveform index for the initial run
                wf_ind = 0;
                gs_resetWaveform = false;
            }
            if(wf_ind >= gs_numArrayVals) {                  // if end of array, cycle back to beginning
                wf_ind = 0;
                if(gs_playSingleWaveformOnly) {
                    //setMotorOutput(0);
                    sendBoolVarToPrimary(OUTPUT_ENABLED, false);
                    gs_outputEnabled = false;
                    gs_playSingleWaveformOnly = false;        // reset for future waveforms. 
                }
            }
            gs_displacementDemand = gs_outputWaveform[wf_ind];
            
//            LATBbits.LATB1 = 1;

            if(!SI1FIFOCSbits.SWFFULL) {  // FIFO should not fill up, but if a __delay command were put on the master side, it could happen
                send32bVariableToPrimary(DISPLACEMENT_DEMAND, (uint16_t)gs_displacementDemand);
            }
            
            wf_ind++;
            waveformCount = 0;
        }
//        
    }
    else {
        //setMotorOutput(0);
        outputCount = 0;
        waveformCount = 0;
        wf_ind = 0;
        setMotorOutput(gs_pwm1Cycles);  // if output is disabled, gs_pwm1Cycles will be decayed to zero in T1 interrupt loop in primary core
 //       LATBbits.LATB1 = 0;
    }
    
    // Clear the PWM1 interrupt flag
    IFS4bits.PWM1IF = 0;
    
}