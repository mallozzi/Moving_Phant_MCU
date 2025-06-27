/*
 * File:   main_MovPhantMast.c
 * Author: richa
 *
 * Created on February 25, 2025, 3:47 PM
 */


#include "xc.h"
#include <libpic30.h>
#include "enums.h"

// FOSCSEL
#pragma config FNOSC = FRC                  // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config IESO = OFF                   // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FOSC
#pragma config OSCIOFNC = ON                // OSC2 Pin Function bit (OSC2 is regular IO pin)

#pragma config FCKSM = CSECMD               // Clock Switching Mode bits

// FWDT
#pragma config FWDTEN = ON_SW               // Watchdog Timer Enable bit (WDT controlled via SW, use WDTCON.ON bit)

// FICD
#pragma config ICS = PGD2                   // Selects PGC2 and PGD2 for serial programming pins

// ---------------  Slave Configuration bits -------------------
// FOSCSEL
#pragma config S1FNOSC = FRC              // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config S1IESO = OFF                 // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FOSC
#pragma config S1OSCIOFNC = ON              // OSC2 Pin Function bit (OSC2 is regular IO pin)

#pragma config S1FCKSM = CSECMD             // Clock Switching Mode bits

// FWDT
#pragma config S1FWDTEN = ON_SW             // Watchdog Timer Enable bit (WDT controlled via SW, use WDTCON.ON bit)



#pragma config CPRB1 = SLV1                 // LED1 pin
//#pragma config CPRB2 = SLV1                 // LED2 pin
#pragma config CPRB14 = SLV1                // PWM1 pin
#pragma config CPRB15 = SLV1                // DIR pin

#include "globals.h"
#include "Configure.h"
#include "Timer1.h"
#include "StateManagement.h"
#include "registerHandler.h"
#include "MathUtil.h"   // file is in Slave folder directory
#include "PWMcontrolMstr.h"
#include "enums.h"
#include "MovPhantSlave.h"
#include "MSCommunication_mstr.h"

unsigned long int g_OscillatorFreq = 128000000; //oscillator frequency in cycles/sec. This is twice the instruction cycle frequency
// Note: __delay32() delays a certain number of instruction clock cycles (FCY). So __delay32(g_OscillatorFreq) would be a two second delay

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
    
 // Initial Configuration
    configurePPS();       // must be done very early - review carefully if doing any config prior to this.
    configureInitial();
    configureDirection();
    configureEncoderInputs();
    configurePWM2();
    configurePWM3();
    configureTimer1();
    configureAnalogToDigital();
    configureI2C();
    configureQuadEncoder();
 //   configureDerivedQuantities();  // This must be the last configuration run.

    // Set PCB LED state if desired
//    LATBbits.LATB2 = 0;   // Illuminate LED to show MCU power
//    LATBbits.LATB1 = 0;

    INTCON2bits.GIE  = 1;    //global interrupt enable
    
    // Temp test
    int16_t aa = -20001;
    uint16_t bb;
    int16_t cc;
    bb = (uint16_t) aa;
    cc = (int16_t) bb;
    // End temp test
    
    _program_secondary(1,0,MovPhantSlave);
    _start_secondary();
   // __delay32(g_OscillatorFreq*2);
    enableMSFifo();
    Register fifoReg;
  //  BoolVariable testBool = true;
    // Launch feedback loop with no output
    g_pwm1Cycles = 0;  // begin with no output
    startTimer1(g_feedbackUpdatePeriod);
    
    //uint16_t val=0;
    // Make RB11 digital input for pushbutton
    TRISBbits.TRISB11 = 1;
    while(1) {
        // IMPORTANT NOTE ABOUT DELAYS IN THIS LOOP:
        // Substantial delays should not be put in the main while loop, as the design of the 
        // primary-secondary FIFO communication assumes that the FIFOs are not both filling up.
        // If a FIFO fills up, the cores could end up just waiting for one another to finish reading.
        // With this design, it is more likely that the Slave-write-master-read FIFO would fill up first, as
        // a write is made each time the waveform index is advanced in the PWM1 interrupt loop. If that gets
        // stuck waiting for the master to read off its FIFO, the master write FIFO could then fill up as a result
        // and both cores would just wait for one another.
  //      __delay32(g_OscillatorFreq/2);
       
//        if(val==0) {
//            startMotion();
//     //       LATBbits.LATB2 = 0;
//     //       sendBoolVarToSecondary(OUTPUT_ENABLED, true);
//     //       send32bVariableToSecondary(WAVEFORM_TIMESTEP_MICROS, 7000);
//            val=1;
//        }
//        else if (val==1) {
//     //       LATBbits.LATB2 = 1;
//            stopMotion();
//           // sendBoolVarToSecondary(OUTPUT_ENABLED, false);
//            //send32bVariableToSecondary(WAVEFORM_TIMESTEP_MICROS, 0x0F000010);
//            val=0;
//        }
//        __delay32(g_OscillatorFreq/200);
        // Monitor Master-Secondary Read Fifo for incoming transmission
        while(!MSI1FIFOCSbits.RFEMPTY) {  // read until the read FIFO is empty
            if(!MSI1FIFOCSbits.RFEMPTY) {
     //           LATBbits.LATB2 = 1;
                fifoReg=MRSWFDATA;
                if(fifoReg == REG_VARIABLE_32)  {               // command
                    receive32bVariableFromSecondary();
                }
                else if(fifoReg == REG_BOOLVAR) {
                   receiveBoolVarFromSecondary();
                }

            }
        }
        
//        if(val==0 && PORTBbits.RB11 == 1) {
//            sendCommandToSecondary(START_MOTION);
//            val=1;
//        }
//        else if (val==1 && PORTBbits.RB11 == 0) {
//            sendCommandToSecondary(STOP_MOTION);
//            val=0;
//        }
        
                // 2**** NEW VERSION OF MAIN CODE ****2
         
         // The purpose of starting the motor this way rather than calling a function from registerHandler is to allow
         // the I2c transmission to finish without having to wait for all the waveform configuration code to run.
         // Master does the configuration on its side, then sends a command to the secondary to do its configuration
         // and start the motion
         if(g_startMotor) {
            configureDerivedQuantities();
          //  setUpWaveform();
           // g_resetWaveform = true;
            setZeroPosition();
            g_displacementDemand = 0;           
 //           g_zeroPosOutput = false;
            //g_outputEnabled = true;
            sendParamtersToSecondary();
            while(!MSI1FIFOCSbits.WFEMPTY);         // wait for secondary to finish reading the FIFO. 
            enableDriver(true);
            sendCommandToSecondary(START_MOTION);
            g_startMotor = false;         // stops code from entering this block until start button pushed again
            
            // TO DO: send command to slave
         }
         
         if(g_gotoLandmark) {
             gotoLandmark();
             g_gotoLandmark = false;
         }
        
  //      __delay32(g_OscillatorFreq/40);
     
         // 2**** END SECTION: NEW VERSION OF MAIN CODE ***2
        
        
        
        // 8**** THIS SECTION HAS THE MAIN CODE THAT NEEDS TO BE MODIFIED FOR MASTER-SLAVE OPERATION ****8
//         __delay32(g_OscillatorFreq/256);
//         
//         
//         
//         // The purpose of starting the motor this way rather than calling a function from registerHandler is to allow
//         // the I2c transmission to finish without having to wait for all the waveform configuration code to run.
//         // Master does the configuration on its side, then sends a command to the secondary to do its configuration
//         // and start the motion
//         if(g_startMotor) {
//            configureDerivedQuantities();
//          //  setUpWaveform();
//            g_resetWaveform = true;
//            setZeroPosition();
//            g_displacementDemand = 0;
//            enableDriver(true);           
//            g_zeroPosOutput = false;
//            g_outputEnabled = true;
//            g_startMotor = false;         // stops code from entering this block until start button pushed again
//            
//            // TO DO: send command to slave
//         }
//         
//         if(g_gotoLandmark) {
//             gotoLandmark();
//             g_gotoLandmark = false;
//         }
     
         // 8**** END SECTION ***8
    }  // while(1))
    
    return 0;
}


//Timer1 interrupt service routine
void __attribute__((__interrupt__,no_auto_psv)) _T1Interrupt(void)
{
    // The Timer1 interrupt is used to update the feedback loop. Position and velocity are read at each interrupt and the output adjusted accordingly.
    // The output is set by updating the g_pwm1Cycles variable. 
    
  //  static uint16_t count=0;
    static uint16_t speedRegister;
    static int16_t velocity;                        // signed velocity in units of encoder steps per interrupt period
    static uint16_t posLowByte;
    static uint32_t posHighByte;
    static uint32_t position;                       //absolute encoder position reading with large zero offset
    static int32_t displacement;                    //position relative to zero position
    static int16_t pwmPosition;                    // pwm output corresponding to the position
    
    // Position feedback control parameters
    static int32_t displacementError;               // error signal in units of encoder steps.
    static int32_t integralDisplacementError = 0;   // integral of displacement error signal.
    static int32_t integralErrorLimit = 30000;      // value at which integral error signal is clipped
    static uint16_t tmpCount = 0;                 // development variable for diagnostics
    
    
    // Read velocity register. When direction is positive, velocity counter counts backward.
    speedRegister = VEL1CNT;   
    velocity = (int16_t) speedRegister;
    if (speedRegister >= 0x7FFF) {   // speed was actually negative
        // take the two's complement negative
        speedRegister = ~speedRegister + 1;   // two's complement
        velocity = -(int16_t)speedRegister;
    }
    // Set velocity PWM output
    setOnCyclesPWM3((uint16_t)(velocity + g_pwm3ZeroOffset));
    
    // Read position register and update pwm position output
    posLowByte = POS1CNTL; // Should load POS1CNTH into POS1HLD
    posHighByte = POS1HLD;
    position = (posHighByte<<16) + posLowByte;
    if(position >= g_encoderZeroPos) {
        displacement = (int32_t)(position - g_encoderZeroPos);
    }
    else {
        displacement = -(int32_t)(g_encoderZeroPos - position);
    }
    // Convert displacement to a pwm output for external position signal output. Clip if out of bounds
    pwmPosition = ( __builtin_divsd(displacement, g_encoderToPwmDenom) + (int32_t)g_pwm2ZeroOffset );
    if(pwmPosition < 0) {
        pwmPosition = 0;
    }
    else if (pwmPosition > (int16_t)g_maxPWMInteger) {
        pwmPosition = (int16_t) g_maxPWMInteger;
    }
    
    // set the position PWM output
    setOnCyclesPWM2((uint16_t)pwmPosition);
    
    // ---------------  POSITION FEEDBACK CONTROL ------------------
    if(g_outputEnabled) {
        displacementError = displacement - g_displacementDemand;
        integralDisplacementError+= displacementError;
        // clip integral error
        if(integralDisplacementError > integralErrorLimit) {
            integralDisplacementError = integralErrorLimit;
        }
        if(integralDisplacementError < -integralErrorLimit) {
            integralDisplacementError = -integralErrorLimit;
        }

        if(!g_motorTestMode) {
            // Set output = Proportional term + Integral term + Derivative Term
            g_pwm1Cycles = -MultiplyByFraction((int16_t)displacementError, g_propConstNum, g_propConstDenom) 
                          - MultiplyByFraction((int16_t)integralDisplacementError, g_intConstNum, g_intConstDenom)
                          - MultiplyByFraction(velocity, g_derivConstNum, g_derivConstDenom);
        }
        else {  // motor test mode - spin motor at constant pwm
            g_pwm1Cycles = g_motorTestPwm;
        }
        // temp code for diagnostics
        tmpCount++;
        if (tmpCount > 100) { 
 //           LATBbits.LATB2 = ~PORTBbits.RB2;
            tmpCount=0;
        }
    }
    else {  // g_outputEnabled
	// Decay output voltage gradually. Rate of decay in ms will depend upon T1 interrupt rate
        integralDisplacementError = 0;
        g_pwm1Cycles = (int16_t)( (int32_t)g_pwm1Cycles*93/100 );   
        g_displacementDemand = 0;   // This probably doesn't matter anymore and should probably just be set in the secondary and removed from here.
      //  LATBbits.LATB2 = 0;
    }
    
    // This is done even if output disabled because g_displacementDemand will set the future output. The pwm1 cycles are
    // decayed down in the else code above if the output gets turned off.
    sendVariableToSecondary(PWM1_CYCLES, (uint16_t)g_pwm1Cycles);  // send to secondary core
    
    // --------------- END POSITION FEEDBACK CONTROL ------------------  

    //LED output for development purposes
//    if(g_pwm1Cycles > 11800) {
//        LATBbits.LATB1 = 1;
//        LATBbits.LATB2 = 0;
//        
//    }
//    else if(g_pwm1Cycles < -11800) {
//        LATBbits.LATB1 = 0;
//        LATBbits.LATB2 = 1;
//
//    }
//    else {
//        LATBbits.LATB1 = 0;
//        LATBbits.LATB2 = 0;
//    }
    
    IFS0bits.T1IF = 0;

}

void __attribute__((__interrupt__,no_auto_psv)) _SI2C1Interrupt(void) {
    // ISR to handle I2C slave communication for transmitting and receiving 16-bit data, using an 8-bit register address.
    // With either a write or a read operation, the register value is first sent as a write to this MCU with the register value.
    // In the write request, the register value is then followed immediately by the 16-bit data value (with the additional I2C acknowledge bit).
    // In the read request, after the register value write request, the address is sent again with a read request. This code then sends back
    // the 16-bit data value in response, with the least significant 8 bits first, followed by the most significant 8 bits.
    uint16_t val=0;
    static uint16_t tmp=0;
    static uint16_t index=0;
    static uint8_t registerNumber=0;
    static uint16_t lsb=0;
    static uint16_t msb=0;
    static uint16_t dataByte16=0;
    
    if(I2C1STATbits.D_A == 0) {     // it was an address byte        
        
        IFS1bits.SI2C1IF = 0;           //clear interrupt flag
        tmp = I2C1RCV;          // read receive buffer so that it gets cleared     
        // The code below that checks for the buffer full error I2COV is related to a flaw in the micro-controller implementation in slave mode. The buffer
        // can be full from other activities, before any i2c data has actually come in.
        if(I2C1STATbits.I2COV ==1){
            
            I2C1STATbits.I2COV = 0;
            tmp = I2C1RCV;
            if(tmp >> 1 != I2C1ADD){ //check to make sure buffer value matches address, and do another read if not.
                tmp = I2C1RCV;
            }
        }
       
    }
    if(I2C1STATbits.D_A == 1 && I2C1STATbits.R_W == 0){ // it was a data byte and a write to slave request
        
        val = I2C1RCV;              // read data buffer
        
        if(index == 0) {
            // register number is 8-bit
            registerNumber = (uint8_t)(val & 0x00FF);
            index = index+1;
        }
        else if(index==1) {
            // least significant 8 bits of a 16-bit data byte
            lsb = val;
            index = index+1;
        }
        else if(index == 2) {
            // most significant 8 bits of a 16-bit data byte
            msb = val;
            dataByte16 = (msb << 8) + lsb;      // construct 16-bit number
            
            // *******************************
            // Custom Code to do something with the data that came in
            setRegisterValue(registerNumber, dataByte16);  
            //********************************
            
            lsb = 0;
            msb = 0;
            index = 0;
        }
    }
    else if(I2C1STATbits.R_W == 1) { //it is a master read from slave request. I believe I2C1STATbits.D_A = 1 here.                
       if(index == 1) { //zero index was a write to send the register number

           //************ CUSTOM CODE SEGMENT**********
           //** assumes that the registerValue has been populated by earlier write operations, and fetches data associated with that register  
           val = getRegisterValue(registerNumber);  // custom function to fetch the data to transmit  
           //******************************************

           lsb = (val & 0x00FF);           //least significant bits
           msb = (val & 0xFF00) >> 8;      // most significant bits
           I2C1TRN = (uint8_t)lsb;         // load lsb data into transmit buffer
           index = index+1;
       }
       else if(index==2) {
           I2C1TRN = (uint8_t)msb;         // load msb data into transmit buffer
           index = 0;  //reset index
       }
    }

    IFS1bits.SI2C1IF = 0;
    I2C1CONLbits.SCLREL = 1;        //release clock
}
