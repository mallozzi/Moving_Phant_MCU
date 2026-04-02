/*
 * File:   main_MovPhantMast.c
 * Author: richa
 *
 * Created on February 25, 2025, 3:47 PM
 */


#include "xc.h"
#include <libpic30.h>
#include "enums.h"
#include "motorCntrlMstr.h"
#include "StateManagement.h"

// FOSCSEL
#pragma config FNOSC = FRC                  // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config IESO = OFF                   // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FOSC
#pragma config OSCIOFNC = ON                // OSC2 Pin Function bit (OSC2 is regular IO pin)

#pragma config FCKSM = CSECMD               // Clock Switching Mode bits

// FICD
#pragma config ICS = PGD2                   // Selects PGC2 and PGD2 for serial programming pins

// Watchdog timer configuration
#pragma config FWDTEN = ON_SW               // Watchdog Timer Enable bit (WDT controlled via SW, use WDTCON.ON bit)
//#pragma config RCLKSEL = LPRC               // Use LPRC as watchdog timer clock, nominal period of 1 millisecond
#pragma config RWDTPS = 12                  // post scaler. Setting of 13 sets it to 2^13 = 8192 cycles of the watchdog timer clock

// ---------------  Slave Configuration bits -------------------
// FOSCSEL
#pragma config S1FNOSC = FRC              // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config S1IESO = OFF                 // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FOSC
#pragma config S1OSCIOFNC = ON              // OSC2 Pin Function bit (OSC2 is regular IO pin)

#pragma config S1FCKSM = CSECMD             // Clock Switching Mode bits

// FWDT
#pragma config S1FWDTEN = ON_SW             // Watchdog Timer Enable bit (WDT controlled via SW, use WDTCON.ON bit)


// Assign secondary core pin ownership
#pragma config CPRB1 = SLV1                 // LED2 pin
#pragma config CPRC4 = SLV1                 // PWM_Driver1 pin (motor 1)
#pragma config CPRC5 = SLV1                 // DIR1 pin (motor 1)
#pragma config CPRC10 = SLV1                // PWM_Driver2 pin (motor 2)
#pragma config CPRC11 = SLV1                // DIR2 pin (motor 2)
#pragma config CPRB11 = SLV1                // Quad Encode A module RB11 port
#pragma config CPRB13 = SLV1                // Quad Encode B module RB13 port

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
    
       
    while (OSCCONbits.OSWEN != 0);      // Wait for Clock switch to occur
    
    WDTCONLbits.WDTWINEN = 0;           // set non-window mode for watchdog timer
    WDTCONLbits.ON = 1;                 // Turn on watchdog timer
    
 // Initial Configuration
    configurePPS();       // must be done very early - review carefully if doing any config prior to this.
    configureInitial();
    configureInterruptOnChange();
    configurePWM1();
    configurePWM2();
    configurePWM3();
    configureTimer1();
    configureAnalogToDigital();
    configureI2C();
    configureQuadEncoder();


   // INTCON2bits.GIE  = 1;    //global interrupt enable
    
    _program_secondary(1,0,MovPhantSlave);
    _start_secondary();
    __delay32(g_OscillatorFreq*2);
    enableMSFifo();
    Register fifoReg;
    // Launch feedback loop with no output
    g_pwm1Cycles = 0;  // begin with no output
    g_pwm2Cycles = 0;  // begin with no output
    startTimer1(g_feedbackHalfUpdatePeriod);  // argument determines Timer1 interrupt interval in units of Timer1 periods
    
 //   __delay32(g_OscillatorFreq*2);
    
    //uint16_t val=0;
    // Make RB11 digital input for pushbutton
 //   TRISBbits.TRISB11 = 1;
    uint32_t blinkCounter=0;                // counter for LED blink
    uint32_t blinkCounterMax = 500000;      // determines blink rate
    uint32_t readCounter = 0;               // counter for secondary quad encoder read
    uint32_t readCounterMax = 10;           // determines interval to read secondary quad encoder
    bool outOfBounds = false;               // if proximity sensor detects out of bounds starting state
    
    // Watchdog timer
    volatile unsigned *wdtKey; 
    wdtKey = (&WDTCONH);
    
    INTCON2bits.GIE  = 0;                   // disable global interrupts
    readSecondaryQuadEncoder();             // Perform initial read of secondary quadrature encoder
    INTCON2bits.GIE  = 1;                   //global interrupt enable
    
    
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
        
        // clear watchdog timer
        *wdtKey = 0x5743;  // clear WDT by performing a 16-bit write to WDTCON
        
        // Blink LED 1
        if(blinkCounter == blinkCounterMax) {
 //           LATDbits.LATD10 = ~PORTDbits.RD10;
            blinkCounter = 0;
        }
        else {
            blinkCounter++;
        }
        
        // Monitor Master-Secondary Read Fifo for incoming transmission
        while(!MSI1FIFOCSbits.RFEMPTY) {  // read until the read FIFO is empty
            
            if(!MSI1FIFOCSbits.RFEMPTY) {
                fifoReg=MRSWFDATA;
                if(fifoReg == REG_VARIABLE_32)  {               // command
                    receive32bVariableFromSecondary();
                }
                else if(fifoReg == REG_VARIABLE) {
                    receiveVariableFromSecondary();
                }
                else if(fifoReg == REG_BOOLVAR) {
                   receiveBoolVarFromSecondary();
                }
            }
 //           __delay32(100);
        }
        
        // Read the secondary quadrature encoder position with every pass. The position is updated in the 
        // ...g_secondaryQuadEncPos variable very frequently and is therefore up to date wherever else it is needed,
        // ...as this main loop executes very quickly when no interrupt service routine is executing.
        if(readCounter > readCounterMax) {
            INTCON2bits.GIE  = 0;    //global interrupt disable to avoid fifo conflicts
            readSecondaryQuadEncoder();
            INTCON2bits.GIE  = 1;    //global interrupt enable
            readCounter=0;
        }
        else{
            readCounter++;
        }
//        __delay32(2000);
         
         // The purpose of starting the motor this way rather than calling a function from registerHandler is to allow
         // the I2c transmission to finish without having to wait for all the waveform configuration code to run.
         // Master does the configuration on its side, then sends a command to the secondary to do its configuration
         // and start the motion
         if(g_startMotor) {             
            // setLED1(1);
            outOfBounds = false;                                // even if we are out of bounds we want to be able to walk back in
            configureDerivedQuantities();
            if(g_stepMode) {
                g_statusFlags = g_statusFlags & 1;              // clear status flags except for proximity sensor error
                if( PORTCbits.RC12 && PORTCbits.RC13 ) {        // if we are in bounds, clear the proximity violation
                    g_statusFlags = g_statusFlags & 0xFFFE;     // clears bit 0                    
                }
            }
            else {
                g_statusFlags = 0;                      // clear all status flags
                // check to make sure proximity sensors are not out of bounds
                if( (!PORTCbits.RC12) || (!PORTCbits.RC13) ) {                   // low signal is out of bounds
                    // wait a few microseconds and try again to make sure it wasn't a transient
                    __delay32(g_OscillatorFreq*2/1000000);
                    if( (!PORTCbits.RC12) || (!PORTCbits.RC13) ) {
                        outOfBounds = true;
                    }
                }
                
            }
            if(!outOfBounds) {
                setZeroPosition();
                g_displacement1Demand = 0;   
                g_displacement2Demand = 0; 
                sendParamtersToSecondary();
                while(!MSI1FIFOCSbits.WFEMPTY);             // wait for secondary to finish reading the FIFO. 
                enableDriver(true);
                sendCommandToSecondary(START_MOTION);                  
            }
            else {
                g_statusFlags = g_statusFlags | 1;          // set proximity sensor error status flag
                
            }
            g_startMotor = false;                           // stops code from entering this block until start button pushed again
         }
         
         if(g_gotoLandmark) {
             gotoLandmark();
             g_gotoLandmark = false;
         }

    }  // while(1))
    
    return 0;
}


//Timer1 interrupt service routine
void __attribute__((__interrupt__,no_auto_psv)) _T1Interrupt(void)
{
    // The Timer1 interrupt is used to update the feedback loop. Each motor has the feedback updated every other
    // interrupt, so that one motor has an update period of two Timer1 interrupt periods.
    // The output is set by updating the g_pwmxCycles variable. 
    // NOTE: This ISR should be disabled during Master-Slave FIFO writes, as it also triggers writes to the FIFO
    //       that could interfere.
    
    static uint16_t counter = 0;                    // Counts interrupts to manage alternating motor updates.
    static bool firstPass = true;                   // identifies the first time the ISR is called for variable initialization purposes
    
    // variables used to temporarily hold quadrature encoder bytes while reading
    static uint16_t posLowByte;
    static uint32_t posHighByte;
    
    //feedback control parameters for motor 1
    static uint16_t speed1Register;
    static int16_t velocity1;                       // signed velocity in units of encoder steps per interrupt period
    static uint32_t position1;                      //absolute encoder position reading with large zero offset
    static int32_t displacement1;                   //position relative to zero position
    static int16_t pwmPosition1;                    // pwm output corresponding to the position
    
    //feedback control parameters for motor 2
    static int32_t displacement2;                   //position relative to zero position
    static int16_t pwmPosition2;                    // pwm output corresponding to the position
    static uint32_t lastPositionEnc2;                // stores previous position of quad encoder 2 for velocity calculation
    
    // Position feedback error parameters for motor 1
    static int32_t displacement1Error;              // error signal in units of encoder steps.
    static int32_t integralDisplacement1Error = 0;  // integral of displacement error signal.
    
    // Position feedback error parameters for motor 2
    static int32_t displacement2Error;              // error signal in units of encoder steps.
    static int32_t integralDisplacement2Error = 0;  // integral of displacement error signal.
   
    // other feedback error variables
    static int32_t integralErrorLimit = 30000;      // value at which integral error signal is clipped
    
    // slow-start parameters
//    static bool lastEnabledState = false;           // stores state of g_outputEnabled last time this function was entered
    static int32_t ramped1Demand;                   // actual demand value that will be used to drive motor 1
    static int32_t ramped2Demand;                   // actual demand value that will be used to drive motor 2
    static int16_t rampUpNumerator=0;                // numerator of slow ramp up factor
    static int32_t rampDenomCalculated;            // for speed purposes, this is the calculated ramp numerator
    static uint8_t denomBitShifts=10;               // denominator of ramp up factor, expressed as a number of bit shifts
    static bool fullyRamped = false;                // true if we have finished ramping up the slow start.
//    static int64_t tmp;
//    static int32_t tmp2;
//    static uint16_t rampCntr = 0;
    
    rampDenomCalculated = 1 << denomBitShifts;
    
    
    if(counter == 0) { // motor 1 feedback loop update
         
        // Read velocity register. When direction is positive, velocity counter counts backward.
        speed1Register = VEL1CNT;   
        velocity1 = (int16_t) speed1Register;
        if (speed1Register >= 0x7FFF) {   // speed was actually negative
            // take the two's complement negative
            speed1Register = ~speed1Register + 1;   // two's complement
            velocity1 = -(int16_t)speed1Register;
        }
        // Set velocity PWM for analog output. Velocity is pwm3 module. 
        setOnCyclesPWM3((uint16_t)(velocity1 + g_pwm3ZeroOffset));

        // Read position register and update pwm position output
        posLowByte = POS1CNTL; // Should load POS1CNTH into POS1HLD
        posHighByte = POS1HLD;
        position1 = (posHighByte<<16) + posLowByte;
        if(position1 >= g_encoder1ZeroPos) {
            displacement1 = (int32_t)(position1 - g_encoder1ZeroPos);
        }
        else {
            displacement1 = -(int32_t)(g_encoder1ZeroPos - position1);
        }
        // Convert displacement to a pwm output for external position analog signal output.
        // This is not part of the feedback calculation, just an external signal for monitoring position
        pwmPosition1 = ( __builtin_divsd(displacement1, g_encoderToPwmDenom_1) + (int32_t)g_pwm1ZeroOffset );
        
        // clip the output if it goes out of bounds
        if(pwmPosition1 < 0) {
            pwmPosition1 = 0;
        }
        else if (pwmPosition1 > (int16_t)g_maxPWMInteger) {
            pwmPosition1 = (int16_t) g_maxPWMInteger;
        }

        // set the position PWM for analog output
        setOnCyclesPWM1((uint16_t)pwmPosition1);

        // ---------------  POSITION FEEDBACK CONTROL ------------------
 //       rampNumerator++;
        if(g_output1Enabled) { 
            
            // Create a version of the demand that is ramped u slowly, then when done uses the orginal demand. Upon
            // a normal stop situation, it ramps down slowly
            if(fullyRamped) {
                ramped1Demand = g_displacement1Demand;
            }
            else {
                 
                ramped1Demand = g_displacement1Demand * rampUpNumerator;  // this is a fast calculation
                ramped1Demand = ramped1Demand >> denomBitShifts;        // a fast way to divide by a power of 2 with truncation
            }
            
            displacement1Error = displacement1 - ramped1Demand;
            integralDisplacement1Error+= displacement1Error;
            // clip integral error
            if(integralDisplacement1Error > integralErrorLimit) {
                integralDisplacement1Error = integralErrorLimit;
            }
            if(integralDisplacement1Error < -integralErrorLimit) {
                integralDisplacement1Error = -integralErrorLimit;
            }

            if(!g_motorTestMode) {
                // Set output = Proportional term + Integral term + Derivative Term
                g_pwm1Cycles = -MultiplyByFraction((int16_t)displacement1Error, g_propConstNum1, g_propConstDenom1) 
                              - MultiplyByFraction((int16_t)integralDisplacement1Error, g_intConstNum1, g_intConstDenom1)
                              - MultiplyByFraction(velocity1, g_derivConstNum1, g_derivConstDenom1);
            }
            else {  // motor test mode - spin motor at constant pwm
                g_pwm1Cycles = g_motorTestPwm;
            }

        }
        else {  // !g_output1Enabled
            
            // Decay output voltage gradually. Rate of decay in ms will depend upon T1 interrupt rate
            integralDisplacement1Error = 0;
           // g_pwm1Cycles = (int16_t)( (int32_t)g_pwm1Cycles*98/100 );   original
            g_pwm1Cycles = __builtin_divsd((int32_t)g_pwm1Cycles*98, 100);   // multiplies g_pwm1Cycles by 98/100
            g_displacement1Demand = 0;   // This probably doesn't matter anymore and should probably just be set in the secondary and removed from here.

            if(!g_output2Enabled) {
                rampUpNumerator=0;
                fullyRamped = false;
            } // if both motors are disabled
        }

        // This is done even if output disabled because g_displacement1Demand will set the future output. The pwm1 cycles are
        // decayed down in the else code above if the output gets turned off.
        sendVariableToSecondary(PWM1_CYCLES, (uint16_t)g_pwm1Cycles);  // send to secondary core
        
    } // end of motor 1 loop
    else { // counter==1, MOTOR 2 feedback loop update
 //       LATDbits.LATD10 = 1;
        // The first time through this routine, initialize things so that the velocity comes out zero
        if(firstPass) {                 
            lastPositionEnc2 = g_secondaryQuadEncPos;  // ensures that the velocity will be zero the first time through
            firstPass = false;
        }
        
        // g_secondaryQuadEncPos and lastPositionEnc2 are unsigned integers, so we need to deal with negative velocities
        if(g_secondaryQuadEncPos >= lastPositionEnc2) {
            g_secondaryQuadEncVel = (int32_t)(g_secondaryQuadEncPos - lastPositionEnc2);
        }
        else {
            g_secondaryQuadEncVel = -(int32_t)(lastPositionEnc2 - g_secondaryQuadEncPos);
        }
        lastPositionEnc2 = g_secondaryQuadEncPos;
        
        
        // Set velocity PWM for analog output. Uncomment the line below to output motor 2 velocity, and comment out the corresponding
        // ... line in the motor 1 code above.
        //setOnCyclesPWM3((uint16_t)(g_secondaryQuadEncVel + g_pwm3ZeroOffset));
        
        // encoder position is constantly updated in the main routine, so g_secondaryQuadEncPos has current position taken
        // ...within the last few microseconds. The code block below calculates the displacement relative to the zero position
        // ...for use in the feedback loop and for sending to the analog output PWM
        if(g_secondaryQuadEncPos >= g_encoder2ZeroPos) {
            displacement2 = (int32_t)(g_secondaryQuadEncPos - g_encoder2ZeroPos);  
        }
        else {
            displacement2 = -(int32_t)(g_encoder2ZeroPos - g_secondaryQuadEncPos);
        }
        // Convert displacement to a pwm output for external position analog signal output.
        // This is not part of the feedback calculation, just an external signal for monitoring position
        pwmPosition2 = ( __builtin_divsd(displacement2, g_encoderToPwmDenom_2) + (int32_t)g_pwm2ZeroOffset );
        
        // clip the output if it goes out of bounds
        if(pwmPosition2 < 0) {
            pwmPosition2 = 0;
        }
        else if (pwmPosition2 > (int16_t)g_maxPWMInteger) {
            pwmPosition2 = (int16_t) g_maxPWMInteger;
        }

        // set the position PWM output
        setOnCyclesPWM2((uint16_t)pwmPosition2);

        // ---------------  POSITION FEEDBACK CONTROL ------------------
        if(g_output2Enabled) {
            if(fullyRamped) {
                ramped2Demand = g_displacement2Demand;
            }
            else {
                ramped2Demand = g_displacement2Demand * rampUpNumerator;  // this is a fast calculation
                ramped2Demand = ramped2Demand >> denomBitShifts;        // a fast way to divide by a power of 2 with truncation
            }
            
            displacement2Error = displacement2 - ramped2Demand;

            integralDisplacement2Error+= displacement2Error;
            // clip integral error
            if(integralDisplacement2Error > integralErrorLimit) {
                integralDisplacement2Error = integralErrorLimit;
            }
            if(integralDisplacement2Error < -integralErrorLimit) {
                integralDisplacement2Error = -integralErrorLimit;
            }

            if(!g_motorTestMode) {
                // Set output = Proportional term + Integral term + Derivative Term. g_pwm2Cycles refers to motor 2, not
                // the pwm2 module on the primary core
                g_pwm2Cycles = -MultiplyByFraction((int16_t)displacement2Error, g_propConstNum2, g_propConstDenom2) 
                              - MultiplyByFraction((int16_t)integralDisplacement2Error, g_intConstNum2, g_intConstDenom2)
                              - MultiplyByFraction(g_secondaryQuadEncVel, g_derivConstNum2, g_derivConstDenom2);
            }
            else {  // motor test mode - spin motor at constant pwm
                g_pwm2Cycles = g_motorTestPwm;
            }

        }
        else {  // !g_output2Enabled
            // Decay output voltage gradually. Rate of decay in ms will depend upon T1 interrupt rate
            integralDisplacement2Error = 0;
            //g_pwm2Cycles = (int16_t)( (int32_t)g_pwm2Cycles*98/100 );   //original
            g_pwm2Cycles = __builtin_divsd((int32_t)g_pwm2Cycles*98, 100);    // multiplies g_pwm2Cycles by 98/100
            g_displacement2Demand = 0;   // This probably doesn't matter anymore and should probably just be set in the secondary and removed from here.
            
            if(!g_output1Enabled) {     // if both motors are disabled.
                rampUpNumerator = 0;
                fullyRamped = false;
            }
        }

        // This is done even if output disabled because g_displacement2Demand will set the future output. The pwm2 cycles are
        // decayed down in the else code above if the output gets turned off.            
        sendVariableToSecondary(PWM2_CYCLES, (uint16_t)g_pwm2Cycles);  // send to secondary core
    } // end of motor 2 loop
    rampUpNumerator++;
    counter++;
    counter = counter%2;    // counter should count 0 to 1 repeatedly
    
    // check if ramping stage is done
    if(rampUpNumerator == rampDenomCalculated) {
        fullyRamped = true;
        rampUpNumerator = 0;
    }
    // --------------- END POSITION FEEDBACK CONTROL ------------------  

    
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


void __attribute__((__interrupt__,no_auto_psv)) _CNBInterrupt(void) {
    // Interrupt Service Routine for Change Notice on PORTB pins. Detects one of 
    
    if(CNFBbits.CNFB15) {                       // RB15
        stopMotion();
    }
    CNFBbits.CNFB15 = 0;
    
    IFS0bits.CNBIF = 0;                 // clear interrupt flag
}

void __attribute__((__interrupt__,no_auto_psv)) _CNCInterrupt(void) {
    // Interrupt Service Routine for Change Notice on Proximity Sensor pins
    
    static bool allowStep = false;
    // If we tripped a proximity sensor, we want to enable stepping to get back in bounds. The variable allowStep is 
    // set to true if a proximity sensor was already tripped previously and we are now in stepping mode, so that the
    // user can use the stepping buttons to come back in bounds.
    
    allowStep = (g_statusFlags & 1) && g_stepMode;  // proximity sensor error already set and we are in step mode
    
    // motor 1 proximity sensor on RC12
    if(CNFCbits.CNFC12 && !allowStep) {             // in stepping mode we allow violation so we can get out of jail
        __delay32(g_OscillatorFreq*2/1000000);      // TEMP: delay a bit to make sure it wasn't a glitch. TO DO: remove after hardware fix
                
        if(!PORTCbits.RC12) {                       // if it has stayed low
            stopMotion();                           // Stop both motors
            g_statusFlags = g_statusFlags | 1;      // set the error flag for out of range error
        }
    }
    CNFCbits.CNFC12 = 0;
    
    // motor 2 proximity sensor on RC13
    if(CNFCbits.CNFC13 && !allowStep) {             // in stepping mode we allow violatin so we can get out of jail
        __delay32(g_OscillatorFreq*2/1000000);      // TEMP: delay a bit to make sure it wasn't a glitch. TO DO: remove after hardware fix
        if(!PORTCbits.RC13) {                        // If it has stayed low
            stopMotion();                           // Stop both motors
            g_statusFlags = g_statusFlags | 1;      // set the error flag for out of range error
        }
    }
    CNFCbits.CNFC13 = 0;
    
    IFS1bits.CNCIF = 0; 
}