#include "Configure.h"
#include <xc.h>
#include "globals.h"
//#include "PWMcontrol.h"
#include <stdbool.h>
//#include <p33CH128MP502.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

void configureInitial() {
// Initial configuration. It is assumed that this method is called before other configurations are done, as it broadly sets a lot of
// config states to a rational starting value, assuming they will later be set by other configuration calls.
    
    // Start by making all I/O pins digital outputs in their low state 
    ANSELA = 0;
    TRISA = 0;
    LATA = 0;
    ANSELB = 0;
    TRISB = 0;
    LATB = 0;
    ANSELC = 0;
    TRISC = 0;
    LATC = 0;
    ANSELD = 0;
    TRISD = 0;
    LATD = 0;
    
    INTCON1bits.NSTDIS = 1;  //1 to disable nested interrupts
    IPC4bits.SI2C1IP = 5;    // give I2c a higher interrup priority than Timer1, which has a natural IP of 4
    
    // LED 1 Pin RD10
    TRISDbits.TRISD10 = 0;
    
    // Configure the fault input and sleep output pins
    // RA0 is fault input from driver
    TRISAbits.TRISA0 = 1;       // digital input
    TRISBbits.TRISB13 = 0;      // digital output
    
    // Set driver to sleep mode by default. 
    // RB13 puts the driver into sleep mode of it is logic low, and the toggle switch is set to MCU control
    LATBbits.LATB13 = 0;        // default to sleep mode so that it must be actively enabled to turn motor
    
    // Configure Fault output pin RB0
    TRISBbits.TRISB0 = 0;
    LATBbits.LATB0 = 0;
    
    // Configure Fault Input Pin from driver RA0 as digital input
    TRISAbits.TRISA0 = 1;
    ANSELAbits.ANSELA0 = 0;
    
    // set motor and goto landmark actions to non-starting state
    g_startMotor = false;
    g_gotoLandmark = false;
    
    // Quadrature encoder inputs. Not analog pins, so no need to set that
    // Set digital input pins for Quad Enc 1 (motor 1). Motor 2 quad enc is on secondary core
    TRISBbits.TRISB6 = 1;
    TRISBbits.TRISB7 = 1;
    
    // PWM parameters
    g_maxPWMInteger = 12799;            // maximum PWM integer allowed
    //PWM velocity and position output parameters
    g_pwm1ZeroOffset = 6756;
    g_pwm2ZeroOffset = 6756;
    g_pwm3ZeroOffset = 6758;
    g_encoderToPwmDenom = 50;           // Encoder has 4000 steps per turn of motor (not geared output shaft). 
                                        // Note this gets overwritten in configureDerivedQuantities()
    
    // Timer1 interrupt period sets the update rate of the feedback loop
    //With instruction cycle at 64 MIPS and prescaler set to 256:1, 2500 is every 10 milliseconds.
    g_feedbackHalfUpdatePeriod = 2500;
    
    // Waveform parameters
    g_waveformType = 0;                 // 0-sine wave; 1-Pulse
    g_waveformUpdatePeriod = 200;       // Number of PWM1 periods between each waveform index advance
    //g_numArrayVals = 400;             // number of array values in output waveform. Will be overwritten
    g_motionAmplitudeMM1 = 1;           // motion amplitude in mm. Small default value to avoid physical damage
    g_motionAmplitudeMM2 = 1;           // motion amplitude in mm. Small default value to avoid physical damage
    g_reverseDirection1 = 0;            // 0 for forward motion sign, 1 for reverse motion sign
    g_reverseDirection2 = 0;            // 0 for forward motion sign, 1 for reverse motion sign
    g_freqUser = 20;                    // cycles per minute. Will be overwritten by software.
    
    // Quadrature Encoders. Initialize to halfway through their range and set landmarks to this position.
    // ...motor 1
    g_encoder1ZeroPos = 0x7FFF;      // set to halfway to max 32-bit unsigned. Leaves plenty of room in either direction without rollover
    g_encoder1ZeroPos = (g_encoder1ZeroPos << 16);
    g_landmark1Position = g_encoder1ZeroPos;    
    // write zero position offset to position counter - Not necessary anymore, since we initialize position to 0x7FFF in quad 
    // encode configuration. TO DO: delete when verified.
//    POS1HLD = 0x7FFF;
//    POS1CNTL = 0;                   // this will transfer msb to POS1CNTH
    
    // ...motor 2
    g_encoder2ZeroPos = 0x7FFF;      // set to halfway to max 32-bit unsigned. Leaves plenty of room in either direction without rollover
    g_encoder2ZeroPos = (g_encoder2ZeroPos << 16);
    g_landmark2Position = g_encoder2ZeroPos;    
    
    g_displacement1Demand = 0;       // Set the demand to the current position so that no initial output is created from feedback loop
    g_displacement2Demand = 0;       // Set the demand to the current position so that no initial output is created from feedback loop
    g_encoderStepsPerMM = 500;
    g_maxDisplacementMM = 50;       // This will be overwritten by software
 
    // Feedback Parameters. For safety, initialize for no signal. Will be replaced at runtime
    g_propConstNum1 = 0;
    g_propConstDenom1 = 10;
    g_intConstNum1 = 0;
    g_intConstDenom1 = 100;
    g_derivConstNum1 = 0;
    g_derivConstDenom1 = 100;
    
    g_propConstNum2 = 0;
    g_propConstDenom2 = 10;
    g_intConstNum2 = 0;
    g_intConstDenom2 = 100;
    g_derivConstNum2 = 0;
    g_derivConstDenom2 = 100;
    
//    INTCON1bits.NSTDIS = 1;  //disable nested interrupts
 //   IPC4bits.SI2C1IP = 3;    // give I2c a higher interrupt priority than Timer1, which has a natural IP of 4
    
    // Initialize error flags
    g_errorFlags = 0;
    g_faultDetected = false;
 
    
}

void configureAnalogToDigital() {
    
    // Configure analog inputs pins as analog input
    ANSELAbits.ANSELA1 = 1;
    TRISAbits.TRISA1 = 1;
    
    ANSELAbits.ANSELA2 = 1;
    TRISAbits.TRISA2 = 1;
    
    ANSELAbits.ANSELA3 = 1;
    TRISAbits.TRISA3 = 1;
    
    // TO DO: ADC module configuration and initialization
 
}

void configurePPS() {
    // Configure Peripheral Pin Select. This must be done before any application code is executed
    
    // Unlock control register
    __builtin_write_RPCON(0x0000);
    
    // Assign RB7 to Quadrature Input A and RB6 to Quadrature Input B
    RPINR14bits.QEIA1R = 39;
    RPINR14bits.QEIB1R = 38;
    
    // Lock control register
    __builtin_write_RPCON(0x0800);
}

void configureQuadEncoder() {
    QEI1CONbits.QEIEN = 1;  // Enable quad encoder module
    QEI1CONbits.PIMOD = 0;  // Index input does not affect counter
    
    // Initialize counter to approximately halfway through its full range to avoid rolling over
    // counter is 32 bits, so halfway through in hex is 0x7FFF FFFF
    POS1CNTH = 0x0100;      // This line seems unnecessary and may be a relic of testing. Test without it at some point
    POS1HLD = 0x7FFF;       // Write high bit to Position 1 Counter Hold Register
    POS1CNTL = 0x0000;      // This write transfers POS1HLD into POS1CNTLH
    
    // Here POS1CNTH should be 0x7FFF
    // TO DO: this line was not commented in Mov1a prototype....I believe it was a relic of a test and
    // should be deleted once things are working
//    POS1HLD = 0x0300;  // reset it to some other number
       

}

void configureI2C() {
    //Configure I2C Module
    
    // Temp - make the I2c pins digital inputs until implementing as I2C
    TRISBbits.TRISB8 = 1;
    TRISBbits.TRISB9 = 1;
    
//    //...enable I2C master interrupts
    IFS1bits.SI2C1IF = 0;   // clear interrupt flag
    IEC1bits.SI2C1IE = 1;   // enable I2C interrupts
    INTCON2bits.GIE  = 1;    //global interrupt enable   
    
    I2C1CONLbits.STREN = 1; //enable clock stretching
    I2C1CONHbits.SDAHT = 1; //minimum 300 ns hold time after falling clock edge
    I2C1ADD = 111;          //pick an address for the slave module
    I2C1CONLbits.I2CEN = 1; //enable I2C module
}

void configureDerivedQuantities() {
    // Calculate quantities that depend upon other configuration parameters. This function  should be run after all other configuration function
    
    // Adjust waveform update time (time between index advances) based on the frequency requested. The goal is to have a 10 ms update time
    // for waveforms with a period of less than 8 seconds, 20 ms for 8-16, and 40 ms for periods longer than 16s. Assuming that the max pwm1
    // integer is 12799, and the input clocking is set up so that this corresponds to 50 microseconds per interrupt time, then a value
    // of g_waveformUpdatePeriod of 200 gives 10 ms update time.
    
    // Define some thresholds for the user-entered frequency in cycles/minute that will be used to set the waveform update period
//    uint16_t thresh1 = 60;  // period of 1 sec
//    uint16_t thresh2 = 8;   // period of 7.5 sec
//    uint16_t thresh3 = 4;   //period of 15 sec
//    uint16_t thresh4 = 2;   // period of 30 sec
    // Temporary threshold settings for older MCU
    uint16_t thresh1 = 60;  // period of 1 sec
    uint16_t thresh2 = 19;   // period of 3.16 sec
    uint16_t thresh3 = 10;   //period of 6 sec
    uint16_t thresh4 = 5;   // period of 12 sec
    
    if(g_freqUser > thresh1) {
        g_waveformUpdatePeriod = 100;  // targets 5 ms update period assuming PWM1 max integer of 12799
    }
    else if(g_freqUser > thresh2)
    {
        g_waveformUpdatePeriod = 200;  // targets 10 ms update period assuming PWM1 max integer of 12799
    }
    else if(g_freqUser > thresh3) {
        g_waveformUpdatePeriod = 400; // targets 20 ms update period assuming PWM1 max integer of 12799
    }
    else if(g_freqUser > thresh4) {
        g_waveformUpdatePeriod = 800; // targets 40 ms update period assuming PWM1 max integer of 12799
    }
    else {
        g_waveformUpdatePeriod = 1600; // targets 80 ms update period assuming PWM1 max integer of 12799
    }
    
    // Configures the PWM-output-to-position encoder
    g_encoderToPwmDenom = (int16_t)(  (float)(g_maxDisplacementMM) * (float)(g_encoderStepsPerMM) / (float)g_maxPWMInteger + 0.5);
    
    // Calculates the time between each step in the waveform playout. Depends upon configuration of the PWM and the number of interrupts.
    // This will determine how many points are in the waveform array
    g_waveformTimeStep_microS = (uint32_t)(g_maxPWMInteger + 1) * (uint32_t)g_waveformUpdatePeriod / (uint32_t)(g_OscillatorFreq*2 / 1000000);
    calcNumPoints();
}

void calcNumPoints() {
    // Calculate the number of points in one period of the waveform (one pass through the waveform array). The intention is that the caller
    // of this function knows the value of g_waveformTimeStep_microS (update time of waveform index)
    // and ensures that the period of the waveform array is an integer number of waveform update time steps. This function will round to the
    // nearest number of points just in case.
    
    float period_microSec =  60.0 * 1000000.0 / (float)g_freqUser;
    g_numArrayVals = (uint16_t)( period_microSec / (float)g_waveformTimeStep_microS + 0.5 );
}
