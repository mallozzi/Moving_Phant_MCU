#include "Configure.h"
#include <xc.h>
#include "globals.h"
//#include "PWMcontrol.h"
#include <stdbool.h>
#include <p33CH128MP502.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

void configureInitial() {
// Initial configuration. It is assumed that this method is called before other configurations are done, as it broadly sets a lot of
// config states to a rational starting value, assuming they will later be set by other configuration calls.
    
    // Start by making all I/O pins digital outputs in their low state, for safety 
    ANSELA = 0;
    TRISA = 0;
    LATA = 0;
    ANSELB = 0;
    TRISB = 0;
    LATB = 0;
    
    INTCON1bits.NSTDIS = 1;  //1 to disable nested interrupts
    IPC4bits.SI2C1IP = 5;    // give I2c a higher interrup priority than Timer1, which has a natural IP of 4
    
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
    
    // Set low-pass filter parameters
 //   g_filtNumerator=97;            // low-pass PWM filter parameter defined as integer numerator and denominator
 //   g_filtDenominator=100; 
    
    
    //PWM velocity and position output parameters
 //   g_zeroPosOutput = false;
   // g_playSingleWaveformOnly = false;
    g_pwm2ZeroOffset = 6756;
    g_pwm3ZeroOffset = 6758;
    g_encoderToPwmDenom = 50;  // Encoder has 4000 steps per turn of motor (not geared output shaft). 
                               // Note this gets overwritten in configureDerivedQuantities()
    
    // Timer1 interrupt period sets the update rate of the feedback loop
    //With instruction cycle at 64 MIPS and prescaler set to 256:1, 2500 is every 10 milliseconds.
    g_feedbackUpdatePeriod = 2500;
    
    // Waveform parameters
    g_waveformType = 0;             // 0-sine wave; 1-Pulse
    g_waveformUpdatePeriod = 200;   // Number of PWM1 periods between each waveform index advance
    //g_numArrayVals = 400;           // number of array values in output waveform. Will be overwritten
    g_motionAmplitudeMM = 1;        // motion amplitude in mm. Small default value to avoid physical damage
    g_reverseDirection = 0;         // 0 for forward motion sign, 1 for reverse motion sign
    g_freqUser = 20;                // cycles per minute. Will be overwritten by software.
    
    // Quadrature Encoder
    g_encoderZeroPos = 0x7FFF;      // set to halfway to max 32-bit unsigned. Leaves plenty of room in either direction without rollover
    g_encoderZeroPos = (g_encoderZeroPos << 16);
    g_landmarkPosition = g_encoderZeroPos;    
    //write zero position offset to position counter
    POS1HLD = 0x7FFF;
    POS1CNTL = 0;                   // this will transfer msb to POS1CNTH
    g_displacementDemand = 0;       // Set the demand to the current position so that no initial output is created from feedback loop
    g_encoderStepsPerMM = 500;
    g_maxDisplacementMM = 50;       // This will be overwritten by software
 
    // Feedback Parameters. For safety, initialize for no signal. Will be replaced at runtime
    g_propConstNum = 0;
    g_propConstDenom = 10;
    g_intConstNum = 0;
    g_intConstDenom = 100;
    g_derivConstNum = 0;
    g_derivConstDenom = 100;
    
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

void configureEncoderInputs() {

    // Set digital input pins
    TRISBbits.TRISB6 = 1;
    TRISBbits.TRISB7 = 1;
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



void configureDirection() {
    // direction output pin is RB15
    TRISBbits.TRISB15 = 0;      // set to digital output
    LATBbits.LATB15 = 1;        // set initially to positive direction (arbitrary)
}

void configureQuadEncoder() {
    QEI1CONbits.QEIEN = 1;  // Enable quad encoder module
    QEI1CONbits.PIMOD = 0;  // Index input does not affect counter
    
    // Initialize counter to approximately halfway through its full range to avoid rolling over
    // counter is 32 bits, so halfway through in hex is 0x7FFF FFFF
    POS1CNTH = 0x0100;
    POS1HLD = 0x7FFF;  // Write high bit to Position 1 Counter Hold Register
    POS1CNTL = 0x0002;
    
    // Here POS1CNTH should be 0x7FFF
    POS1HLD = 0x0300;  // reset it to some other number
    
 //   uint16_t aa = POS1CNTL;  // Reading this should load POS1CNTH to POS1HLD
    

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
    
    // Non-integer number of times the velocity is read in one advancement of the waveform index.
 //   g_velReadsPerWfUpdate = (float)(g_maxPWMInteger + 1) * g_waveformUpdatePeriod / (4 * (float)g_feedbackUpdatePeriod * g_timer1Prescale);
    
    // Configures the PWM-output-to-position encoder
    g_encoderToPwmDenom = (int16_t)(  (float)(g_maxDisplacementMM) * (float)(g_encoderStepsPerMM) / (float)g_maxPWMInteger + 0.5);
    
    // Calculates the time between each step in the waveform playout. Depends upon configuration of the PWM and the number of interrupts.
    // This will determine how many points are in the waveform array
    g_waveformTimeStep_microS = (uint32_t)(g_maxPWMInteger + 1) * (uint32_t)g_waveformUpdatePeriod / (uint32_t)(g_OscillatorFreq*2 / 1000000);
//    if(g_maxPWMInteger == 0) {
//        LATBbits.LATB2 = 1;
//    }    
    // Set the number of points in the waveform array
    calcNumPoints();
}

//int32_t* makePosSineWaveform(int32_t amplitudeEncPP, uint16_t numValues) {
//    // Allocates a sine waveform of a given amplitude number of values numValues. Also allocates the global zero waveform of the same length.
//    // INPUTS
//    // amplitudeEncPP is the peak-to-peak amplitude of the waveform in units of encoder steps
//    // numValues is the number of values in the array.
//    uint16_t ii;
//    float x;
//    float step = 2*3.14159265359 / numValues;
//    float amplitude;                            // true encoder amplitude rather than peak-to-peak
//    
//    amplitude = (float)amplitudeEncPP / 2.0;
//    if(g_zeroWaveform != NULL) {
//        free(g_zeroWaveform);
//    }
//    
//    // Allocate memory. Note: to use dynamic memory allocation, a heap must be 
//    //  ...created in the linker. See notes on project
//    int32_t* waveformArray = (int32_t*)malloc(numValues*sizeof(int32_t));
//    for(ii=0; ii<numValues; ii++) {
//        x = amplitude * sin(ii*step);
//        waveformArray[ii] = (int32_t)(x+0.5);
//    }
//    
////    g_zeroWaveform = makeZeroWaveform(numValues);  // create corresponding zero waveform
//    g_numArrayVals = numValues;
//    
//    return waveformArray;  
//}
//
//
//int32_t* makeVelSineWaveform(int32_t posAmplitude, uint16_t numValues) {
//    // Allocates a sine waveform of a given amplitude number of values numValues. Also allocates the global zero waveform of the same length.
//    // This is designed as a velocity-controlled waveform, so the amplitude is estimated so that the position
//    // displacement is the posAmplitude input.
//    // INPUTS
//    // posAmplitude is the target position displacement (0-peak) in units of encoder steps 
//    uint16_t ii;
//    float x;
//    float step = 2*3.14159265359 / numValues;
//    float amplitude;
//    
//    // scale amplitude to make the position amplitude (0-peak) equal to the posAmplitude input
//    // A half sine wave has average value of 2/pi = 0.636953.
//     amplitude = (float)posAmplitude / numValues / g_velReadsPerWfUpdate / 0.635953;
//    //amplitude = (float)posAmplitude / numValues / 0.635953;
//    
//    
//    if(g_zeroWaveform != NULL) {
//        free(g_zeroWaveform);
//    }
//    
//    // Allocate memory. Note: to use dynamic memory allocation, a heap must be 
//    //  ...created in the linker. See notes on project
//    int32_t* waveformArray = (int32_t*)malloc(numValues*sizeof(int32_t));
//    for(ii=0; ii<numValues; ii++) {
//        x = amplitude * sin(ii*step);
//        waveformArray[ii] = (int32_t)(x+0.5);
//    }
//    
// //   g_zeroWaveform = makeZeroWaveform(numValues);  // create corresponding zero waveform
//    g_numArrayVals = numValues;
//    
//    return waveformArray;  
//}
//
//int32_t* makeRampWaveform(int32_t stepSize, uint16_t numValues) {
//    // Creates a waveform meant to move the motor a defined amount. The last 10%
//    // of the waveform sits steady to allow the motor to settle.
//    // INPUTS
//    // stepSize is the number of encoder steps to move the motor
//    // numValues is the number of values in the array
//    uint16_t ii;
//    float rampValue;
//    float rampIncrement;
//    uint16_t settleInd;         // index where ramping is finished to allow settling time
//    
//    if(g_zeroWaveform != NULL) {
//        free(g_zeroWaveform);
//    }
//
//    // Allocate memory. Note: to use dynamic memory allocation, a heap must be 
//    //  ...created in the linker. See notes on project
//    int32_t* waveformArray = (int32_t*)malloc(numValues*sizeof(int32_t));
//    
//    settleInd = (uint16_t)(numValues * 0.9);        // float operation instead of multiplying by 9/10 in case someone makes very long array causing overflow
//    rampIncrement = (float)(stepSize) / settleInd;
//    rampValue = 0;
//    
//    for(ii=0; ii<numValues; ii++) {
//        if(ii < settleInd) {
//            waveformArray[ii] = rampValue;
//            rampValue+= rampIncrement;
//        }
//        else {
//            waveformArray[ii] = rampValue;          // constant for last portion of array
//        }
//    }
//    
// //   g_zeroWaveform = makeZeroWaveform(numValues);
//    g_numArrayVals = numValues;
// //   g_playSingleWaveformOnly = true;
//    return waveformArray;
//}
//
//int32_t* makeWideVelPulseWaveform(int32_t totalSteps, uint16_t numValues) {
//    // Allocates an inverted cosine waveform of a given amplitude number of values numValues. 
//    // Waveform is one period, starting and ending at 0. The waveform is intended as a velocity
//    // waveform, with the total number of steps targeted to be equal to the totalSteps input
//    // Also allocates the global zero waveform of the same length.
//    // INPUTS
//    // totalSteps is the total encoder steps to take in units of encoder steps. This will be used as a target
//    //      target in calculating the pulse amplitude, assuming this is a velocity waveform
//    uint16_t ii;
//    float x;
//    float step = 2*3.14159265359 / numValues;
//    float amplitude;
//    
//    //amplitude = (float)totalSteps / (g_velReadsPerWfUpdate * numValues);
//    amplitude = (float)totalSteps / ( numValues);
//    
//    if(g_zeroWaveform != NULL) {
//        free(g_zeroWaveform);
//    }
//    
//    // Allocate memory. Note: to use dynamic memory allocation, a heap must be 
//    //  ...created in the linker. See notes on project
//    int32_t* waveformArray = (int32_t*)malloc(numValues*sizeof(int32_t));
//    for(ii=0; ii<numValues; ii++) {
//        x = -amplitude * (cos(ii*step)-1.0);
//        waveformArray[ii] = (int32_t)(x+0.5);
//    }
//    
////    g_zeroWaveform = makeZeroWaveform(numValues);  // create corresponding zero waveform
//    g_numArrayVals = numValues;
//    return waveformArray;  
//}
//
//int32_t* makeZeroWaveform(uint16_t numValues) {
//    // Creates a waveform of zeros that is useful in bringing output to zero gracefully. Returns a pointer to the 
//    // newly-allocated array of zeros
//    
//    uint16_t ii;
//    int32_t* waveformArray = (int32_t*)malloc(numValues*sizeof(int32_t));
//    for(ii=0; ii<numValues; ii++) {
//        waveformArray[ii] = 0;
//    }
//    
//    return waveformArray;
//}
//
//int32_t* makeConstWaveform(int32_t value, uint16_t numValues) {
//    // Creates a waveform of constant values 
//    
//    uint16_t ii;
//    int32_t* waveformArray = (int32_t*)malloc(numValues*sizeof(int32_t));
//    for(ii=0; ii<numValues; ii++) {
//        waveformArray[ii] = value;
//    }
//    
//    return waveformArray;
//}

void calcNumPoints() {
    // Calculate the number of points in one period of the waveform (one pass through the waveform array). The intention is that the caller
    // of this function knows the value of g_waveformTimeStep_microS (update time of waveform index)
    // and ensures that the period of the waveform array is an integer number of waveform update time steps. This function will round to the
    // nearest number of points just in case.
    
    float period_microSec =  60.0 * 1000000.0 / (float)g_freqUser;
    g_numArrayVals = (uint16_t)( period_microSec / (float)g_waveformTimeStep_microS + 0.5 );
//    if(period_microSec > 100000) {
//        LATBbits.LATB2 = 1;
//    }
}


//void configurePPS() {
//    // Configure Peripheral Pin Select. This must be done before any application code is executed
//    
//    // Unlock control register
//    __builtin_write_RPCON(0x0000);
//    
//    // Set the remappable pin inputs for Input Capture:
//    RPINR3bits.ICM1R = 45;      // SCCP Capture 1 to RP45, which corresponds to pin RB13
//    RPINR4bits.ICM2R = 44;      // SCCP Capture 2 to RP44, which corresponds to pin RB12
//    RPINR5bits.ICM3R = 43;      // SCCP Capture 3 to RP43, which corresponds to pin RB11
//    RPINR6bits.ICM4R = 38;      // SCCP Capture 4 to RP38, which corresponds to pin RB6
//    RPINR7bits.ICM5R = 39;      // SCCP Capture 5 to RP39, which corresponds to pin RB7
//            
//    // Lock control register
//    __builtin_write_RPCON(0x0800);
//}

//void configureInputCapture() {
//    // Configures Input Capture mode on multiple pins. Assumes PPS has already been configured
//    
//    // *** Capture 1 module
//    TRISBbits.TRISB13 = 1;              // Set to digital input
//    CCP1CON1Lbits.CCPON = 0;            // Disable module before making changes. Probably not necessary
//    CCP1CON1Lbits.CCSEL = 1;            // Select Input Capture mode
//    CCP1CON1Lbits.CLKSEL = 0;           // Set Fosc/2 as the clock source 
//    CCP1CON1Lbits.MOD = 0;              // Edge detect mode (a bit different from every rising/falling edge mode in how overflow handled)
//    CCP1CON1Lbits.T32 = 1;              // Set timer to 32 bit mode
//    CCP1CON2Hbits.ICS = 0;              // Set Input Capture source to IC1 pin (from PPS)
//    CCP1CON1Lbits.CCPON = 1;            // Enable the module
//    
//    // Interrupts for Capture 1 module
//    IEC0bits.CCP1IE = 1;                // Enable Capture Compare interrupt
//    IFS0bits.CCP1IF = 0;                // Clear Capture 1 interrupt flag
//    
//    // *** Capture 2 module
//    TRISBbits.TRISB12 = 1;              // Set to digital input
//    CCP2CON1Lbits.CCPON = 0;            // Disable module before making changes. Probably not necessary
//    CCP2CON1Lbits.CCSEL = 1;            // Select Input Capture mode
//    CCP2CON1Lbits.CLKSEL = 0;           // Set Fosc/2 as the clock source 
//    CCP2CON1Lbits.MOD = 0;              // Edge detect mode (a bit different from every rising/falling edge mode in how overflow handled)
//    CCP2CON1Lbits.T32 = 1;              // Set timer to 32 bit mode
//    CCP2CON2Hbits.ICS = 0;              // Set Input Capture source to IC2 pin (from PPS)
//    CCP2CON1Lbits.CCPON = 1;            // Enable the module
//    
//    // Interrupts for Capture 2 module
//    IEC1bits.CCP2IE = 1;                // Enable Capture Compare interrupt
//    IFS1bits.CCP2IF = 0;                // Clear Capture 2 interrupt flag
//    
//    // *** Capture 3 module
//    TRISBbits.TRISB11 = 1;              // Set to digital input
//    CCP3CON1Lbits.CCPON = 0;            // Disable module before making changes. Probably not necessary
//    CCP3CON1Lbits.CCSEL = 1;            // Select Input Capture mode
//    CCP3CON1Lbits.CLKSEL = 0;           // Set Fosc/2 as the clock source 
//    CCP3CON1Lbits.MOD = 0;              // Edge detect mode (a bit different from every rising/falling edge mode in how overflow handled)
//    CCP3CON1Lbits.T32 = 1;              // Set timer to 32 bit mode
//    CCP3CON2Hbits.ICS = 0;              // Set Input Capture source to IC3 pin (from PPS)
//    CCP3CON1Lbits.CCPON = 1;            // Enable the module
//    
//    // Interrupts for Capture 3 module
//    IEC2bits.CCP3IE = 1;                // Enable Capture Compare interrupt
//    IFS2bits.CCP3IF = 0;                // Clear Capture 3 interrupt flag
//    
//    // *** Capture 4 module
//    TRISBbits.TRISB6 = 1;               // Set to digital input
//    CCP4CON1Lbits.CCPON = 0;            // Disable module before making changes. Probably not necessary
//    CCP4CON1Lbits.CCSEL = 1;            // Select Input Capture mode
//    CCP4CON1Lbits.CLKSEL = 0;           // Set Fosc/2 as the clock source 
//    CCP4CON1Lbits.MOD = 0;              // Edge detect mode (a bit different from every rising/falling edge mode in how overflow handled)
//    CCP4CON1Lbits.T32 = 1;              // Set timer to 32 bit mode
//    CCP4CON2Hbits.ICS = 0;              // Set Input Capture source to IC4 pin (from PPS)
//    CCP4CON1Lbits.CCPON = 1;            // Enable the module
//    
//    // Interrupts for Capture 4 module
//    IEC2bits.CCP4IE = 1;                // Enable Capture Compare interrupt
//    IFS2bits.CCP4IF = 0;                // Clear Capture 4 interrupt flag
//    
//    // *** Capture 5 module
//    TRISBbits.TRISB7 = 1;               // Set to digital input
//    CCP2CON1Lbits.CCPON = 0;            // Disable module before making changes. Probably not necessary
//    CCP2CON1Lbits.CCSEL = 1;            // Select Input Capture mode
//    CCP2CON1Lbits.CLKSEL = 0;           // Set Fosc/2 as the clock source 
//    CCP2CON1Lbits.MOD = 0;              // Edge detect mode (a bit different from every rising/falling edge mode in how overflow handled)
//    CCP2CON1Lbits.T32 = 1;              // Set timer to 32 bit mode
//    CCP2CON2Hbits.ICS = 0;              // Set Input Capture source to IC5 pin (from PPS)
//    CCP2CON1Lbits.CCPON = 1;            // Enable the module
//    
//    // Interrupts for Capture 5 module
//    IEC2bits.CCP5IE = 1;                // Enable Capture Compare interrupt
//    IFS2bits.CCP5IF = 0;                // Clear Capture 5 interrupt flag
//    
//}