
#include <xc.h>
#include "PWMcontrol.h"
#include "globals_slave.h"

void configurePWM1() {
// Configure Motor1 driver PWM, but do not start it. Also sets up interrupts. The name PWM1 is a legacy name, and the 1 should be viewed as Motor 1, not PWM 1.
// The Motor 1 driver PWM is PWM2
    
    //Configure PWM module clock. This setup uses the PLL post divider output as the clock source, which runs at twice the oscillator frequency.
    //...PCLKCON applies to entire PWM module and is set just once
    PCLKCONbits.MCLKSEL = 2; //Primary PLL post divider output, which is twice the oscillator frequency. This is common among all the PWM modules
    //... 
    PG2CONLbits.CLKSEL = 1; //PWM clock source is the same clock selected by MCLKSEL, which for MCLKSEL=0 is the FRC oscillator
    
    //Set PWM mode and turn on the output pin
    PG2CONLbits.MODSEL = 0; //independent edge mode    
    PG2IOCONH = 0x0008; //PWM module controls high output pin only
    
    PG2EVTLbits.UPDTRG = 1; // write to PG1DC automatically triggers update request
    
    
    //set period 
    gs_maxPWMInteger = 12799;            // maximum PWM integer allowed
    PG2PER = gs_maxPWMInteger; //PWM period is PG1PER+1 PWM module clock cycles. Value of 12799 is designed to give period of 12800 / 256 MHz = 50 micro-seconds
    PG2PHASE = 0; //no offset from start of PWM period
    PG2DC = 0; //width of pulse in clock cycles. Initial value of zero would give no output
    
    //Set up PWM1 interrupts. PWM interrupts come at the end of each PWM period.
    IFS4bits.PWM2IF = 0; //Clear PWM1 interrupt flag
    IEC4bits.PWM2IE = 1; //Enable PWM1 interrupt      ********************DON'T FORGET TO RE-ENABLE THE INTERRUPT !!!!!!!!!!!!!!! ****************
    INTCON2bits.GIE = 1; //global interrupt enable   
    
    //Make sure PWM1 is disabled. It will get enabled by the startPWM1() function.
    PG2CONLbits.ON = 0; //PWM1 generator is not enabled by this configuration function
}

void configurePWM2() {
// Configure PWM for Motor 2. Name is a legacy and the 2 should now be interpreted as Motor 2. The actual PWM module is 1
    
    //Configure PWM module clock. This setup uses the PLL post divider output as the clock source, which runs at twice the oscillator frequency.
    //PCLKCONbits.MCLKSEL was set in configuration of the motor 1 PWM (PWM2)
    PG1CONLbits.CLKSEL = 1; //PWM clock source is the same clock selected by MCLKSEL, which for MCLKSEL=0 is the FRC oscillator
    
    //Set PWM mode and turn on the output pin
    PG1CONLbits.MODSEL = 0; //independent edge mode    
    PG1IOCONH = 0x0008; //PWM module controls high output pin only
    
    PG1EVTLbits.UPDTRG = 1; // write to PG1DC automatically triggers update request
    
    
    //set period 
  //  gs_maxPWMInteger = 12799;            // maximum PWM integer allowed
    PG1PER = gs_maxPWMInteger; //PWM period is PG1PER+1 clock cycles. Value of 12799 is designed to give period of 12800 / 256 MHz = 50 micro-seconds
    PG1PHASE = 0; //no offset from start of PWM period
    PG1DC = 0; //width of pulse in clock cycles. Initial value of zero would give no output
    
    //Make sure PWM2 interrupts are disabled
    IEC4bits.PWM1IE = 0; //Do not enable PWM2 interrupt. All motor PWM updating will be done from the Motor 1 pwm interrupt
    
    //Set module to be disabled initially
    PG1CONLbits.ON = 0; 
}
//
//void configurePWM3() {
//// Configure PWM3
//    
//    //Configure PWM module clock. This setup uses the PLL post divider output as the clock source, which runs at twice the oscillator frequency.
//    //PCLKCONbits.MCLKSEL = 2; //Primary PLL post divider output, which is twice the oscillator frequency
//    PG3CONLbits.CLKSEL = 1; //PWM clock source is the same clock selected by MCLKSEL, which for MCLKSEL=0 is the FRC oscillator
//    
//    //Set PWM mode and turn on the output pin
//    PG3CONLbits.MODSEL = 0; //independent edge mode    
//    PG3IOCONH = 0x0008; //PWM module controls high output pin only
//    
//    PG3EVTLbits.UPDTRG = 1; // write to PG1DC automatically triggers update request
//    
//    
//    //set period 
// //   gs_maxPWMInteger = 12799;            // maximum PWM integer allowed
//    PG3PER = gs_maxPWMInteger; //PWM period is PG1PER+1 clock cycles. Value of 12799 is designed to give period of 12800 / 256 MHz = 50 micro-seconds
//    PG3PHASE = 0; //no offset from start of PWM period
//    PG3DC = 0; //width of pulse in clock cycles. Initial value of zero would give no output
//    
//    //Make sure PWM2 interrupts are disabled
//    IEC4bits.PWM3IE = 0; //Do not enable PWM2 interrupt
//    
//    //Enable PWM2
//    PG3CONLbits.ON = 1; 
//}

void setOnCyclesPWM1(uint16_t nCyclesOn) {
    // sets the number of cycles for the on portion of the pulse. If this is set to the same value that is in
    // the PG1PER register, a 100% duty cycle pulse is produced.
    // INPUT
    // nCyclesOn is the number of PWM clock cycles for the on portion of the pulse. Note that the PWM clock is not the same
    //  as the instruction cycle or the main oscillator. For a configuration of the PWM clock as the PLL post divider output 
    //  (see MCLKSEL setting), the PWM clock runs at twice the main oscillator frequency so that one PWM cycle is half of an 
    //  oscillator cycle.
    
    PG2DC = nCyclesOn;  // the 1 in PWM1 is Motor 1. The MCU PWM module is PWM2
}

void setOnCyclesPWM2(uint16_t nCyclesOn) {
    // sets the number of cycles for the on portion of the pulse. If this is set to the same value that is in
    // the PG1PER register, a 100% duty cycle pulse is produced.
    // INPUT
    // nCyclesOn is the number of PWM clock cycles for the on portion of the pulse. Note that the PWM clock is not the same
    //  as the instruction cycle or the main oscillator. For a configuration of the PWM clock as the PLL post divider output 
    //  (see MCLKSEL setting), the PWM clock runs at twice the main oscillator frequency so that one PWM cycle is half of an 
    //  oscillator cycle.
    
    PG1DC = nCyclesOn;   // the 2 in PWM2 is Motor 2. The MCU PWM module is PWM1
}
//
//void setOnCyclesPWM3(uint16_t nCyclesOn) {
//    // sets the number of cycles for the on portion of the pulse. If this is set to the same value that is in
//    // the PG2PER register, a 100% duty cycle pulse is produced. This PWM is used for the velocity output signal.
//    // INPUT
//    // nCyclesOn is the number of PWM clock cycles for the on portion of the pulse. Note that the PWM clock is not the same
//    //  as the instruction cycle or the main oscillator. For a configuration of the PWM clock as the PLL post divider output 
//    //  (see MCLKSEL setting), the PWM clock runs at twice the main oscillator frequency so that one PWM cycle is half of an 
//    //  oscillator cycle.
//    
//    PG3DC = nCyclesOn;
//}


//void setPeriodCyclesPWM1(uint16_t nCyclesPeriod) {
//    // Sets the number of PWM clock cycles for the period of the PWM pulse. See configurePWM1() to see how the PWM clock is configured.
//    PG2PER = nCyclesPeriod;
//}


void startPWM1() {
    // Starts the Motor 1 PWM module (PWM2) running. Must be configured first, and the number of on cycles set.
    IFS4bits.PWM2IF = 0; //clear PWM1 interrupt flag
    PG2CONLbits.ON = 1;
}


void stopPWM1() {
    //Stops the Motor1 PWM module (PWM2) from running
    PG2CONLbits.ON = 0;
}

void startPWM2() {
    // Starts the Motor 2 PWM module (PWM1) running. Must be configured first, and the number of on cycles set.
    IFS4bits.PWM1IF = 0; //clear PWM1 interrupt flag
    PG1CONLbits.ON = 1;
}


void stopPWM2() {
    //Stops the Motor 2 PWM module (PWM1) from running
    PG1CONLbits.ON = 0;
}
