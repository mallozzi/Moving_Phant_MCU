
#include <xc.h>
#include <libpic30.h>
#include <stdlib.h>
#include "globals_slave.h"
#include "motorCntrlSlv.h"
#include "MathUtil.h"
#include "PWMcontrol.h"



void setMotorOutput(int16_t pwmSignedDutyCycleInt) {
    // Sets the motor PWM and direction. Applies low-pass filter to output. Clips output to max possible if input is out of range.
    // INPUT
    // pwmSignedDutyCycleInt is the duty cycle integer for the PWM output. Negative values get converted to positive output with the 
    //      direction output set accordingly.
    static int16_t oldPwmVal = 0;
    static int16_t newPwmVal = 0;
    static uint8_t dir=1;
    static uint16_t pwmDutyCycleInt=0;
    static int16_t oldSignedPwmDutyCycleInt=0;
    
    // Low-pass filter the changes
    newPwmVal = MultiplyByFraction(oldPwmVal-oldSignedPwmDutyCycleInt, gs_filtNumerator, gs_filtDenominator) + oldSignedPwmDutyCycleInt;
    oldPwmVal = newPwmVal;
    oldSignedPwmDutyCycleInt = pwmSignedDutyCycleInt;
    
    // Convert negative values to a direction and create a positive duty cycle integer
    if(newPwmVal >= 0) {
        dir=1;              // which direction is positive vs negative depends upon how things are hooked up.
        pwmDutyCycleInt = (uint16_t)newPwmVal;
    }
    else {
        dir=0;
        pwmDutyCycleInt = (uint16_t)(-newPwmVal);
    }
    
    // Cut off duty cycle integer if it goes out of range
    if(pwmDutyCycleInt > gs_maxPWMInteger) {
        pwmDutyCycleInt = gs_maxPWMInteger;
    }
    
//    if(pwmDutyCycleInt > 0) {
//        LATBbits.LATB1 = 1;
//    }
    
    INTCON2bits.GIE = 0;                // disable global interrupts
    LATBbits.LATB15 = dir;              // set direction
    setOnCyclesPWM1(pwmDutyCycleInt);   // actual direct setting of motor output voltage
    INTCON2bits.GIE = 1;                //re-enable global interrupt 
    
}