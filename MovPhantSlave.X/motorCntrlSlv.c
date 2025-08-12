
#include <xc.h>
#include <libpic30.h>
#include <stdlib.h>
#include "globals_slave.h"
#include "motorCntrlSlv.h"
#include "MathUtil.h"
#include "PWMcontrol.h"



void setMotorOutput1(int16_t pwmSignedDutyCycleInt) {
    // Sets the motor PWM and direction. Applies low-pass filter to output. Clips output to max possible if input is out of range.
    // Motor 1 is controlled by PWM2
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
    
    
    INTCON2bits.GIE = 0;                // disable global interrupts
    LATCbits.LATC5 = dir;              // set direction. Do this directly for maximal speed rather than call function
    setOnCyclesPWM1(pwmDutyCycleInt);   // sets motor output voltage. PWM1 is motor1, so this actually sets the pwm 2 module
    INTCON2bits.GIE = 1;                // re-enable global interrupt 
    
}

void setMotorOutput2(int16_t pwmSignedDutyCycleInt) {
    // Sets the motor PWM and direction. Applies low-pass filter to output. Clips output to max possible if input is out of range.
    // Motor 2 is controlled by PWM1
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
        // TEMP: Reverse direction for miswiring. TODO: remove this when done
       // dir=1;              // which direction is positive vs negative depends upon how things are hooked up.
        dir=0;              // which direction is positive vs negative depends upon how things are hooked up.
        pwmDutyCycleInt = (uint16_t)newPwmVal;
    }
    else {
        //dir=0;                // This is the correct one
        dir=1;                  // temporary workaround for miswiring of encoder on motor 2 (green)
        pwmDutyCycleInt = (uint16_t)(-newPwmVal);
    }
    
    // Cut off duty cycle integer if it goes out of range
    if(pwmDutyCycleInt > gs_maxPWMInteger) {
        pwmDutyCycleInt = gs_maxPWMInteger;
    }
    
    INTCON2bits.GIE = 0;                // disable global interrupts
    LATCbits.LATC11 = dir;               // set direction. Do this directly for maximal speed rather than call function
    setOnCyclesPWM2(pwmDutyCycleInt);   // sets motor output voltage. Remember PWM1 is motor 1, which is pwm 2 module
    INTCON2bits.GIE = 1;                //re-enable global interrupt 
    
}

uint32_t readQuadEncoderPos() {
    // Reads the quadrature encoder position and returns a 32 bit unsigned integer
    // variables used to temporarily hold quadrature encoder bytes while reading
    static uint16_t posLowByte;
    static uint32_t posHighByte;
    static uint32_t position;
    
    // Read position register and update pwm position output
    posLowByte = POS1CNTL; // Should load POS1CNTH into POS1HLD
    posHighByte = POS1HLD;
    position = ( ((uint32_t)posHighByte) <<16) + (uint32_t)posLowByte;
    return position;
}

int16_t readEncoderVelocity() {
    static int16_t velocity;
    static uint16_t speedRegister;
    speedRegister = VEL1CNT;   
    velocity = (int16_t) speedRegister;
    if (speedRegister >= 0x7FFF) {   // speed was actually negative
        // take the two's complement negative
        speedRegister = ~speedRegister + 1;   // two's complement
        velocity = -(int16_t)speedRegister;
    }
    
    return velocity;
}