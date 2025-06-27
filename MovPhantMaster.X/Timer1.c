#include <xc.h>

#include "Timer1.h"
#include "globals.h"


//USAGE NOTES
//call configureTimer1() at beginning of code to take care of precursors. The prescaler should be set in that function.
//The timer will not start, and interrupts will not be turned on. Timer1 can be used from there by calling startTimer1_instrCycl,
//which will set the timing period, enable the timer, turn on the interrupt, and start it.


void configureTimer1() {
    //Configure Timer1, but does not start it or enable the interrupts.
    //The 'Peripheral Bus Clock' referred to in the datasheets seems to be half the oscillator frequency, which
    //happens to be the same as the instruction cycle frequency unless the CLKDIV.DOZE bits are used to slow down 
    //the instruction cycle frequency
    //With the above assumption on CLKDIV.DOZE, one cycle of Timer1 corresponds to two cycles of the oscillator. If the
    //oscillator is 128MHz, then a timer1 cycle is 2*(prescaler value)/(128,000,000) = 2*256/128,000,000 = .004 ms.....
    //in other words, 250 timer1 cycles per ms.
    
    T1CONbits.TON = 0;      //disable before doing anything else
    T1CONbits.TCS = 0;      //user internal peripheral bus clock source (same as instruction cycle, Fosc/2)
    T1CONbits.TCKPS = 3;    //pre-scaler. Value of 3 (0b11) is 256:1. Be sure to update g_timer1Prescale if changed.
    g_timer1Prescale = 256; //pre-scaler value, set for use elsewhere in calculations
    TMR1 = 0;               //clear the Timer1 register
    PR1 = 12500;            //Timer1 period. With instruction cycle at 64 MIPS and prescaler set to 256:1, 2500 is every 10 milliseconds.
    IEC0bits.T1IE = 0;      //disable Timer1 interrupt
    INTCON2bits.GIE = 1;    //Enable global interrupts
    IFS0bits.T1IF = 0;      //clears interrupt flag
}


void startTimer1(uint16_t nCycles) {
    //sets Timer1 to nCycle Timer1 cycles, enables it, and starts it
    PR1 = nCycles;          //match period in Timer1 cycles. One Timer1 cycle depends upon Timer1 configuration parameters.
    TMR1 = 0;               //clear the Timer1 register
    IFS0bits.T1IF = 0;      //clear interrupt flag
    IEC0bits.T1IE = 1;      //enable Timer1 interrupt
    T1CONbits.TON = 1;      //enables timer
}

