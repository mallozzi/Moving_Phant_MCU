/* 
 * File:   PWMcontrol.h
 * Author: richa
 *
 * Created on September 24, 2019, 3:37 PM
 */

#ifndef PWMCONTROL_H
#define	PWMCONTROL_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#include <stdint.h>

void configurePWM1();
//void configurePWM2();
//void configurePWM3();
void setOnCyclesPWM1(uint16_t nCyclesOn);
//void setOnCyclesPWM2(uint16_t nCyclesOn);
//void setOnCyclesPWM3(uint16_t nCyclesOn);
void setPeriodCyclesPWM1(uint16_t nCyclesPeriod);
void startPWM1();
void stopPWM1();


#endif	/* PWMCONTROL_H */

