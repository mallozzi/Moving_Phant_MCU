/* 
 * File:   Configure.h
 * Author: richa
 *
 * Created on September 26, 2019, 6:03 PM
 */

#ifndef CONFIGURE_H
#define	CONFIGURE_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#include <stdint.h>

void configureInitial();
void configurePPS();
void configureAnalogToDigital();
void configureDerivedQuantities();
void configureI2C();
void configureQuadEncoder();
void configureInterruptOnChange();
void calcNumPoints();

#endif	/* CONFIGURE_H */

