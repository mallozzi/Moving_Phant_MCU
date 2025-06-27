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
void configureAnalogToDigital();
void configureEncoderInputs();
void configurePPS();
void configureDerivedQuantities();
void configureDirection();
void configureI2C();
void configureQuadEncoder();
//int32_t* makePosSineWaveform(int32_t amplitudeEnc, uint16_t numValues);
//int32_t* makeVelSineWaveform(int32_t posAmplitude, uint16_t numValues);
//int32_t* makeRampWaveform(int32_t amplitude, uint16_t numValues);
//int32_t* makeWideVelPulseWaveform(int32_t amplitude, uint16_t numValues);
//int32_t* makeZeroWaveform(uint16_t numValues);
//int32_t* makeConstWaveform(int32_t value, uint16_t numValues);
void calcNumPoints();

#endif	/* CONFIGURE_H */

