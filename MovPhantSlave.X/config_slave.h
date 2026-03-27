

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef CONFIG_SLAVE_H
#define	CONFIG_SLAVE_H

#include <xc.h> // include processor files - each processor file is guarded.  


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

void configSlaveInitial();
void configureSecondaryPPS();
void configureQuadEncoder();
//void setOutputWaveformMotor1(int32_t* waveformArray);

void setUpWaveforms();
int32_t* designPosSineWaveform(int16_t mmDisplacementPP);
int32_t* designRampWaveform(int16_t mmStepSize);

int32_t* makePosSineWaveform(int32_t amplitudeEnc, uint16_t numValues);
//int32_t* makeVelSineWaveform(int32_t posAmplitude, uint16_t numValues);
int32_t* makeRampWaveform(int32_t amplitude, uint16_t numValues);
int32_t* makeWideVelPulseWaveform(int32_t amplitude, uint16_t numValues);
int32_t* makeZeroWaveform(uint16_t numValues);
int32_t* makeConstWaveform(int32_t value, uint16_t numValues);
void allocateArbitraryWaveform(uint16_t nPts);
void setWaveformValue(uint16_t value, uint16_t whichWaveform);



#endif	/* CONFIG_SLAVE_H */

