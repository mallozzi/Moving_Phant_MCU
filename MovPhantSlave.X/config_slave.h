

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

void setOutputWaveform(int32_t* waveformArray);

void setUpWaveform();
void designPosSineWaveform(int16_t mmDisplacementPP);
void designRampWaveform(int16_t mmStepSize);
//void designVelSineWaveform(int16_t mmDisplacementPP);
//void designVelPulseWaveform(int16_t mmDisplacement);

int32_t* makePosSineWaveform(int32_t amplitudeEnc, uint16_t numValues);
//int32_t* makeVelSineWaveform(int32_t posAmplitude, uint16_t numValues);
int32_t* makeRampWaveform(int32_t amplitude, uint16_t numValues);
int32_t* makeWideVelPulseWaveform(int32_t amplitude, uint16_t numValues);
int32_t* makeZeroWaveform(uint16_t numValues);
int32_t* makeConstWaveform(int32_t value, uint16_t numValues);
//void calcNumPoints();



#endif	/* CONFIG_SLAVE_H */

