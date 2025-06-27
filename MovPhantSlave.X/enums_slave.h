// This file defines enum variables that are common to both primary and secondary cores. Thus it is included in both projects.  


#ifndef ENUMS_H
#define	ENUMS_H

#include <xc.h> // include processor files - each processor file is guarded.  


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */



typedef enum {
    REG_COMMAND,
    REG_VARIABLE,           // 16-bit variable
    REG_VARIABLE_32,        // 32-bit variable
    REG_BOOLVAR             // boolean variable
} Register;

typedef enum {
    START_MOTION,
    STOP_MOTION,
    SEND_VARIABLES
} Command;

typedef enum {
    PWM1_CYCLES,
    MAX_PWM_INT,
    NUM_ARRAY_VALS,
    WAVEFORM_UPDATE_PERIOD,
    WAVEFORM_TYPE,
    MOTION_AMPLITUDE_MM,
    REVERSE_DIRECTION,
    FREQ_USER,
    ENCODER_STEPS_PER_MM
} Variable;

typedef enum {
    OUTPUT_ENABLED,
    PLAY_SINGLE_WAVEFORM
} BoolVariable;

typedef enum {
    WAVEFORM_TIMESTEP_MICROS,
    DISPLACEMENT_DEMAND
}Variable32;


#endif	/* ENUMS_H */

