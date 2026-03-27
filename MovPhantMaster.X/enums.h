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

//NOTE: THESE ENUMS SHOULD EXACTLY MATCH THE ENUMS IN THE SECONDARY CORE

typedef enum {
    REG_COMMAND,
    REG_VARIABLE,           // 16-bit variable
    REG_VARIABLE_32,        // 32-bit variable
    REG_BOOLVAR,            // boolean variable
    REG_PING                // ping request    
} Register;

typedef enum {
    START_MOTION,
    STOP_MOTION,
    READ_QUAD_ENC,        
    SEND_VARIABLES
} Command;

typedef enum {
    PWM1_CYCLES,
    PWM2_CYCLES,
    MAX_PWM_INT,
    NUM_ARRAY_VALS,
    WAVEFORM_UPDATE_PERIOD,
    WAVEFORM_TYPE,
    MOTION_AMPLITUDE_MM1,
    MOTION_AMPLITUDE_MM2,
    REVERSE_DIRECTION1,
    REVERSE_DIRECTION2,
    FREQ_USER,
    ENCODER_STEPS_PER_MM,
    QUAD_ENC_VEL,
    PING_VAR,
    WAVEFORM_NPTS,
    WAVEFORM_DATA_VAL_1,
    WAVEFORM_DATA_VAL_2
} Variable;

typedef enum {
    OUTPUT1_ENABLED,
    OUTPUT2_ENABLED,
    USER_MOT1_ENABLED,
    USER_MOT2_ENABLED,
    PLAY_SINGLE_WAVEFORM
} BoolVariable;

typedef enum {
    WAVEFORM_TIMESTEP_MICROS,
    DISPLACEMENT1_DEMAND,
    DISPLACEMENT2_DEMAND,
    QUAD_ENC_POS
}Variable32;


#endif	/* ENUMS_H */

