// Manages sending parameters back and forth to secondary processor

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef PARAM_MANAGER_H
#define	PARAM_MANAGER_H

#include <xc.h> // include processor files - each processor file is guarded.  


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

// These enums must be identical on master and secondary

// Registers
enum reg {
    COMMAND,
    OUTPUT_ENABLED,
    PWM_MOT1
};

// Commands
enum commands {
    START_MOT1,
    ENABLE_OUTPUT,
    DISABLE_OUTPUT
};

#endif	/* PARAM_MANAGER_H */

